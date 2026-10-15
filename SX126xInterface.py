##########################################################
# SX126xInterface.py                                     #
#                                                        #
# A Reticulum custom interface that drives SX1262/SX1268 #
# LoRa chips directly over SPI on Linux SBCs (Raspberry  #
# Pi, Orange Pi, etc). No separate RNode MCU required.   #
#                                                        #
# Implements:                                            #
#   - Direct SPI control via LoRaRF-Python library       #
#   - CSMA/CA collision avoidance (p-persistent)         #
#   - RNode-compatible split-packet framing for the      #
#     full Reticulum 500-byte MTU                        #
#   - Configurable pin mapping for various Pi HATs       #
#                                                        #
# Default pin config is for the MeshAdv Pi HAT v1.1:     #
#   CS=21, IRQ=16, Busy=20, Reset=18, TXen=13, RXen=12  #
#                                                        #
# Place this file in ~/.reticulum/interfaces/ and add    #
# an interface config block to ~/.reticulum/config       #
#                                                        #
# Example config:                                        #
#                                                        #
#   [[MeshAdv LoRa]]                                     #
#     type = SX126xInterface                             #
#     interface_enabled = True                            #
#     frequency = 915000000                              #
#     bandwidth = 125000                                 #
#     spreadingfactor = 8                                #
#     codingrate = 5                                     #
#     txpower = 22                                       #
#     spi_bus = 0                                        #
#     spi_cs = 0                                         #
#     pin_cs = 21                                        #
#     pin_irq = 16                                       #
#     pin_busy = 20                                      #
#     pin_reset = 18                                     #
#     pin_txen = 13                                      #
#     pin_rxen = 12                                      #
#     dio3_tcxo_voltage = 1.8                            #
#                                                        #
# License: MIT                                           #
##########################################################

# Custom interfaces loaded by Reticulum via exec() have
# "Interface" and "RNS" injected into their globals.
# We import additional stdlib modules we need here.
import threading
import time
import math
import os
import random


class SX126xInterface(Interface):
    MAX_CHUNK         = 32768
    DEFAULT_IFAC_SIZE = 8

    FREQ_MIN = 137000000
    FREQ_MAX = 1020000000

    # SX1262 maximum single LoRa frame payload
    LORA_MAX_PAYLOAD  = 255
    # We prepend a 1-byte split-frame header, so usable per frame
    FRAME_PAYLOAD_MAX = 254

    # Split-packet framing flags (RNode-compatible)
    FLAG_SPLIT = 0x01

    RSSI_OFFSET = 157

    def __init__(self, owner, configuration):
        # Check for the LoRaRF dependency early
        import importlib
        if importlib.util.find_spec("LoRaRF") is None:
            RNS.log("The SX126xInterface requires the LoRaRF library to be installed.", RNS.LOG_CRITICAL)
            RNS.log("You can install it with the command: pip install LoRaRF", RNS.LOG_CRITICAL)
            RNS.panic()

        super().__init__()

        # Parse configuration through the standard helper
        c = Interface.get_config_obj(configuration)
        name = c["name"]

        # LoRa radio parameters
        frequency = int(c["frequency"]) if "frequency" in c else 915000000
        bandwidth = int(c["bandwidth"]) if "bandwidth" in c else 125000
        txpower   = int(c["txpower"])   if "txpower" in c else 22
        sf        = int(c["spreadingfactor"]) if "spreadingfactor" in c else 8
        cr        = int(c["codingrate"]) if "codingrate" in c else 5

        # SPI bus configuration
        spi_bus   = int(c["spi_bus"])   if "spi_bus" in c else 0
        spi_cs    = int(c["spi_cs"])    if "spi_cs" in c else 0

        # GPIO pin configuration (BCM numbering)
        # MeshAdv Pi HAT v1.1 defaults
        pin_cs    = int(c["pin_cs"])    if "pin_cs" in c else 21
        pin_irq   = int(c["pin_irq"])   if "pin_irq" in c else 16
        pin_busy  = int(c["pin_busy"])  if "pin_busy" in c else 20
        pin_reset = int(c["pin_reset"]) if "pin_reset" in c else 18
        pin_txen  = int(c["pin_txen"])  if "pin_txen" in c else 13
        pin_rxen  = int(c["pin_rxen"])  if "pin_rxen" in c else 12

        # DIO3 TCXO voltage (set to 0 or None to disable)
        dio3_tcxo = float(c["dio3_tcxo_voltage"]) if "dio3_tcxo_voltage" in c else 1.8

        # Sync word: 0x12 = private/RNode, 0x34 = public/LoRaWAN
        # Accept both "0x12" and "18" style values
        if "sync_word" in c:
            sw = c["sync_word"].strip()
            sync_word = int(sw, 16) if sw.startswith("0x") or sw.startswith("0X") else int(sw)
        else:
            sync_word = 0x12

        # CSMA/CA parameters
        self.csma_p           = float(c["csma_p"])           if "csma_p" in c else 0.1
        self.csma_slot_ms     = float(c["csma_slot_ms"])     if "csma_slot_ms" in c else 50.0
        self.csma_max_backoff = int(c["csma_max_backoff"])    if "csma_max_backoff" in c else 5
        # Channel is considered busy when instantaneous RSSI is above this
        # level (dBm). Only meaningful while the chip is in RX mode; the
        # transmit loop keeps it in RX between frames (see process_outgoing).
        self.csma_rssi_threshold = float(c["csma_rssi_threshold"]) if "csma_rssi_threshold" in c else -90.0

        # Airtime limits (optional, percent 0-100)
        self.st_alock = float(c["airtime_limit_short"]) if "airtime_limit_short" in c else None
        self.lt_alock = float(c["airtime_limit_long"])  if "airtime_limit_long"  in c else None

        # HW_MTU must match what RNodeInterface uses (508)
        self.HW_MTU = 508

        self.owner       = owner
        self.name        = name
        self.online      = False
        self.detached    = False
        self.IN          = True
        self.OUT         = True

        self.frequency   = frequency
        self.bandwidth   = bandwidth
        self.txpower     = txpower
        self.sf          = sf
        self.cr          = cr
        self.sync_word   = sync_word
        self.spi_bus     = spi_bus
        self.spi_cs      = spi_cs
        self.pin_cs      = pin_cs
        self.pin_irq     = pin_irq
        self.pin_busy    = pin_busy
        self.pin_reset   = pin_reset
        self.pin_txen    = pin_txen
        self.pin_rxen    = pin_rxen
        self.dio3_tcxo   = dio3_tcxo

        self.bitrate     = 0
        self.r_stat_rssi = None
        self.r_stat_snr  = None

        self.packet_queue    = []
        # single lock guarding the radio: held across the whole TX sequence
        # and across every RX status/drain operation, so the chip is never
        # RX-continuous while a TX frame is being written to its FIFO
        self.tx_lock         = threading.Lock()
        self.interface_ready = True
        self.announce_rate_target = None

        # Airtime tracking
        self._airtime_short_window = 15.0         # seconds
        self._airtime_long_window  = 60.0 * 60.0  # 1 hour
        self._airtime_short_sum    = 0.0
        self._airtime_long_sum     = 0.0
        self._airtime_short_start  = time.time()
        self._airtime_long_start   = time.time()

        # TX completion tracking. Every outcome is counted and logged so
        # missed TxDone IRQ edges can be correlated with throughput drops
        # during large transfers (see Prns issue #246 for the failure
        # pattern this diagnoses: chip back in standby, TxDone IRQ lost).
        self._tx_frames_ok         = 0  # completed via TxDone IRQ
        self._tx_frames_missed_irq = 0  # healed: IRQ missed/late, frame presumed sent
        self._tx_frames_failed     = 0  # could not be resolved as sent

        # Split-packet reassembly state. A legitimate split pair arrives
        # back-to-back (worst case ~100 ms at SF12), so the pending window
        # only needs to cover receive-side processing delays; keeping it
        # short also keeps the random 4-bit seq collision window small.
        self._rx_fragments = {}   # seq_id -> (timestamp, data)
        self._frag_timeout = 1.0  # seconds a pending fragment may wait

        # Validate configuration
        validcfg = True
        if frequency < SX126xInterface.FREQ_MIN or frequency > SX126xInterface.FREQ_MAX:
            RNS.log("Invalid frequency configured for "+str(self), RNS.LOG_ERROR)
            validcfg = False
        if txpower < -9 or txpower > 22:
            RNS.log("Invalid TX power configured for "+str(self), RNS.LOG_ERROR)
            validcfg = False
        if bandwidth not in [7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000]:
            RNS.log("Invalid bandwidth configured for "+str(self), RNS.LOG_ERROR)
            validcfg = False
        if sf < 5 or sf > 12:
            RNS.log("Invalid spreading factor configured for "+str(self), RNS.LOG_ERROR)
            validcfg = False
        if cr < 5 or cr > 8:
            RNS.log("Invalid coding rate configured for "+str(self), RNS.LOG_ERROR)
            validcfg = False

        if not validcfg:
            raise ValueError("The configuration for "+str(self)+" contains errors, interface is offline")

        try:
            self._init_radio()
            self._update_bitrate()
            self.online = True
            RNS.log(str(self)+" is now online", RNS.LOG_NOTICE)
            RNS.log("  Frequency : "+str(self.frequency/1e6)+" MHz", RNS.LOG_VERBOSE)
            RNS.log("  Bandwidth : "+str(self.bandwidth/1e3)+" kHz", RNS.LOG_VERBOSE)
            RNS.log("  TX Power  : "+str(self.txpower)+" dBm", RNS.LOG_VERBOSE)
            RNS.log("  SF        : "+str(self.sf), RNS.LOG_VERBOSE)
            RNS.log("  CR        : 4/"+str(self.cr), RNS.LOG_VERBOSE)
            RNS.log("  On-air    : "+str(round(self.bitrate/1000.0, 2))+" kbps", RNS.LOG_VERBOSE)

            # Start the receive loop
            self._rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._rx_thread.start()

        except Exception as e:
            RNS.log("Could not initialise SX126x radio for "+str(self)+": "+str(e), RNS.LOG_ERROR)
            raise

    def _init_radio(self):
        """Initialise the SX1262/SX1268 radio chip over SPI."""
        from LoRaRF import SX126x

        self.radio = SX126x()

        RNS.log(str(self)+" Initialising SX126x on SPI bus "+str(self.spi_bus)+" CS "+str(self.spi_cs), RNS.LOG_VERBOSE)

        if not self.radio.begin(
            bus=self.spi_bus,
            cs=self.spi_cs,
            reset=self.pin_reset,
            busy=self.pin_busy,
            irq=self.pin_irq,
            txen=self.pin_txen,
            rxen=self.pin_rxen
        ):
            raise IOError("SX126x radio not detected or failed to initialise")

        # Enable DIO3 as TCXO control if configured
        # The MeshAdv Pi HAT E22 modules use TCXO with DIO3
        if self.dio3_tcxo and self.dio3_tcxo > 0:
            tcxo_map = {
                1.6: SX126x.DIO3_OUTPUT_1_6,
                1.7: SX126x.DIO3_OUTPUT_1_7,
                1.8: SX126x.DIO3_OUTPUT_1_8,
                2.2: SX126x.DIO3_OUTPUT_2_2,
                2.4: SX126x.DIO3_OUTPUT_2_4,
                2.7: SX126x.DIO3_OUTPUT_2_7,
                3.0: SX126x.DIO3_OUTPUT_3_0,
                3.3: SX126x.DIO3_OUTPUT_3_3,
            }
            closest = min(tcxo_map.keys(), key=lambda v: abs(v - self.dio3_tcxo))
            self.radio.setDio3TcxoCtrl(tcxo_map[closest], SX126x.TCXO_DELAY_2_5)
            RNS.log(str(self)+" TCXO enabled at "+str(closest)+"V via DIO3", RNS.LOG_VERBOSE)

        # Enable DIO2 as RF switch control (used by E22 modules)
        self.radio.setDio2RfSwitch(True)

        # Use DC-DC regulator for better efficiency
        self.radio.setRegulator(SX126x.REGULATOR_DC_DC)

        # Configure frequency
        self.radio.setFrequency(self.frequency)

        # Configure TX power (SX1262)
        self.radio.setTxPower(self.txpower, SX126x.TX_POWER_SX1262)

        # Set RX gain to boosted for best sensitivity
        # RX_GAIN_BOOSTED = 0x96, RX_GAIN_POWER_SAVING = 0x94
        try:
            self.radio.setRxGain(SX126x.RX_GAIN_BOOSTED)
        except AttributeError:
            self.radio.setRxGain(0x96)

        # Configure LoRa modulation parameters
        # LoRaRF.setLoRaModulation expects: sf, bw_in_hz, cr, ldro
        ldro = self._should_use_ldro()
        self.radio.setLoRaModulation(self.sf, self.bandwidth, self.cr, ldro)

        # Configure LoRa packet parameters
        # Explicit header, preamble 8 symbols, max payload 255, CRC on, no IQ invert
        try:
            header_type = SX126x.HEADER_EXPLICIT
        except AttributeError:
            header_type = 0x00

        self.radio.setLoRaPacket(
            header_type,
            8,                                    # preamble symbols
            SX126xInterface.LORA_MAX_PAYLOAD,     # max payload length
            True,                                 # CRC enabled
            False                                 # no IQ invert
        )

        # Set sync word
        self.radio.setSyncWord(self.sync_word)

        RNS.log(str(self)+" SX126x radio initialised successfully", RNS.LOG_VERBOSE)

    def _should_use_ldro(self):
        """Determine if Low Data Rate Optimization should be enabled."""
        symbol_time = (2**self.sf) / self.bandwidth
        return symbol_time > 0.016

    def _update_bitrate(self):
        """
        Calculate the on-air bitrate.
        Uses the same formula as RNodeInterface.updateBitrate():
          bitrate = sf * ( (4.0/cr) / (2^sf / (bw/1000)) ) * 1000
        """
        try:
            self.bitrate = self.sf * ( (4.0/self.cr) / (math.pow(2, self.sf)/(self.bandwidth/1000)) ) * 1000
        except Exception:
            self.bitrate = 0

    def _calculate_toa(self, payload_len):
        """Calculate time-on-air in seconds for a given payload length."""
        bw = self.bandwidth
        sf = self.sf
        cr = self.cr
        preamble_len = 8
        has_crc = True
        explicit_header = True
        ldro = self._should_use_ldro()

        t_sym = (2**sf) / bw

        # Preamble duration
        t_preamble = (preamble_len + 4.25) * t_sym

        # Payload symbol count
        de = 1 if ldro else 0
        ih = 0 if explicit_header else 1
        crc_bits = 16 if has_crc else 0

        num = max(8 * payload_len - 4 * sf + 28 + crc_bits - 20 * ih, 0)
        denom = 4 * (sf - 2 * de)
        n_payload = 8 + math.ceil(num / denom) * cr

        t_payload = n_payload * t_sym
        return t_preamble + t_payload

    # Override: LoRa interfaces should not do ingress limiting
    def should_ingress_limit(self):
        return False

    #############################
    # CSMA/CA Implementation    #
    #############################

    def _channel_is_clear(self):
        """Check if the channel appears clear using RSSI-based carrier sense."""
        try:
            # rssiInst() samples the channel right now, but is only valid
            # while the chip is actually in RX mode — in STANDBY the RSSI
            # register reads 0 (i.e. 0 dBm, "busy" at any negative
            # threshold). process_outgoing re-enters RX before every CSMA
            # check so the reading is live. (packetRssi() would be the RSSI
            # of the *last received* packet — stale — and is unsuitable.)
            rssi = self.radio.rssiInst()
            return rssi < self.csma_rssi_threshold
        except Exception:
            return True

    def _csma_wait(self, frame_len):
        """
        Perform p-persistent CSMA/CA before transmitting.
        Similar approach to RNode firmware CSMA, adapted for userspace.
        """
        slot_time = self.csma_slot_ms / 1000.0
        attempt = 0
        busy_attempts = 0

        while attempt < self.csma_max_backoff * 10:
            if self._channel_is_clear():
                if random.random() < self.csma_p:
                    return  # Proceed with transmission
                else:
                    time.sleep(slot_time)
            else:
                busy_attempts += 1
                backoff_exp = min(attempt // 2, self.csma_max_backoff)
                max_slots = 2**backoff_exp
                wait_slots = random.randint(0, max_slots)
                time.sleep(wait_slots * slot_time)

            attempt += 1

        # Note: reaching this point is NOT a carrier-sense failure — p-
        # persistent CSMA simply did not roll a transmit within the
        # attempt cap. Distinguish the two causes in the log: at
        # csma_p=0.1 with a 10-attempt cap, ~35% of transmissions log
        # this on a perfectly clear channel (0.9^10).
        if busy_attempts:
            RNS.log(str(self)+" CSMA channel busy in "+str(busy_attempts)+" of "+str(attempt)+" attempts, transmitting", RNS.LOG_DEBUG)
        else:
            RNS.log(str(self)+" CSMA p-persistent lottery exhausted after "+str(attempt)+" clear attempts, transmitting", RNS.LOG_DEBUG)

    #############################
    # Split-Packet Framing      #
    #############################

    def _make_frames(self, data):
        """
        Split data into LoRa frames with RNode-compatible framing.

        Each frame gets a 1-byte header:
          - Upper nibble: 4-bit random sequence ID
          - Bit 0 (FLAG_SPLIT): 1 if packet is split across 2 frames

        Packets <= 254 bytes: single frame, FLAG_SPLIT=0
        Packets > 254 bytes:  split into 2 frames, FLAG_SPLIT=1
        """
        max_payload = SX126xInterface.FRAME_PAYLOAD_MAX
        frames = []

        if len(data) <= max_payload:
            seq_id = (random.randint(0, 15) << 4)
            header = seq_id & 0xF0  # FLAG_SPLIT = 0
            frames.append(bytes([header]) + data)
        else:
            seq_id = (random.randint(0, 15) << 4)
            header = (seq_id & 0xF0) | SX126xInterface.FLAG_SPLIT

            split_point = max_payload
            frame1 = bytes([header]) + data[:split_point]
            frame2 = bytes([header]) + data[split_point:]
            frames.append(frame1)
            frames.append(frame2)

        return frames

    def _reassemble(self, frame_data):
        """
        Reassemble received frames using the split-packet protocol.
        Returns the reassembled packet data, or None if waiting for
        the second fragment.
        """
        if len(frame_data) < 1:
            return None

        header = frame_data[0]
        seq_id = header & 0xF0
        is_split = bool(header & SX126xInterface.FLAG_SPLIT)
        payload = frame_data[1:]

        if not is_split:
            # a whole frame with this sequence means any pending first
            # fragment for the same seq can never be completed; drop it so
            # a later unrelated split frame-2 cannot finish it into garbage
            self._rx_fragments.pop(seq_id, None)
            return payload

        now = time.time()

        # Clean stale fragments
        stale = [k for k, (ts, _) in self._rx_fragments.items()
                 if now - ts > self._frag_timeout]
        for k in stale:
            del self._rx_fragments[k]

        if len(payload) == SX126xInterface.FRAME_PAYLOAD_MAX:
            # frame1: a split's first frame always carries exactly
            # FRAME_PAYLOAD_MAX payload bytes. A pending entry with the same
            # seq is a frame2 that arrived first, so concatenate in order.
            if seq_id in self._rx_fragments:
                ts, second_payload = self._rx_fragments.pop(seq_id)
                if now - ts <= self._frag_timeout:
                    return payload + second_payload
                # pending fragment is stale (from another, unrelated split
                # that happened to reuse the 4-bit seq): start fresh
            self._rx_fragments[seq_id] = (now, payload)
            return None
        else:
            # frame2: only ever completes a pending frame1 with the same seq
            if seq_id in self._rx_fragments:
                ts, first_payload = self._rx_fragments.pop(seq_id)
                if now - ts <= self._frag_timeout:
                    return first_payload + payload
                # stale fragment: the two halves belong to different
                # packets, dropping both is the only safe outcome
            # orphan frame2 (frame1 lost on air): nothing to reassemble,
            # and storing it would only let an unrelated frame1 with the
            # same random seq complete into a garbage packet
            return None

    #############################
    # TX / RX                   #
    #############################

    def _transmit_frame(self, frame):
        """Transmit a single LoRa frame via SPI."""
        from LoRaRF import SX126x

        self.radio.beginPacket()
        self.radio.put(frame)
        self.radio.endPacket()
        # Wait for the TX-done GPIO interrupt. The chip's airtime is short
        # (~35 ms worst case at SF12/125 kHz), but on a loaded Pi Zero 2 W
        # the RPi.GPIO callback thread can be CPU-starved for seconds, so a
        # short timeout "misses" the TX-done edge and status() reports the
        # stale STATUS_TX_WAIT (1) even though the frame went out. 10 s
        # matches the original working value and survives the starvation.
        sent = self.radio.wait(timeout=10)

        if sent and self.radio.status() == SX126x.STATUS_TX_DONE:
            self._tx_frames_ok += 1
            return True

        status = self.radio.status()
        if status == SX126x.STATUS_TX_TIMEOUT:
            # a genuine chip-side TX timeout, not a missed edge
            self._tx_frames_failed += 1
            RNS.log(str(self)+" TX failed with status "+str(status)+" (TX failures: "+str(self._tx_frames_failed)+", missed IRQ: "+str(self._tx_frames_missed_irq)+")", RNS.LOG_WARNING)
            return False

        # wait() timed out and the driver state is still TX_WAIT: the
        # TxDone edge may have been missed or late while the chip already
        # finished. Ask the driver to re-check the chip's IRQ register
        # instead of paying a 10 s stall plus a retransmit for a frame
        # that went out. If the frame was not actually sent, the RNS link
        # layer retransmits it (packets are only acknowledged on
        # reception), so a "presumed sent" verdict cannot lose data.
        if self.radio.healTxDone():
            self._tx_frames_missed_irq += 1
            RNS.log(str(self)+" TxDone IRQ missed or late, frame presumed sent (missed IRQ: "+str(self._tx_frames_missed_irq)+", TX failures: "+str(self._tx_frames_failed)+")", RNS.LOG_WARNING)
            return True

        self._tx_frames_failed += 1
        RNS.log(str(self)+" TX failed with status "+str(status)+" (TX failures: "+str(self._tx_frames_failed)+", missed IRQ: "+str(self._tx_frames_missed_irq)+")", RNS.LOG_WARNING)
        return False

    def tx_stats(self):
        """
        Return TX completion counters as a dict:
          tx_ok         frames completed via the TxDone IRQ
          tx_missed_irq frames healed after a missed/late TxDone IRQ edge
          tx_failed     frames that could not be resolved as sent

        If tx_missed_irq grows and the growth correlates with throughput
        drops (e.g. NomadNet page fetches stalling), the radio's SPI
        wiring or the Pi's GPIO callback thread is the culprit.
        """
        return {
            "tx_ok": self._tx_frames_ok,
            "tx_missed_irq": self._tx_frames_missed_irq,
            "tx_failed": self._tx_frames_failed,
        }

    def _drain_pending_rx(self):
        """
        Read a pending received frame out of the chip FIFO before we go
        deaf for TX. Must be called with self.tx_lock held (so the RX loop
        cannot drain concurrently). Returns frame bytes, or None.
        """
        from LoRaRF import SX126x
        try:
            if self.radio.status() == SX126x.STATUS_RX_DONE:
                # drain() reads the full payload atomically in the driver
                frame_data = self.radio.drain()
                if frame_data:
                    return frame_data
        except Exception as e:
            RNS.log(str(self)+" Error draining pending RX frame: "+str(e), RNS.LOG_ERROR)
        return None

    def process_incoming(self, data):
        """Called when a complete packet has been received and reassembled."""
        self.rxb += len(data)
        self.owner.inbound(data, self)
        self.r_stat_rssi = None
        self.r_stat_snr  = None

    def process_outgoing(self, data):
        """Called by Reticulum to send a packet out this interface."""
        if not self.online:
            return

        datalen = len(data)
        if datalen > self.HW_MTU:
            RNS.log(str(self)+" Dropping oversized packet ("+str(datalen)+" > "+str(self.HW_MTU)+")", RNS.LOG_ERROR)
            return

        pending_frame = None
        with self.tx_lock:
            try:
                # A frame may have been received (IRQ raised) while we were
                # waiting for the lock. We are about to make the chip deaf
                # for TX, so pull it out of the FIFO before we start.
                pending_frame = self._drain_pending_rx()

                frames = self._make_frames(data)

                for i, frame in enumerate(frames):
                    # After each TX the chip falls back to STANDBY, where the
                    # instantaneous RSSI register reads 0 — misread by
                    # _channel_is_clear() as a 0 dBm (very strong) signal,
                    # i.e. "busy" at any negative threshold, which produced
                    # spurious "CSMA gave up" bursts between split frames.
                    # Re-enter RX continuous before every carrier-sense check
                    # (a no-op via the driver's guard when already in RX) and
                    # let the RX chain settle so rssiInst() is live.
                    self._start_rx()
                    time.sleep(0.001)
                    self._csma_wait(len(frame))

                    toa = self._calculate_toa(len(frame))
                    self._track_airtime(toa)

                    if self.st_alock and self._get_short_airtime_pct() > self.st_alock:
                        RNS.log(str(self)+" Short-term airtime limit exceeded, queueing", RNS.LOG_DEBUG)
                        self.packet_queue.append(data)
                        return
                    if self.lt_alock and self._get_long_airtime_pct() > self.lt_alock:
                        RNS.log(str(self)+" Long-term airtime limit exceeded, dropping", RNS.LOG_WARNING)
                        return

                    success = self._transmit_frame(frame)
                    if not success:
                        RNS.log(str(self)+" Frame transmission failed", RNS.LOG_WARNING)
                        self._start_rx()
                        return

                self.txb += datalen

                # Re-enter RX mode after TX
                self._start_rx()

            except Exception as e:
                RNS.log(str(self)+" Error during transmission: "+str(e), RNS.LOG_ERROR)
                try:
                    self._start_rx()
                except Exception:
                    pass

        # Deliver the drained frame outside the lock: reassembly and
        # owner.inbound() can take milliseconds and must not block TX
        if pending_frame is not None:
            packet = self._reassemble(pending_frame)
            if packet is not None:
                self.process_incoming(packet)

    def _start_rx(self):
        """Put the radio into continuous receive mode."""
        from LoRaRF import SX126x
        self.radio.request(SX126x.RX_CONTINUOUS)

    def _receive_loop(self):
        """Background thread that continuously receives LoRa frames."""
        from LoRaRF import SX126x

        RNS.log(str(self)+" Receive loop started", RNS.LOG_VERBOSE)
        self._start_rx()

        while self.online and not self.detached:
            try:
                # wait() outside the radio lock: it blocks for up to 1 s and
                # must not hold the lock that TX needs. It only returns True
                # once the IRQ callback has raised an event (or timeout).
                if self.radio.wait(timeout=1):
                    frame_data = None
                    status = None
                    # status()/drain() under the lock: they consume the IRQ
                    # event flag and move the driver's buffer pointer, so
                    # they must be atomic w.r.t. TX and _drain_pending_rx()
                    with self.tx_lock:
                        status = self.radio.status()
                        if status == SX126x.STATUS_RX_DONE:
                            frame_data = self.radio.drain()
                            try:
                                self.r_stat_rssi = self.radio.packetRssi()
                                self.r_stat_snr  = self.radio.snr()
                            except Exception:
                                pass

                    # reassembly + owner.inbound() outside the lock: they
                    # can take milliseconds and must not block TX
                    if frame_data:
                        RNS.log(str(self)+" Received frame ("+str(len(frame_data))+" bytes, RSSI: "+str(self.r_stat_rssi)+", SNR: "+str(self.r_stat_snr)+")", RNS.LOG_DEBUG)
                        packet = self._reassemble(frame_data)
                        if packet is not None:
                            self.process_incoming(packet)

                    elif status == SX126x.STATUS_CRC_ERR:
                        RNS.log(str(self)+" CRC error on received frame", RNS.LOG_DEBUG)

                    elif status == SX126x.STATUS_HEADER_ERR:
                        RNS.log(str(self)+" Header error on received frame", RNS.LOG_DEBUG)

                    if status != SX126x.STATUS_RX_DONE:
                        time.sleep(0.01)

            except Exception as e:
                RNS.log(str(self)+" Error in receive loop: "+str(e), RNS.LOG_ERROR)
                time.sleep(1.0)
                try:
                    self._start_rx()
                except Exception:
                    pass

        RNS.log(str(self)+" Receive loop ended", RNS.LOG_VERBOSE)

    #############################
    # Airtime Tracking          #
    #############################

    def _track_airtime(self, toa_seconds):
        """Track cumulative airtime for rate limiting."""
        now = time.time()
        self._airtime_short_sum += toa_seconds
        self._airtime_long_sum  += toa_seconds

        if now - self._airtime_short_start > self._airtime_short_window:
            self._airtime_short_sum   = toa_seconds
            self._airtime_short_start = now
        if now - self._airtime_long_start > self._airtime_long_window:
            self._airtime_long_sum   = toa_seconds
            self._airtime_long_start = now

    def _get_short_airtime_pct(self):
        elapsed = max(time.time() - self._airtime_short_start, 0.001)
        window  = min(elapsed, self._airtime_short_window)
        return (self._airtime_short_sum / window) * 100.0

    def _get_long_airtime_pct(self):
        elapsed = max(time.time() - self._airtime_long_start, 0.001)
        window  = min(elapsed, self._airtime_long_window)
        return (self._airtime_long_sum / window) * 100.0

    #############################
    # Interface Lifecycle       #
    #############################

    def detach(self):
        """Shut down the interface."""
        self.detached = True
        self.online   = False
        try:
            self.radio.end()
        except Exception:
            pass
        RNS.log(str(self)+" detached", RNS.LOG_NOTICE)

    def __str__(self):
        return "SX126xInterface["+self.name+"]"


# -------------------------------------------------------
# CRITICAL: Register the interface class so Reticulum's
# external-interface loader can find it. Without this
# line the module will load but the interface will not
# be instantiated.
# -------------------------------------------------------
interface_class = SX126xInterface
