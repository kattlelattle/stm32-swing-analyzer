#swing analyzer - pc side of the uart protocol
#decodes the board's packets and has a copy of the swing detector for re-running saved logs
#packet: 0xAA 0x55 | type | len | payload | crc8(type, len, payload), has to match firmware/App/Inc/frame.h

from __future__ import annotations

import math
import random
import struct
from dataclasses import dataclass

SYNC = b"\xAA\x55"
T_SAMPLE, T_STATUS, T_SWING = 0x01, 0x02, 0x03

_SAMPLE = struct.Struct("<II6hB")          #21 bytes
_STATUS = struct.Struct("<9IHBBBB2x")      #44 bytes
_SWING = struct.Struct("<4IHB")            #19 bytes
_LEN = {T_SAMPLE: _SAMPLE.size, T_STATUS: _STATUS.size, T_SWING: _SWING.size}

FLAG_GYRO_CLIPPED = 0x01
FLAG_ACC_CLIPPED = 0x02

BMI_STATUS_TEXT = {
    0: "OK",
    1: "SPI error (HAL call failed)",
    2: "wrong chip ID - check wiring, 3.3 V power and CS",
    3: "config upload not accepted (INTERNAL_STATUS != 1)",
    4: "a setting did not stick (register read back a different value)",
    5: "chip ID OK, but bmi270_config.c has no config array",
    6: "sensor reported a fatal error (ERR_REG bit 0)",
}


def seq_gap(prev: int, cur: int):
    #missing samples between two sequence numbers, None if the board was reset
    if cur <= prev:
        return None
    return cur - prev - 1


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


@dataclass
class Sample:
    seq: int
    t_cyc: int
    acc: tuple
    gyr: tuple
    flags: int


@dataclass
class Status:
    core_hz: int
    uptime_ms: int
    ticks: int
    missed_reads: int
    ring_overflows: int
    tx_dropped: int
    spi_errors: int
    swing_count: int
    rate_mhz: int
    gyro_fs_dps: int
    acc_fs_g: int
    bmi_status: int
    chip_id: int
    bmi_diag: int = 0


@dataclass
class Swing:
    swing_id: int
    start_seq: int
    peak_seq: int
    end_seq: int
    peak_mag_raw: int
    flags: int

    def peak_dps(self, gyro_fs_dps: int = 2000) -> float:
        return self.peak_mag_raw * gyro_fs_dps / 32768.0


#encoding

def _frame(ftype: int, payload: bytes) -> bytes:
    body = bytes([ftype, len(payload)]) + payload
    return SYNC + body + bytes([crc8(body)])


def encode_sample(s: Sample) -> bytes:
    return _frame(T_SAMPLE, _SAMPLE.pack(s.seq & 0xFFFFFFFF, s.t_cyc & 0xFFFFFFFF,
                                         *s.acc, *s.gyr, s.flags))


def encode_status(st: Status) -> bytes:
    return _frame(T_STATUS, _STATUS.pack(st.core_hz, st.uptime_ms, st.ticks, st.missed_reads,
                                         st.ring_overflows, st.tx_dropped, st.spi_errors,
                                         st.swing_count, st.rate_mhz, st.gyro_fs_dps,
                                         st.acc_fs_g, st.bmi_status, st.chip_id, st.bmi_diag))


def encode_swing(sw: Swing) -> bytes:
    return _frame(T_SWING, _SWING.pack(sw.swing_id, sw.start_seq, sw.peak_seq, sw.end_seq,
                                       sw.peak_mag_raw, sw.flags))


#decoding

class Parser:
    #feed it bytes in any chunk sizes and it returns the packets it found
    #corrupted data gets skipped up to the next 0xAA 0x55 header with a good crc

    def __init__(self) -> None:
        self.buf = bytearray()
        self.bad_crc = 0
        self.skipped_bytes = 0

    def feed(self, data: bytes) -> list:
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                keep = 1 if self.buf[-1:] == b"\xAA" else 0
                self.skipped_bytes += len(self.buf) - keep
                del self.buf[: len(self.buf) - keep]
                return out
            if i > 0:
                self.skipped_bytes += i
                del self.buf[:i]
            if len(self.buf) < 4:
                return out
            ftype, ln = self.buf[2], self.buf[3]
            if _LEN.get(ftype) != ln:          #not a real header, skip the 0xAA
                self.skipped_bytes += 1
                del self.buf[:1]
                continue
            total = 5 + ln
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2: 4 + ln])
            if crc8(body) != self.buf[4 + ln]:
                self.bad_crc += 1
                self.skipped_bytes += 1
                del self.buf[:1]
                continue
            payload = body[2:]
            del self.buf[:total]
            if ftype == T_SAMPLE:
                v = _SAMPLE.unpack(payload)
                out.append(Sample(v[0], v[1], v[2:5], v[5:8], v[8]))
            elif ftype == T_STATUS:
                out.append(Status(*_STATUS.unpack(payload)))
            else:
                out.append(Swing(*_SWING.unpack(payload)))


#swing detector, same as firmware/App/Src/swing_detect.c (test_host.py checks they agree)

@dataclass
class SwingCfg:
    #same settings as SWING_CFG in app.c
    start_dps: float = 300
    end_dps: float = 150
    start_samples: int = 5
    end_samples: int = 50
    min_samples: int = 50
    max_samples: int = 2000
    cooldown_samples: int = 300
    gyro_fs_dps: int = 2000
    skip_axis: int = 1      #y is the handle axis, spinning the racket in hand is left out


def isqrt32(x: int) -> int:
    return math.isqrt(x)


class SwingDetector:
    IDLE, IN_SWING, COOLDOWN = 0, 1, 2

    def __init__(self, cfg: SwingCfg | None = None) -> None:
        self.cfg = cfg or SwingCfg()
        to_raw = lambda dps: int(dps * 32768 // self.cfg.gyro_fs_dps) & 0xFFFF
        self.start2 = to_raw(self.cfg.start_dps) ** 2
        self.end2 = to_raw(self.cfg.end_dps) ** 2
        self.state = self.IDLE
        self.run = 0
        self.len = 0
        self.count = 0
        self.start_seq = self.peak_seq = self.peak_mag2 = self.flags = 0

    def update(self, seq: int, gyr, sample_flags: int = 0):
        c = self.cfg
        m2 = sum(gyr[i] * gyr[i] for i in range(3) if i != c.skip_axis)
        if self.state == self.IDLE:
            self.run = self.run + 1 if m2 > self.start2 else 0
            if self.run >= c.start_samples:
                self.state = self.IN_SWING
                self.start_seq = seq - (c.start_samples - 1)
                self.len = c.start_samples
                self.peak_mag2, self.peak_seq = m2, seq
                self.flags = sample_flags & FLAG_GYRO_CLIPPED
                self.run = 0
            return None
        if self.state == self.IN_SWING:
            self.len += 1
            self.flags |= sample_flags & FLAG_GYRO_CLIPPED
            if m2 > self.peak_mag2:
                self.peak_mag2, self.peak_seq = m2, seq
            self.run = self.run + 1 if m2 < self.end2 else 0
            if self.len > c.max_samples:
                self.state, self.run = self.COOLDOWN, 0
                return None
            if self.run >= c.end_samples:
                active = self.len - self.run
                self.state, self.run = self.COOLDOWN, 0
                if active < c.min_samples:
                    return None
                self.count += 1
                return Swing(self.count, self.start_seq & 0xFFFFFFFF, self.peak_seq,
                             (seq - c.end_samples) & 0xFFFFFFFF,
                             min(isqrt32(self.peak_mag2), 0xFFFF), self.flags)
            return None
        self.run += 1
        if self.run >= c.cooldown_samples:
            self.state, self.run = self.IDLE, 0
        return None


#simulated data for testing without a board

def simulate_gyro(n_samples: int, seed: int = 1, swing_every: int = 2500):
    #fake gyro data (raw counts) with fake swings
    rng = random.Random(seed)
    scale = 32768 / 2000
    next_swing = 800
    pulse = None
    for seq in range(n_samples):
        if seq == next_swing:
            pulse = (seq, rng.uniform(80, 200), rng.uniform(400, 2300))  #start, width, peak deg/s
            next_swing += swing_every + rng.randint(-500, 500)
        gx, gy, gz = (rng.gauss(0, 3) for _ in range(3))
        if pulse:
            t0, w, pk = pulse
            x = (seq - t0 - 2 * w) / (w / 2)
            amp = pk * math.exp(-x * x)
            gz += amp
            gx += 0.3 * amp
            if seq - t0 > 4 * w:
                pulse = None
        clip = lambda v: max(-32768, min(32767, int(round(v * scale))))
        yield seq, (clip(gx), clip(gy), clip(gz))
