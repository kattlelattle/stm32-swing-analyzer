#pc tests: crc, packet round trip, and the c and python swing detectors agreeing
#run: make -C firmware/host_test, then python python/test_host.py
#tests logic only, not spi, dma, timing or wiring
import os
import random
import subprocess
import sys

import swingproto as sp

HERE = os.path.dirname(os.path.abspath(__file__))
HOST_SIM = os.path.join(HERE, "..", "firmware", "host_test", "host_sim")
if os.name == "nt":
    HOST_SIM += ".exe"


def test_crc_known_value():
    #standard crc-8 check value: "123456789" gives 0xF4
    assert sp.crc8(b"123456789") == 0xF4


def test_roundtrip_with_garbage():
    rng = random.Random(0)
    frames, expect = b"", []
    for i in range(500):
        s = sp.Sample(i, i * 180000, (1, -2, 3), (rng.randint(-32768, 32767), 0, -5), 0)
        expect.append(s)
        frames += sp.encode_sample(s)
        if i % 7 == 0:
            frames += bytes(rng.randrange(256) for _ in range(rng.randint(1, 30)))
    p = sp.Parser()
    got = []
    for k in range(0, len(frames), 13):            #feed it in awkward chunk sizes
        got += [x for x in p.feed(frames[k:k + 13]) if isinstance(x, sp.Sample)]
    assert got == expect, "garbage between frames broke decoding"


def test_corrupted_frame_is_rejected():
    good = sp.encode_sample(sp.Sample(1, 2, (0, 0, 0), (0, 0, 0), 0))
    bad = bytearray(good)
    bad[10] ^= 0xFF
    p = sp.Parser()
    out = p.feed(bytes(bad) + good)
    assert len(out) == 1 and out[0].seq == 1 and p.bad_crc >= 1


def test_status_roundtrip():
    st = sp.Status(180_000_000, 1, 2, 3, 4, 5, 6, 7, 1_000_000, 2000, 16, 3, 0x24, 0x02)
    (got,) = sp.Parser().feed(sp.encode_status(st))
    assert got == st


def test_board_reset_is_not_counted_as_drops():
    assert sp.seq_gap(41, 42) == 0
    assert sp.seq_gap(41, 44) == 2
    assert sp.seq_gap(5000, 0) is None      #board reset, not 4 billion drops


def test_c_and_python_agree():
    if not os.path.exists(HOST_SIM):
        print("SKIP: build firmware/host_test first")
        return
    data = list(sp.simulate_gyro(60000, seed=7))
    stdin = "".join(f"{seq} {g[0]} {g[1]} {g[2]}\n" for seq, g in data).encode()
    raw = subprocess.run([HOST_SIM], input=stdin, capture_output=True, check=True).stdout

    pkts = sp.Parser().feed(raw)
    samples = [x for x in pkts if isinstance(x, sp.Sample)]
    c_swings = [x for x in pkts if isinstance(x, sp.Swing)]
    status = [x for x in pkts if isinstance(x, sp.Status)]
    assert [s.seq for s in samples] == [d[0] for d in data]
    assert all(s.gyr == d[1] for s, d in zip(samples, data))

    det = sp.SwingDetector()
    clipped = lambda g: sp.FLAG_GYRO_CLIPPED if any(v in (32767, -32768) for v in g) else 0
    py_swings = [e for seq, g in data if (e := det.update(seq, g, clipped(g))) is not None]
    assert any(s.flags & sp.FLAG_GYRO_CLIPPED for s in c_swings), "expected some clipped swings"
    assert py_swings == c_swings, "C and Python detectors disagree"
    assert status and status[-1].swing_count == len(c_swings)
    assert len(c_swings) > 5, "simulator should produce several swings"
    print(f"  C and Python both found {len(c_swings)} swings in 60 s of simulated data")


if __name__ == "__main__":
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as e:
                fails += 1
                print(f"FAIL {name}: {e}")
    sys.exit(1 if fails else 0)
