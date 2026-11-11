#swing analyzer - live view of swing speed and swing count
#  python swing_dashboard.py --port COM10 --save run.bin   (port from: python -m serial.tools.list_ports)
#  python swing_dashboard.py --fake                        simulated data, no board needed
#--save keeps every raw byte so analyze_log.py can work out the results later
from __future__ import annotations

import argparse
import collections
import threading
import time

import swingproto as sp

WINDOW_S = 3.0


class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.trace = collections.deque(maxlen=int(WINDOW_S * 1000))   #(seq, |gyro| deg/s)
        self.swings: list[sp.Swing] = []
        self.status: sp.Status | None = None
        self.samples = 0
        self.gaps = 0          #samples missing from the sequence counter
        self.last_seq = None
        self.resets = 0        #board resets seen (sequence counter went back to 0)
        self.parser = sp.Parser()
        self.error = None


def handle(state: State, pkts, fs_dps: int) -> None:
    with state.lock:
        for p in pkts:
            if isinstance(p, sp.Sample):
                if state.last_seq is not None:
                    gap = sp.seq_gap(state.last_seq, p.seq)
                    if gap is None:
                        state.resets += 1
                        state.trace.clear()
                    else:
                        state.gaps += gap
                state.last_seq = p.seq
                state.samples += 1
                g = p.gyr
                mag = sum(g[i] ** 2 for i in range(3) if i != sp.SwingCfg.skip_axis) ** 0.5 * fs_dps / 32768
                state.trace.append((p.seq, mag))
            elif isinstance(p, sp.Swing):
                state.swings.append(p)
            elif isinstance(p, sp.Status):
                state.status = p


def serial_reader(state: State, port: str, baud: int, save) -> None:
    import serial  #pyserial
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as e:
        state.error = f"Could not open {port}: {e}"
        return
    while True:
        try:
            data = ser.read(4096)
        except serial.SerialException as e:
            state.error = f"Serial port error (board unplugged?): {e}"
            return
        if not data:
            continue
        if save:
            save.write(data)
        st = state.status
        handle(state, state.parser.feed(data), st.gyro_fs_dps if st else 2000)


def fake_reader(state: State, save) -> None:
    #simulated packets at about 1 khz in real time
    det = sp.SwingDetector()
    t0 = time.perf_counter()
    for seq, g in sp.simulate_gyro(10 ** 9, seed=int(time.time())):
        buf = sp.encode_sample(sp.Sample(seq, seq * 180_000, (0, 0, 2048), g, 0))
        ev = det.update(seq, g)
        if ev:
            buf += sp.encode_swing(ev)
        if seq % 1000 == 0:
            buf += sp.encode_status(sp.Status(180_000_000, seq, seq, 0, 0, 0, 0, det.count,
                                              1_000_000, 2000, 16, 0, 0x24))
        if save:
            save.write(buf)
        handle(state, state.parser.feed(buf), 2000)
        if seq % 20 == 0:
            delay = t0 + seq / 1000 - time.perf_counter()
            if delay > 0:
                time.sleep(delay)


def run_gui(state: State, fake: bool) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, (ax, ax_txt) = plt.subplots(2, 1, figsize=(9, 6), height_ratios=[3, 1])
    fig.canvas.manager.set_window_title("Swing Analyzer" + (" - SIMULATED DATA" if fake else ""))
    (line,) = ax.plot([], [], lw=1)
    peaks = ax.scatter([], [], color="red", zorder=3, label="detected swing peak")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("|angular velocity| (deg/s)")
    ax.set_ylim(0, 2300)
    ax.axhline(2000, ls="--", lw=0.8, color="gray")
    ax.text(0.01, 0.93, "sensor limit 2000 deg/s", transform=ax.transAxes, fontsize=8, color="gray")
    ax.legend(loc="upper right")
    if fake:
        ax.set_title("SIMULATED DATA", color="red")
    ax_txt.axis("off")
    txt = ax_txt.text(0, 1, "", va="top", family="monospace", fontsize=10)

    def update(_):
        with state.lock:
            tr = list(state.trace)
            swings = list(state.swings)
            st, n, gaps, err = state.status, state.samples, state.gaps, state.error
            resets = state.resets
            bad = state.parser.bad_crc
        fs = st.gyro_fs_dps if st else 2000
        if tr:
            s_end = tr[-1][0]
            line.set_data([(s - s_end) / 1000 for s, _ in tr], [m for _, m in tr])
            ax.set_xlim(-WINDOW_S, 0)
            vis = [w for w in swings if s_end - w.peak_seq < WINDOW_S * 1000]
            peaks.set_offsets([[(w.peak_seq - s_end) / 1000, w.peak_dps(fs)] for w in vis]
                              or [[float("nan"), float("nan")]])
        lines = []
        if err:
            lines.append("ERROR: " + err)
        if st and st.bmi_status != 0:
            lines.append(f"IMU ERROR {st.bmi_status}: {sp.BMI_STATUS_TEXT.get(st.bmi_status, '?')}"
                         f"  (chip id read 0x{st.chip_id:02X}, expected 0x24; diag 0x{st.bmi_diag:02X})")
        if n == 0 and not err:
            lines.append("Waiting for data... (right COM port? baud 460800? press the black RESET button)")
        last = swings[-1] if swings else None
        lines.append(f"Swings: {len(swings):4d}   "
                     + (f"last peak: {last.peak_dps(fs):7.0f} deg/s"
                        + (" (CLIPPED - real value higher)" if last.flags & 1 else "")
                        if last else "last peak: -"))
        if swings:
            lines.append(f"Fastest this session: {max(w.peak_dps(fs) for w in swings):7.0f} deg/s")
        lines.append(f"Samples received: {n}   missing (seq gaps): {gaps}   bad CRC: {bad}"
                     + (f"   board resets: {resets}" if resets else ""))
        if st:
            lines.append(f"Board counters: missed reads {st.missed_reads}, ring overflows "
                         f"{st.ring_overflows}, UART drops {st.tx_dropped}, SPI errors {st.spi_errors}")
        txt.set_text("\n".join(lines))
        return line, peaks, txt

    _anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(description="live view of swing speed and swing count")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=460800, help="must match CubeMX USART2 setting")
    ap.add_argument("--save", help="write raw bytes to this .bin file for analyze_log.py")
    ap.add_argument("--fake", action="store_true", help="use SIMULATED data (no board needed)")
    args = ap.parse_args()
    if not args.fake and not args.port:
        ap.error("give --port, or --fake to try it without hardware")

    state = State()
    save = open(args.save, "wb") if args.save else None
    target = (lambda: fake_reader(state, save)) if args.fake else \
             (lambda: serial_reader(state, args.port, args.baud, save))
    threading.Thread(target=target, daemon=True).start()
    try:
        run_gui(state, args.fake)
    finally:
        if save:
            save.close()


if __name__ == "__main__":
    main()
