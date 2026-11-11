# Results

Every number in the main README comes from a file in this folder. All recordings were made on 2026-11-11 with the board described in the main README. The `.txt` files are the output of `python/analyze_log.py` for each recording.

| File | What it shows |
|---|---|
| `2026-11-11_scope_freq_1kHz.png` | Oscilloscope on the D7 debug pin: 1.0000 kHz, one pulse per sample. Pulse width (92.7 µs) is the time for one sensor read. |
| `2026-11-11_scope_jitter_23ns.png`, `2026-11-11_scope_jitter_44ns.png` | Infinite persistence on the edge 1 ms after the trigger, two runs of about 2 minutes. Cursors span the earliest and latest edge. |
| `2026-11-11_jitter_5min.txt` | 5 minute recording with the board still: sampling rate, jitter and dropped samples. The raw log (7.8 MB) is not included. |
| `2026-11-11_drop_test_115200.bin` / `.txt` | UART deliberately slowed to 115200 baud to check that dropped samples are detected. 55.8% were lost and all were counted. |
| `2026-11-11_rotation_720.bin` / `.txt` | Gyro scale check: 5 trials of exactly two turns (720°) on a spinning tray, lined up against a fixed mark. |
| `2026-11-11_swings_A_tuning.txt` | Tuning recording: 15 practice swings and non-swing movement, recorded with the earlier firmware. Spinning the racket in hand was counted as swings, which led to ignoring rotation about the handle. The raw log is not included. |
| `2026-11-11_swings_B_test.bin` / `.txt` | Test recording with the final firmware, separate from tuning: 20 practice swings, then spinning the racket in hand, bouncing a ball and flipping it. |

To re-run an analysis:

```
python python/analyze_log.py results/2026-11-11_swings_B_test.bin
python python/analyze_log.py results/2026-11-11_rotation_720.bin --rotations --target-deg 720
```
