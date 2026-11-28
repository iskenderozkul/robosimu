# v8 scenario: camera frame rate

Question: if the camera frame rate rises, does the reaching quality of the Perceptual 1/e Governor rise too?

Six-axis arm, the 12 test targets of `run_v5.py`, 2 noise seeds (n = 24 per cell), 5 s window, success = settled within 5 mm.
Frame rate = control rate = 15, 30, 60, 100, 200 fps (frame period T = 1/fps). Visual delay 60 and 150 ms.
The body schema (image Jacobian and servo lag L) is re-identified by babbling at every frame rate. True servo lag is about 50 ms.

Two averaging rules: **n8** keeps 8 frames at every rate (the paper's setting; its window shrinks as fps rises);
**w80** keeps an 80 ms window (n = round(0.08 fps)), so only sampling changes.

| rule | delay | fps | n | identified L (ms) | k (1/s) | δ_eff (ms) | success | max OS | mean OS | median MT (s) |
|---|---|---|---|---|---|---|---|---|---|---|
| n8 | 60 ms | 15 | 8 | 0 | 1.13 | 327 | 79 % | 31.9 % | 5.51 % | 2.73 |
| n8 | 60 ms | 30 | 8 | 10 | 1.81 | 203 | 100 % | 6.6 % | 2.48 % | 1.00 |
| n8 | 60 ms | 60 | 8 | 25 | 2.43 | 152 | 100 % | 7.0 % | 3.20 % | 0.88 |
| n8 | 60 ms | 100 | 8 | 50 | 2.45 | 150 | 100 % | 2.6 % | 0.40 % | 1.38 |
| n8 | 60 ms | 200 | 8 | 50 | 2.83 | 130 | 100 % | 3.6 % | 1.71 % | 0.72 |
| n8 | 150 ms | 15 | 8 | 0 | 0.88 | 417 | 50 % | 34.0 % | 5.68 % | 2.13 |
| n8 | 150 ms | 30 | 8 | 10 | 1.25 | 293 | 100 % | 6.1 % | 1.66 % | 1.32 |
| n8 | 150 ms | 60 | 8 | 25 | 1.52 | 242 | 100 % | 6.6 % | 2.19 % | 0.97 |
| n8 | 150 ms | 100 | 8 | 50 | 1.53 | 240 | 96 % | 0.8 % | 0.09 % | 2.18 |
| n8 | 150 ms | 200 | 8 | 50 | 1.67 | 220 | 100 % | 2.6 % | 0.87 % | 1.04 |
| w80 | 60 ms | 15 | 1 | 0 | 3.94 | 93 | 100 % | 18.4 % | 5.82 % | 3.73 |
| w80 | 60 ms | 30 | 2 | 10 | 3.56 | 103 | 96 % | 15.5 % | 6.04 % | 3.33 |
| w80 | 60 ms | 60 | 5 | 25 | 2.90 | 127 | 100 % | 8.4 % | 4.19 % | 1.39 |
| w80 | 60 ms | 100 | 8 | 50 | 2.45 | 150 | 100 % | 2.6 % | 0.40 % | 1.38 |
| w80 | 60 ms | 200 | 16 | 50 | 2.45 | 150 | 100 % | 3.1 % | 0.82 % | 0.81 |
| w80 | 150 ms | 15 | 1 | 0 | 2.01 | 183 | 92 % | 32.1 % | 5.69 % | 2.23 |
| w80 | 150 ms | 30 | 2 | 10 | 1.90 | 193 | 100 % | 9.8 % | 3.51 % | 1.35 |
| w80 | 150 ms | 60 | 5 | 25 | 1.70 | 217 | 100 % | 7.3 % | 2.82 % | 1.07 |
| w80 | 150 ms | 100 | 8 | 50 | 1.53 | 240 | 96 % | 0.8 % | 0.09 % | 2.18 |
| w80 | 150 ms | 200 | 16 | 50 | 1.53 | 240 | 100 % | 1.5 % | 0.28 % | 1.13 |

## Findings

1. **15 → 30 fps is the step that decides safety.** At 15 fps one frame lasts 67 ms, longer than the 50 ms servo lag, so the babbling cannot resolve the lag (identified L = 0). δ_eff is then underestimated, the gain is too high, and with n = 8 the maximum overshoot is 32–34 % and success 50–79 %. From 30 fps on, every cell succeeds on 96–100 % of reaches.
2. **The lowest overshoot needs a frame rate at which the lag is identified correctly.** L was identified as 10 ms at 30 fps, 25 ms at 60 fps and 50 ms (the true value) at 100 and 200 fps. With the lag under-identified the gain is slightly too high and the maximum overshoot is 6–7 % with n = 8 (up to 15 % when the 80 ms window holds only 2 frames); at 100–200 fps it is 0.8–3.6 %.
3. **Speed does not follow the frame rate closely.** The gain k = (1/e)/δ_eff rises with fps only through the averaging and sampling terms, because the visual delay δ_obs and the servo lag L do not shrink with the frame rate; at 100–200 fps they make up 85–95 % of δ_eff. Median movement time at ≥ 30 fps was 0.72–1.38 s (60 ms) and 0.97–2.18 s (150 ms) without a monotone trend; it is set mainly by the depth channel (Section 8.10 of the paper), not by k.
4. **Doubling 100 → 200 fps changes little.** δ_eff falls by 20 ms with n = 8 and not at all with an 80 ms window; success and overshoot stay in the same range.
5. **Practical rule.** Use at least 2/L frames per second so that the lag can be identified (≥ 40 fps for L = 50 ms; 100 fps is comfortable). Beyond that, a shorter image-transfer and processing delay helps more than a higher frame rate.

Implementation note: the efference-copy history kept 300–400 samples, i.e. 1.5–2 s at 200 fps, too short for the 2.6 s babbling; it now keeps 3–4 s at any frame rate (`perceptual_governor.py`). The 100 fps results of the paper are unchanged by this fix.

Reproduce: `python scripts/run_v8_framerate.py` (writes `reports/v8_framerate.jsonl` and this file).
Simulation only; motor parameters are representative, not measured.
