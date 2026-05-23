# Apogee Predictor Evaluation

Input: OpenRocket flightdata branches from `rocket.ork`, resampled to 50 ms barometer samples.

Expanded simulation matrix:

- OpenRocket source branches: 3
- Trajectory variants per branch: 7
- Barometer/environment sensor scenarios: 12
- Total cases per aggregation candidate: 252

Trajectory variants include nominal flight, low impulse / high drag, strong headwind drag, heavy slow coast,
light fast coast, tailwind low drag, and high impulse / low drag transforms.

Sensor/environment scenarios include clean data, nominal noise, humid/hot scale error, cold/dry scale error,
pressure drift, port gust pulses, positive/negative port pressure pulses, sample dropouts, quantization,
and a worst-case noise/spike/dropout combination.

Firmware-equivalent gates:

- 9-sample quadratic fit.
- `a < -0.05` curvature gate.
- fit RMSE <= 2.5 m.
- raw prediction jump <= 15 m.
- `plus2sigma5` prediction history sigma <= 8 m.
- predicted apogee no more than 1.0 s ahead.
- predicted apogee no more than 20 m above current altitude.
- trigger margin 0..3 m for 3 consecutive samples.
- no apogee prediction before launch time 8.0 s.

Safety ranking policy:

- Dangerous early trigger: trigger more than 10 m below apogee or more than 1.0 s early.
- Gross early trigger: trigger more than 5 m below apogee.
- Misses are acceptable up to 80%, because descent and timer backup can catch missed predictor triggers.
- Ranking first minimizes dangerous early triggers, then gross early triggers, then mean early-altitude deficit.

Aggregate ranking, lower is better:

| Aggregator | Misses | Dangerous early | Gross early | Mean early-alt deficit (m) | Mean abs trigger time error (s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| `plus2sigma5` | 102/252 (0.405) | 5 | 29 | 3.764 | 0.468 |
| `plus1p5sigma5` | 80/252 (0.317) | 18 | 42 | 4.808 | 0.602 |
| `max5` | 71/252 (0.282) | 27 | 56 | 5.073 | 0.667 |
| `minus1sigma5` | 140/252 (0.556) | 39 | 51 | 7.683 | 0.841 |
| `plus1sigma5` | 63/252 (0.250) | 39 | 70 | 5.940 | 0.759 |
| `median5` | 70/252 (0.278) | 59 | 83 | 7.859 | 0.899 |
| `raw` | 49/252 (0.194) | 66 | 93 | 8.252 | 0.910 |
| `mean5` | 76/252 (0.302) | 70 | 88 | 8.732 | 0.964 |

Recommendation: use `plus2sigma5` for the deployment-trigger aggregation candidate.

Interpretation:

- The newest raw prediction candidate is still useful as a baseline, but it is not the safest trigger candidate under noisy or gusty barometer data.
- Candidates above the mean prediction delay deployment because they require the rocket to get closer to a higher estimated apogee.
- Very conservative candidates can miss the predictor trigger entirely; that is acceptable only because the FSM already has descent and timer backup paths.

Files:

- `summary.csv`: case-by-case metric table.
- `predictions.csv`: every valid prediction and trigger marker.
- `overview.png`: visual check for representative nominal and disturbed cases.
