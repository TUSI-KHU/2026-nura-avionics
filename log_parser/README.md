# NURA Log Parser

`parse_nlg.py` converts NURA binary `.NLG` flight logs into readable CSV and
JSON files.

## Usage

```bash
python3 log_parser/parse_nlg.py analysis/program_flash_NURA_LOG_FL008_NLG_20260530_145006.NLG
```

By default it creates `<input_stem>_parsed/` beside the input file. Use
`--out-dir` to choose the destination:

```bash
python3 log_parser/parse_nlg.py input.NLG --out-dir analysis/parsed_log
```

## Outputs

- `frames.csv`: every frame in sequence order, with decoded fields and units.
- `frames.json`: the same decoded frames as JSON.
- `fast_sample.csv`: high-rate IMU/barometer/state samples.
- `slow_sample.csv`: GPS/magnetometer/barometer fault counters.
- `event.csv`: boot, state transition, ground stop, and storage fault events.
- `decision.csv`: state-machine decision traces when present.
- `summary.json`: frame count, sequence range, timestamp range, and output paths.

The parser verifies frame magic, version, payload length, maximum frame size, and
CRC16-CCITT before writing decoded output.
