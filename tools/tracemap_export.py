#!/usr/bin/env python3
"""Convert NURA TraceMap CSV into timelines and execution/transition maps."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
from collections import Counter, defaultdict
from pathlib import Path


STATE_NAMES = {
    0: "INIT",
    1: "SAFE",
    2: "ARMED",
    3: "LAUNCH",
    4: "COAST",
    5: "APOGEE",
    6: "DROGUE",
    7: "DEPLOY",
    8: "GROUND",
    9: "FAULT",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace_csv", type=Path)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--include-bus", action="store_true")
    return parser.parse_args()


def q(value: str) -> str:
    return '"' + value.replace('"', '\\"') + '"'


def render_dot(dot_path: Path) -> None:
    dot = shutil.which("dot")
    if dot:
        subprocess.run(
            [dot, "-Tsvg", str(dot_path), "-o", str(dot_path.with_suffix(".svg"))],
            check=True,
        )


def main() -> int:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    event_counts: Counter[str] = Counter()
    app_runs: Counter[str] = Counter()
    app_max_duration: defaultdict[str, int] = defaultdict(int)
    transitions: Counter[tuple[int, int]] = Counter()
    communication: Counter[tuple[str, str, int]] = Counter()
    chrome_events: list[dict[str, object]] = []
    first_sequence: int | None = None
    last_sequence = 0
    sequence_gaps = 0

    with args.trace_csv.open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            sequence = int(row["sequence"])
            timestamp_us = int(row["timestamp_us"])
            duration_us = int(row["duration_us"])
            event = row["event"]
            app = row["app"]
            peer = row["peer"]
            topic = int(row["topic"])
            state = int(row["state"])
            detail = int(row["detail"])
            result = int(row["result"])

            if first_sequence is None:
                first_sequence = sequence
            if last_sequence and sequence != last_sequence + 1:
                sequence_gaps += sequence - last_sequence - 1
            last_sequence = sequence
            event_counts[event] += 1

            if event in {"task_end", "state_app_end"}:
                app_runs[app] += 1
                app_max_duration[app] = max(app_max_duration[app], duration_us)
                chrome_events.append(
                    {
                        "name": app,
                        "cat": "task" if event == "task_end" else "state_app",
                        "ph": "X",
                        "ts": max(0, timestamp_us - duration_us),
                        "dur": duration_us,
                        "pid": 1,
                        "tid": app,
                        "args": {"state": STATE_NAMES.get(state, str(state)), "result": result},
                    }
                )
            elif event == "transition_request":
                chrome_events.append(
                    {
                        "name": f"{STATE_NAMES.get(state, state)}->{STATE_NAMES.get(detail, detail)}",
                        "cat": "fsm_request",
                        "ph": "i",
                        "s": "g",
                        "ts": timestamp_us,
                        "pid": 1,
                        "tid": "flight_coordinator",
                        "args": {"requested_by": app},
                    }
                )
            elif event == "transition_commit":
                transitions[(state, detail)] += 1
                chrome_events.append(
                    {
                        "name": f"{STATE_NAMES.get(state, state)}->{STATE_NAMES.get(detail, detail)}",
                        "cat": "fsm_commit",
                        "ph": "i",
                        "s": "g",
                        "ts": timestamp_us,
                        "pid": 1,
                        "tid": "flight_coordinator",
                        "args": {"committed_by": app},
                    }
                )
            elif event in {"bus_publish", "bus_consume", "bus_drop"}:
                communication[(app, peer, topic)] += 1
                if args.include_bus or event == "bus_drop":
                    chrome_events.append(
                        {
                            "name": event,
                            "cat": "software_bus",
                            "ph": "i",
                            "s": "t",
                            "ts": timestamp_us,
                            "pid": 1,
                            "tid": app,
                            "args": {"peer": peer, "topic": topic, "result": result},
                        }
                    )
            elif event in {"deadline_miss", "actuation_intent", "actuation_result"}:
                chrome_events.append(
                    {
                        "name": event,
                        "cat": "safety",
                        "ph": "i",
                        "s": "g",
                        "ts": timestamp_us,
                        "pid": 1,
                        "tid": app,
                        "args": {"peer": peer, "state": STATE_NAMES.get(state, state), "result": result, "detail": detail},
                    }
                )

    timeline_path = args.out_dir / "timeline.json"
    timeline_path.write_text(json.dumps({"traceEvents": chrome_events}), encoding="utf-8")

    fsm_dot = ["digraph fsm {", "  rankdir=LR;", "  node [shape=box];"]
    for (previous, current), count in sorted(transitions.items()):
        fsm_dot.append(
            f"  {q(STATE_NAMES.get(previous, str(previous)))} -> "
            f"{q(STATE_NAMES.get(current, str(current)))} [label={q(str(count))}];"
        )
    fsm_dot.append("}")
    fsm_path = args.out_dir / "fsm.dot"
    fsm_path.write_text("\n".join(fsm_dot) + "\n", encoding="utf-8")

    runtime_dot = ["digraph runtime {", "  rankdir=LR;", "  node [shape=box];"]
    for (app, peer, topic), count in communication.most_common():
        if app == "unknown" or peer == "unknown":
            continue
        runtime_dot.append(
            f"  {q(app)} -> {q(peer)} [label={q(f'topic {topic}: {count}')}];"
        )
    runtime_dot.append("}")
    runtime_path = args.out_dir / "runtime.dot"
    runtime_path.write_text("\n".join(runtime_dot) + "\n", encoding="utf-8")

    summary = [
        "# TraceMap Summary",
        "",
        f"- Source: `{args.trace_csv}`",
        f"- Sequence: `{first_sequence or 0}` to `{last_sequence}`",
        f"- Sequence gaps: `{sequence_gaps}`",
        f"- Deadline misses: `{event_counts['deadline_miss']}`",
        f"- Bus drops: `{event_counts['bus_drop']}`",
        f"- FSM transitions: `{sum(transitions.values())}`",
        "",
        "## Task WCET Observations",
        "",
        "| App | Runs | Max duration (us) |",
        "|---|---:|---:|",
    ]
    for app in sorted(app_runs):
        summary.append(f"| {app} | {app_runs[app]} | {app_max_duration[app]} |")
    (args.out_dir / "summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")

    render_dot(fsm_path)
    render_dot(runtime_path)
    print(f"TraceMap artifacts: {args.out_dir}")
    print(f"sequence_gaps={sequence_gaps} transitions={sum(transitions.values())}")
    return 0 if sequence_gaps == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
