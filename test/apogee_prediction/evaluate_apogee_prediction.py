#!/usr/bin/env python3
"""Evaluate the NURA apogee predictor against OpenRocket altitude data.

The script mirrors the firmware-side first implementation closely enough for
algorithm selection:

- 50 ms barometer sampling
- 3-sample median + EWMA altitude filter
- 9-sample quadratic least-squares apogee fit
- 8 s launch-time inhibit
- 3 consecutive samples before an apogee trigger

The input trajectory comes from OpenRocket flightdata branches. Additional
trajectory and barometer/environment variants are deterministic perturbations of
that OpenRocket data, not full OpenRocket reruns.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import statistics
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


DEFAULT_ORK = Path("/home/sb/Downloads/02_공력팀_26NURA_rocket_0516/rocket.ork")
DEFAULT_RESULTS_DIR = Path("test/apogee_prediction/results")

SAMPLE_PERIOD_S = 0.05
MEDIAN_WINDOW = 3
LPF_ALPHA = 0.35

FIT_WINDOW = 9
CONFIRM_SAMPLES = 3
MIN_FLIGHT_TIME_S = 8.0
MAX_PREDICT_AHEAD_S = 1.0
DEPLOY_ALT_MARGIN_M = 3.0
MAX_ALT_MARGIN_M = 20.0
MIN_CURVATURE = 0.05
MAX_CURVATURE = 120.0
MAX_FIT_RMSE_M = 2.5
MAX_PREDICTION_JUMP_M = 15.0
MAX_PREDICTION_SIGMA_M = 8.0
MIN_DETECT_ALT_M = 30.0

AGGREGATION_WINDOW = 5
GROSS_EARLY_ALTITUDE_M = 5.0
DANGEROUS_EARLY_ALTITUDE_M = 10.0
DANGEROUS_EARLY_TIME_S = 1.0
MAX_ACCEPTABLE_MISS_RATIO = 0.80


@dataclass(frozen=True)
class FlightPoint:
    time_s: float
    altitude_m: float


@dataclass(frozen=True)
class SimulationBranch:
    name: str
    status: str
    max_altitude_m: float
    time_to_apogee_s: float
    points: list[FlightPoint]


@dataclass(frozen=True)
class TrajectoryVariant:
    name: str
    altitude_scale: float = 1.0
    time_scale: float = 1.0
    shape_power: float = 1.0


@dataclass(frozen=True)
class SensorScenario:
    name: str
    noise_sigma_m: float = 0.0
    scale: float = 1.0
    drift_at_apogee_m: float = 0.0
    spike_probability: float = 0.0
    spike_min_m: float = 0.0
    spike_max_m: float = 0.0
    pulse_count: int = 0
    pulse_amp_min_m: float = 0.0
    pulse_amp_max_m: float = 0.0
    pulse_width_s: float = 0.18
    dropout_probability: float = 0.0
    quantization_m: float = 0.0


@dataclass(frozen=True)
class PredictionRow:
    sim_name: str
    status: str
    trajectory: str
    sensor: str
    aggregator: str
    time_s: float
    altitude_m: float
    predicted_apogee_m: float
    true_apogee_m: float
    true_apogee_time_s: float
    error_m: float
    margin_m: float
    trigger: bool


@dataclass(frozen=True)
class RawPrediction:
    apogee_m: float
    fit_rmse_m: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ork", type=Path, default=DEFAULT_ORK)
    parser.add_argument("--out", type=Path, default=DEFAULT_RESULTS_DIR)
    return parser.parse_args()


def trajectory_variants() -> list[TrajectoryVariant]:
    return [
        TrajectoryVariant("nominal"),
        TrajectoryVariant("low_impulse_high_drag", altitude_scale=0.72, time_scale=0.88, shape_power=1.04),
        TrajectoryVariant("strong_headwind_drag", altitude_scale=0.82, time_scale=0.94, shape_power=1.03),
        TrajectoryVariant("heavy_slow_coast", altitude_scale=0.90, time_scale=1.08, shape_power=0.98),
        TrajectoryVariant("light_fast_coast", altitude_scale=1.05, time_scale=0.90, shape_power=1.02),
        TrajectoryVariant("tailwind_low_drag", altitude_scale=1.08, time_scale=1.04, shape_power=0.98),
        TrajectoryVariant("high_impulse_low_drag", altitude_scale=1.18, time_scale=1.08, shape_power=0.96),
    ]


def sensor_scenarios() -> list[SensorScenario]:
    return [
        SensorScenario("clean"),
        SensorScenario("nominal_noise_0p5m", noise_sigma_m=0.5),
        SensorScenario("humid_hot_scale_high", noise_sigma_m=0.3, scale=1.015, drift_at_apogee_m=0.5),
        SensorScenario("cold_dry_scale_low", noise_sigma_m=0.3, scale=0.985, drift_at_apogee_m=-0.5),
        SensorScenario("pressure_drift_up_4m", noise_sigma_m=0.5, drift_at_apogee_m=4.0),
        SensorScenario("pressure_drift_down_4m", noise_sigma_m=0.5, drift_at_apogee_m=-4.0),
        SensorScenario(
            "wind_port_gust_pulses",
            noise_sigma_m=0.8,
            spike_probability=0.01,
            spike_min_m=3.0,
            spike_max_m=7.0,
            pulse_count=3,
            pulse_amp_min_m=-5.0,
            pulse_amp_max_m=5.0,
            pulse_width_s=0.20,
        ),
        SensorScenario(
            "pressure_port_negative_pulse",
            noise_sigma_m=0.5,
            pulse_count=2,
            pulse_amp_min_m=-8.0,
            pulse_amp_max_m=-4.0,
            pulse_width_s=0.16,
        ),
        SensorScenario(
            "pressure_port_positive_pulse",
            noise_sigma_m=0.5,
            pulse_count=2,
            pulse_amp_min_m=4.0,
            pulse_amp_max_m=8.0,
            pulse_width_s=0.16,
        ),
        SensorScenario("baro_sample_dropouts", noise_sigma_m=0.5, dropout_probability=0.08),
        SensorScenario("quantized_0p5m", noise_sigma_m=0.35, quantization_m=0.5),
        SensorScenario(
            "worst_case_noise_spikes",
            noise_sigma_m=1.5,
            spike_probability=0.03,
            spike_min_m=5.0,
            spike_max_m=12.0,
            pulse_count=3,
            pulse_amp_min_m=-7.0,
            pulse_amp_max_m=7.0,
            pulse_width_s=0.18,
            dropout_probability=0.04,
        ),
    ]


def parse_openrocket_branches(path: Path) -> list[SimulationBranch]:
    root = ET.parse(path).getroot()
    branches: list[SimulationBranch] = []

    for sim_index, simulation in enumerate(root.findall(".//simulation")):
        status = simulation.attrib.get("status", "unknown")
        flightdata = simulation.find("flightdata")
        if flightdata is None:
            continue

        branch = flightdata.find("databranch")
        if branch is None:
            continue

        types = [item.strip() for item in branch.attrib["types"].split(",")]
        time_index = types.index("Time")
        altitude_index = types.index("Altitude")

        points: list[FlightPoint] = []
        for datapoint in branch.findall("datapoint"):
            fields = [field.strip() for field in (datapoint.text or "").split(",")]
            if len(fields) <= max(time_index, altitude_index):
                continue
            time_s = parse_float(fields[time_index])
            altitude_m = parse_float(fields[altitude_index])
            if math.isfinite(time_s) and math.isfinite(altitude_m):
                points.append(FlightPoint(time_s, altitude_m))

        if not points:
            continue

        name = f"sim{sim_index + 1}_{status}_{branch.attrib.get('name', 'branch')}"
        branches.append(
            SimulationBranch(
                name=name,
                status=status,
                max_altitude_m=float(flightdata.attrib["maxaltitude"]),
                time_to_apogee_s=float(flightdata.attrib["timetoapogee"]),
                points=points,
            )
        )

    return branches


def parse_float(value: str) -> float:
    if value == "NaN":
        return math.nan
    return float(value)


def transform_trajectory(branch: SimulationBranch, variant: TrajectoryVariant) -> SimulationBranch:
    max_source_altitude = max(point.altitude_m for point in branch.points)
    points: list[FlightPoint] = []

    for point in branch.points:
        normalized = max(point.altitude_m, 0.0) / max_source_altitude if max_source_altitude > 0.0 else 0.0
        shaped = (normalized ** variant.shape_power) * max_source_altitude
        points.append(
            FlightPoint(
                time_s=point.time_s * variant.time_scale,
                altitude_m=shaped * variant.altitude_scale,
            )
        )

    max_point = max(points, key=lambda point: point.altitude_m)
    return SimulationBranch(
        name=f"{branch.name}_{variant.name}",
        status=branch.status,
        max_altitude_m=max_point.altitude_m,
        time_to_apogee_s=max_point.time_s,
        points=points,
    )


def resample(points: list[FlightPoint], period_s: float = SAMPLE_PERIOD_S) -> list[FlightPoint]:
    result: list[FlightPoint] = []
    source_index = 0
    end_time_s = points[-1].time_s
    sample_index = 0

    while (sample_index * period_s) <= end_time_s:
        t = sample_index * period_s
        while source_index + 1 < len(points) and points[source_index + 1].time_s < t:
            source_index += 1

        if source_index + 1 >= len(points):
            altitude = points[-1].altitude_m
        else:
            left = points[source_index]
            right = points[source_index + 1]
            span = right.time_s - left.time_s
            ratio = 0.0 if span <= 0.0 else (t - left.time_s) / span
            altitude = left.altitude_m + ratio * (right.altitude_m - left.altitude_m)

        result.append(FlightPoint(t, altitude))
        sample_index += 1

    return result


def measured_altitudes(points: list[FlightPoint], scenario: SensorScenario, apogee_time_s: float, seed: int) -> list[FlightPoint]:
    rng = random.Random(seed)
    pulses: list[tuple[float, float]] = []
    pulse_start_s = max(0.0, MIN_FLIGHT_TIME_S - 0.8)
    pulse_end_s = max(pulse_start_s + 0.1, apogee_time_s + 0.4)

    for _ in range(scenario.pulse_count):
        center_s = rng.uniform(pulse_start_s, pulse_end_s)
        amplitude_m = rng.uniform(scenario.pulse_amp_min_m, scenario.pulse_amp_max_m)
        pulses.append((center_s, amplitude_m))

    previous_altitude: float | None = None
    result: list[FlightPoint] = []

    for point in points:
        altitude = point.altitude_m * scenario.scale
        altitude += scenario.drift_at_apogee_m * min(max(point.time_s / apogee_time_s, 0.0), 1.0)
        altitude += rng.gauss(0.0, scenario.noise_sigma_m)

        if scenario.spike_probability > 0.0 and rng.random() < scenario.spike_probability:
            spike = rng.choice([-1.0, 1.0]) * rng.uniform(scenario.spike_min_m, scenario.spike_max_m)
            altitude += spike

        for center_s, amplitude_m in pulses:
            normalized_dt = (point.time_s - center_s) / scenario.pulse_width_s
            altitude += amplitude_m * math.exp(-0.5 * normalized_dt * normalized_dt)

        if previous_altitude is not None and rng.random() < scenario.dropout_probability:
            altitude = previous_altitude

        if scenario.quantization_m > 0.0:
            altitude = round(altitude / scenario.quantization_m) * scenario.quantization_m

        previous_altitude = altitude
        result.append(FlightPoint(point.time_s, altitude))

    return result


def filter_altitudes(points: list[FlightPoint]) -> list[FlightPoint]:
    window: list[float] = []
    filtered = 0.0
    ready = False
    result: list[FlightPoint] = []

    for point in points:
        window.append(point.altitude_m)
        if len(window) > MEDIAN_WINDOW:
            window.pop(0)

        median_altitude = statistics.median(window)
        if not ready:
            filtered = median_altitude
            ready = True
        else:
            filtered += LPF_ALPHA * (median_altitude - filtered)
        result.append(FlightPoint(point.time_s, filtered))

    return result


def solve_quadratic(window: list[FlightPoint]) -> tuple[float, float, float] | None:
    t0 = window[0].time_s
    s0 = float(len(window))
    s1 = s2 = s3 = s4 = 0.0
    y0 = y1 = y2 = 0.0

    for point in window:
        x = point.time_s - t0
        y = point.altitude_m
        x2 = x * x
        s1 += x
        s2 += x2
        s3 += x2 * x
        s4 += x2 * x2
        y0 += y
        y1 += x * y
        y2 += x2 * y

    matrix = [
        [s4, s3, s2, y2],
        [s3, s2, s1, y1],
        [s2, s1, s0, y0],
    ]
    return solve_3x3(matrix)


def solve_3x3(matrix: list[list[float]]) -> tuple[float, float, float] | None:
    epsilon = 1.0e-6
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(matrix[row][col]))
        if abs(matrix[pivot][col]) < epsilon:
            return None
        if pivot != col:
            matrix[col], matrix[pivot] = matrix[pivot], matrix[col]

        divisor = matrix[col][col]
        for j in range(col, 4):
            matrix[col][j] /= divisor

        for row in range(3):
            if row == col:
                continue
            factor = matrix[row][col]
            for j in range(col, 4):
                matrix[row][j] -= factor * matrix[col][j]

    result = (matrix[0][3], matrix[1][3], matrix[2][3])
    if all(math.isfinite(value) for value in result):
        return result
    return None


def current_predictor(window: list[FlightPoint], current_altitude_m: float) -> RawPrediction | None:
    fit = solve_quadratic(window)
    if fit is None:
        return None

    a, b, c = fit
    if a >= -MIN_CURVATURE or a <= -MAX_CURVATURE:
        return None

    rmse_m = fit_rmse(window, a, b, c)
    if rmse_m > MAX_FIT_RMSE_M:
        return None

    last_t = window[-1].time_s - window[0].time_s
    t_apogee = -b / (2.0 * a)
    if not math.isfinite(t_apogee):
        return None
    if t_apogee <= last_t or (t_apogee - last_t) > MAX_PREDICT_AHEAD_S:
        return None

    h_apogee = c - ((b * b) / (4.0 * a))
    margin = h_apogee - current_altitude_m
    if (
        not math.isfinite(h_apogee)
        or margin < 0.0
        or margin > MAX_ALT_MARGIN_M
        or current_altitude_m < MIN_DETECT_ALT_M
    ):
        return None
    return RawPrediction(h_apogee, rmse_m)


def fit_rmse(window: list[FlightPoint], a: float, b: float, c: float) -> float:
    t0 = window[0].time_s
    squared_error_sum = 0.0
    for point in window:
        x = point.time_s - t0
        predicted = (a * x * x) + (b * x) + c
        error = point.altitude_m - predicted
        squared_error_sum += error * error
    return math.sqrt(squared_error_sum / len(window))


def aggregator_names() -> list[str]:
    return [
        "raw",
        "mean5",
        "median5",
        "plus1sigma5",
        "plus1p5sigma5",
        "plus2sigma5",
        "minus1sigma5",
        "max5",
    ]


def aggregate_predictions(values: list[float], name: str) -> float | None:
    recent = values[-AGGREGATION_WINDOW:]
    if not recent:
        return None
    if name == "raw":
        return recent[-1]
    if name == "mean5":
        return statistics.mean(recent)
    if name == "median5":
        return statistics.median(recent)
    if name == "plus1sigma5":
        return statistics.mean(recent) + population_sigma(recent)
    if name == "plus1p5sigma5":
        return statistics.mean(recent) + (1.5 * population_sigma(recent))
    if name == "plus2sigma5":
        if len(recent) < AGGREGATION_WINDOW:
            return None
        sigma = population_sigma(recent)
        if sigma > MAX_PREDICTION_SIGMA_M:
            return None
        return statistics.mean(recent) + (2.0 * sigma)
    if name == "minus1sigma5":
        return statistics.mean(recent) - population_sigma(recent)
    if name == "max5":
        return max(recent)
    raise ValueError(f"unknown aggregator: {name}")


def population_sigma(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    mean = statistics.mean(values)
    return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))


def evaluate_profile(
    branch: SimulationBranch,
    trajectory_name: str,
    sensor: SensorScenario,
    seed: int,
) -> tuple[list[PredictionRow], list[dict[str, str]]]:
    points = filter_altitudes(measured_altitudes(resample(branch.points), sensor, branch.time_to_apogee_s, seed))
    aggregators = aggregator_names()
    valid_prediction_history: list[float] = []
    confirm_counts = {name: 0 for name in aggregators}
    triggered = {name: False for name in aggregators}
    trigger_rows: dict[str, PredictionRow] = {}
    rows: list[PredictionRow] = []

    for index in range(FIT_WINDOW - 1, len(points)):
        current = points[index]
        if current.time_s < MIN_FLIGHT_TIME_S:
            continue

        window = points[index - FIT_WINDOW + 1 : index + 1]
        raw_prediction = current_predictor(window, current.altitude_m)
        if raw_prediction is None:
            for name in aggregators:
                confirm_counts[name] = 0
            continue

        if valid_prediction_history and abs(raw_prediction.apogee_m - valid_prediction_history[-1]) > MAX_PREDICTION_JUMP_M:
            for name in aggregators:
                confirm_counts[name] = 0
            continue

        valid_prediction_history.append(raw_prediction.apogee_m)

        for name in aggregators:
            prediction = aggregate_predictions(valid_prediction_history, name)
            if prediction is None:
                continue
            margin = prediction - current.altitude_m
            can_trigger = 0.0 <= margin <= DEPLOY_ALT_MARGIN_M
            confirm_counts[name] = confirm_counts[name] + 1 if can_trigger else 0
            trigger = (not triggered[name]) and confirm_counts[name] >= CONFIRM_SAMPLES
            if trigger:
                triggered[name] = True

            row = PredictionRow(
                sim_name=branch.name,
                status=branch.status,
                trajectory=trajectory_name,
                sensor=sensor.name,
                aggregator=name,
                time_s=current.time_s,
                altitude_m=current.altitude_m,
                predicted_apogee_m=prediction,
                true_apogee_m=branch.max_altitude_m,
                true_apogee_time_s=branch.time_to_apogee_s,
                error_m=prediction - branch.max_altitude_m,
                margin_m=margin,
                trigger=trigger,
            )
            rows.append(row)
            if trigger and name not in trigger_rows:
                trigger_rows[name] = row

    summary = summarize_rows(branch, trajectory_name, sensor.name, rows, trigger_rows, aggregators)
    return rows, summary


def summarize_rows(
    branch: SimulationBranch,
    trajectory_name: str,
    sensor_name: str,
    rows: list[PredictionRow],
    trigger_rows: dict[str, PredictionRow],
    aggregators: Iterable[str],
) -> list[dict[str, str]]:
    summary: list[dict[str, str]] = []
    for name in aggregators:
        selected = [row for row in rows if row.aggregator == name and row.time_s <= branch.time_to_apogee_s]
        errors = [row.error_m for row in selected]
        abs_errors = [abs(error) for error in errors]
        trigger = trigger_rows.get(name)
        trigger_time_error = (trigger.time_s - branch.time_to_apogee_s) if trigger else math.nan
        trigger_altitude_error = (trigger.altitude_m - branch.max_altitude_m) if trigger else math.nan

        summary.append(
            {
                "sim_name": branch.name,
                "status": branch.status,
                "trajectory": trajectory_name,
                "sensor": sensor_name,
                "aggregator": name,
                "true_apogee_m": format_float(branch.max_altitude_m),
                "true_apogee_time_s": format_float(branch.time_to_apogee_s),
                "valid_prediction_count": str(len(selected)),
                "mean_error_m": format_float(statistics.mean(errors) if errors else math.nan),
                "mae_m": format_float(statistics.mean(abs_errors) if abs_errors else math.nan),
                "rmse_m": format_float(math.sqrt(statistics.mean([e * e for e in errors])) if errors else math.nan),
                "trigger_time_s": format_float(trigger.time_s if trigger else math.nan),
                "trigger_time_error_s": format_float(trigger_time_error),
                "trigger_altitude_m": format_float(trigger.altitude_m if trigger else math.nan),
                "trigger_altitude_error_m": format_float(trigger_altitude_error),
                "trigger_predicted_apogee_m": format_float(trigger.predicted_apogee_m if trigger else math.nan),
                "trigger_prediction_error_m": format_float(trigger.error_m if trigger else math.nan),
                "dangerous_early": "1" if is_dangerous_early(trigger_time_error, trigger_altitude_error) else "0",
                "gross_early": "1" if is_gross_early(trigger_altitude_error) else "0",
            }
        )
    return summary


def is_gross_early(trigger_altitude_error_m: float) -> bool:
    return math.isfinite(trigger_altitude_error_m) and trigger_altitude_error_m < -GROSS_EARLY_ALTITUDE_M


def is_dangerous_early(trigger_time_error_s: float, trigger_altitude_error_m: float) -> bool:
    if not math.isfinite(trigger_time_error_s) or not math.isfinite(trigger_altitude_error_m):
        return False
    return (
        trigger_altitude_error_m < -DANGEROUS_EARLY_ALTITUDE_M
        or trigger_time_error_s < -DANGEROUS_EARLY_TIME_S
    )


def format_float(value: float) -> str:
    if not math.isfinite(value):
        return ""
    return f"{value:.3f}"


def write_predictions(path: Path, rows: list[PredictionRow]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "sim_name",
                "status",
                "trajectory",
                "sensor",
                "aggregator",
                "time_s",
                "altitude_m",
                "predicted_apogee_m",
                "true_apogee_m",
                "true_apogee_time_s",
                "error_m",
                "margin_m",
                "trigger",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "sim_name": row.sim_name,
                    "status": row.status,
                    "trajectory": row.trajectory,
                    "sensor": row.sensor,
                    "aggregator": row.aggregator,
                    "time_s": format_float(row.time_s),
                    "altitude_m": format_float(row.altitude_m),
                    "predicted_apogee_m": format_float(row.predicted_apogee_m),
                    "true_apogee_m": format_float(row.true_apogee_m),
                    "true_apogee_time_s": format_float(row.true_apogee_time_s),
                    "error_m": format_float(row.error_m),
                    "margin_m": format_float(row.margin_m),
                    "trigger": "1" if row.trigger else "0",
                }
            )


def write_summary(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def numeric(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    return float(value) if value else math.nan


def rank_aggregators(summary_rows: list[dict[str, str]]) -> list[dict[str, str]]:
    by_aggregator: dict[str, list[dict[str, str]]] = {}
    for row in summary_rows:
        by_aggregator.setdefault(row["aggregator"], []).append(row)

    scores: list[dict[str, str]] = []
    for name, rows in by_aggregator.items():
        trigger_rows = [row for row in rows if row["trigger_time_error_s"]]
        miss_count = len(rows) - len(trigger_rows)
        miss_ratio = miss_count / len(rows)
        gross_count = sum(1 for row in trigger_rows if row["gross_early"] == "1")
        dangerous_count = sum(1 for row in trigger_rows if row["dangerous_early"] == "1")
        altitude_errors = [numeric(row, "trigger_altitude_error_m") for row in trigger_rows]
        time_errors = [numeric(row, "trigger_time_error_s") for row in trigger_rows]
        prediction_errors = [numeric(row, "trigger_prediction_error_m") for row in trigger_rows]
        early_deficits = [-error for error in altitude_errors if math.isfinite(error) and error < 0.0]

        scores.append(
            {
                "aggregator": name,
                "cases": str(len(rows)),
                "miss_count": str(miss_count),
                "miss_ratio": format_float(miss_ratio),
                "dangerous_early_count": str(dangerous_count),
                "gross_early_count": str(gross_count),
                "mean_early_altitude_deficit_m": format_float(statistics.mean(early_deficits) if early_deficits else 0.0),
                "mean_abs_trigger_time_error_s": format_float(statistics.mean([abs(v) for v in time_errors]) if time_errors else math.inf),
                "mean_abs_trigger_prediction_error_m": format_float(
                    statistics.mean([abs(v) for v in prediction_errors]) if prediction_errors else math.inf
                ),
            }
        )

    acceptable = [
        score for score in scores
        if float(score["miss_ratio"]) <= MAX_ACCEPTABLE_MISS_RATIO
    ]
    ranking_pool = acceptable if acceptable else scores
    ranking_pool.sort(key=score_key)
    ranked_names = {score["aggregator"]: index for index, score in enumerate(ranking_pool)}
    scores.sort(key=lambda score: ranked_names.get(score["aggregator"], 9999 + score_key(score)[0]))
    return scores


def score_key(score: dict[str, str]) -> tuple[int, int, float, float, float]:
    return (
        int(score["dangerous_early_count"]),
        int(score["gross_early_count"]),
        float(score["mean_early_altitude_deficit_m"]),
        float(score["miss_ratio"]),
        float(score["mean_abs_trigger_time_error_s"]) if score["mean_abs_trigger_time_error_s"] else math.inf,
    )


def write_recommendation(path: Path, summary_rows: list[dict[str, str]]) -> None:
    ranked = rank_aggregators(summary_rows)
    best = ranked[0]
    total_cases = len(summary_rows) // len(aggregator_names())
    trajectory_count = len({row["trajectory"] for row in summary_rows})
    sensor_count = len({row["sensor"] for row in summary_rows})
    source_count = len({source_branch_name(row) for row in summary_rows})

    lines = [
        "# Apogee Predictor Evaluation",
        "",
        "Input: OpenRocket flightdata branches from `rocket.ork`, resampled to 50 ms barometer samples.",
        "",
        "Expanded simulation matrix:",
        "",
        f"- OpenRocket source branches: {source_count}",
        f"- Trajectory variants per branch: {trajectory_count}",
        f"- Barometer/environment sensor scenarios: {sensor_count}",
        f"- Total cases per aggregation candidate: {total_cases}",
        "",
        "Trajectory variants include nominal flight, low impulse / high drag, strong headwind drag, heavy slow coast,",
        "light fast coast, tailwind low drag, and high impulse / low drag transforms.",
        "",
        "Sensor/environment scenarios include clean data, nominal noise, humid/hot scale error, cold/dry scale error,",
        "pressure drift, port gust pulses, positive/negative port pressure pulses, sample dropouts, quantization,",
        "and a worst-case noise/spike/dropout combination.",
        "",
        "Firmware-equivalent gates:",
        "",
        "- 9-sample quadratic fit.",
        "- `a < -0.05` curvature gate.",
        "- fit RMSE <= 2.5 m.",
        "- raw prediction jump <= 15 m.",
        "- `plus2sigma5` prediction history sigma <= 8 m.",
        "- predicted apogee no more than 1.0 s ahead.",
        "- predicted apogee no more than 20 m above current altitude.",
        "- trigger margin 0..3 m for 3 consecutive samples.",
        "- no apogee prediction before launch time 8.0 s.",
        "",
        "Safety ranking policy:",
        "",
        f"- Dangerous early trigger: trigger more than {DANGEROUS_EARLY_ALTITUDE_M:.0f} m below apogee or more than {DANGEROUS_EARLY_TIME_S:.1f} s early.",
        f"- Gross early trigger: trigger more than {GROSS_EARLY_ALTITUDE_M:.0f} m below apogee.",
        f"- Misses are acceptable up to {MAX_ACCEPTABLE_MISS_RATIO:.0%}, because descent and timer backup can catch missed predictor triggers.",
        "- Ranking first minimizes dangerous early triggers, then gross early triggers, then mean early-altitude deficit.",
        "",
        "Aggregate ranking, lower is better:",
        "",
        "| Aggregator | Misses | Dangerous early | Gross early | Mean early-alt deficit (m) | Mean abs trigger time error (s) |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]

    for score in ranked:
        lines.append(
            "| `{aggregator}` | {miss_count}/{cases} ({miss_ratio}) | {dangerous_early_count} | {gross_early_count} | {mean_early_altitude_deficit_m} | {mean_abs_trigger_time_error_s} |".format(
                **score
            )
        )

    lines.extend(
        [
            "",
            f"Recommendation: use `{best['aggregator']}` for the deployment-trigger aggregation candidate.",
            "",
            "Interpretation:",
            "",
            "- The newest raw prediction candidate is still useful as a baseline, but it is not the safest trigger candidate under noisy or gusty barometer data.",
            "- Candidates above the mean prediction delay deployment because they require the rocket to get closer to a higher estimated apogee.",
            "- Very conservative candidates can miss the predictor trigger entirely; that is acceptable only because the FSM already has descent and timer backup paths.",
            "",
            "Files:",
            "",
            "- `summary.csv`: case-by-case metric table.",
            "- `predictions.csv`: every valid prediction and trigger marker.",
            "- `overview.png`: visual check for representative nominal and disturbed cases.",
        ]
    )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def source_branch_name(row: dict[str, str]) -> str:
    suffix = f"_{row['trajectory']}"
    if row["sim_name"].endswith(suffix):
        return row["sim_name"][: -len(suffix)]
    return row["sim_name"]


def write_overview_plot(path: Path, rows: list[PredictionRow], summary_rows: list[dict[str, str]]) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return

    nominal_sim = next(
        (row.sim_name for row in rows if "uptodate" in row.sim_name and row.trajectory == "nominal"),
        rows[0].sim_name,
    )
    target_cases = [
        ("nominal", "clean"),
        ("nominal", "wind_port_gust_pulses"),
        ("strong_headwind_drag", "worst_case_noise_spikes"),
    ]
    aggregators = ["raw", "plus1sigma5", "plus1p5sigma5", "plus2sigma5", "max5"]

    fig, axes = plt.subplots(len(target_cases), 1, figsize=(12, 12), sharex=False)
    fig.suptitle(f"Apogee prediction overview: {nominal_sim.rsplit('_nominal', 1)[0]}")

    for axis, (trajectory, sensor) in zip(axes, target_cases):
        scenario_rows = [
            row for row in rows
            if row.sim_name.startswith(nominal_sim.rsplit("_nominal", 1)[0])
            and row.trajectory == trajectory
            and row.sensor == sensor
        ]
        if not scenario_rows:
            continue

        altitude_by_time: dict[float, float] = {}
        for row in scenario_rows:
            altitude_by_time.setdefault(row.time_s, row.altitude_m)
        times = sorted(altitude_by_time)
        axis.plot(times, [altitude_by_time[time] for time in times], label="filtered altitude", color="black", linewidth=2)

        for aggregator in aggregators:
            selected = [row for row in scenario_rows if row.aggregator == aggregator]
            axis.plot(
                [row.time_s for row in selected],
                [row.predicted_apogee_m for row in selected],
                label=aggregator,
                linewidth=1.1,
            )
            for row in selected:
                if row.trigger:
                    axis.scatter([row.time_s], [row.altitude_m], marker="x", s=70)

        summary = next(
            row for row in summary_rows
            if row["sim_name"] == scenario_rows[0].sim_name
            and row["trajectory"] == trajectory
            and row["sensor"] == sensor
        )
        axis.axhline(float(summary["true_apogee_m"]), linestyle="--", color="gray", label="true apogee")
        axis.axvline(float(summary["true_apogee_time_s"]), linestyle=":", color="gray", label="true apogee time")
        axis.set_ylabel("altitude / prediction (m)")
        axis.set_title(f"{trajectory} + {sensor}")
        axis.grid(True, alpha=0.25)

    axes[-1].set_xlabel("time since launch (s)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=4)
    fig.tight_layout(rect=(0.0, 0.06, 1.0, 0.97))
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    source_branches = parse_openrocket_branches(args.ork)
    if not source_branches:
        raise SystemExit(f"no OpenRocket flightdata found in {args.ork}")

    args.out.mkdir(parents=True, exist_ok=True)

    all_predictions: list[PredictionRow] = []
    all_summary: list[dict[str, str]] = []
    variants = trajectory_variants()
    sensors = sensor_scenarios()

    for branch_index, source_branch in enumerate(source_branches):
        for variant_index, variant in enumerate(variants):
            branch = transform_trajectory(source_branch, variant)
            for sensor_index, sensor in enumerate(sensors):
                seed = (branch_index * 10000) + (variant_index * 100) + sensor_index
                predictions, summary = evaluate_profile(branch, variant.name, sensor, seed)
                all_predictions.extend(predictions)
                all_summary.extend(summary)

    write_predictions(args.out / "predictions.csv", all_predictions)
    write_summary(args.out / "summary.csv", all_summary)
    write_recommendation(args.out / "recommendation.md", all_summary)
    write_overview_plot(args.out / "overview.png", all_predictions, all_summary)

    print(f"OpenRocket source branches: {len(source_branches)}")
    print(f"trajectory variants: {len(variants)}")
    print(f"sensor scenarios: {len(sensors)}")
    print(f"cases per aggregator: {len(source_branches) * len(variants) * len(sensors)}")
    print(f"prediction rows: {len(all_predictions)}")
    print(f"wrote: {args.out / 'summary.csv'}")
    print(f"wrote: {args.out / 'predictions.csv'}")
    print(f"wrote: {args.out / 'recommendation.md'}")
    print(f"wrote: {args.out / 'overview.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
