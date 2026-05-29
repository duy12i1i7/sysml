#!/usr/bin/env python3
"""Run repeatable simulation trials for the MBSE paper validation section."""

from __future__ import annotations

import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


DEFAULT_TARGETS = [
    ("forward_near", 0.9, 0.0),
    ("forward_mid", 1.5, 0.0),
    ("upper_diag", 1.2, 0.6),
    ("lower_diag", 1.2, -0.6),
    ("forward_far", 1.8, 0.2),
]


SUMMARY_FIELDS = [
    "success",
    "duration_s",
    "role_switches",
    "min_separation_m",
    "robot1_path_m",
    "robot2_path_m",
    "requirement_refs",
]


def project_dir() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_target(value: str) -> tuple[str, float, float]:
    parts = value.split(",")
    if len(parts) == 2:
        x, y = parts
        name = f"target_{x}_{y}".replace(".", "p").replace("-", "m")
    elif len(parts) == 3:
        name, x, y = parts
    else:
        raise argparse.ArgumentTypeError(
            "target must be NAME,X,Y or X,Y, for example upper_diag,1.2,0.6"
        )
    try:
        return name.strip(), float(x), float(y)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def read_summary(summary_path: Path) -> dict[str, str] | None:
    if not summary_path.exists():
        return None
    with summary_path.open(newline="", encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    if not rows:
        return None
    return rows[-1]


def latest_summary(output_dir: Path) -> Path | None:
    summaries = sorted(output_dir.glob("summary_*.csv"))
    return summaries[-1] if summaries else None


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=12)
        return
    except subprocess.TimeoutExpired:
        pass
    process.terminate()
    try:
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def run_clean(root: Path) -> None:
    subprocess.run(
        [str(root / "scripts" / "tb4"), "clean"],
        cwd=root,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def run_trial(
    root: Path,
    trial_id: str,
    target_name: str,
    target_x: float,
    target_y: float,
    output_dir: Path,
    timeout_sec: float,
    use_gazebo: bool,
) -> dict[str, str]:
    trial_dir = output_dir / trial_id
    trial_dir.mkdir(parents=True, exist_ok=True)
    log_path = trial_dir / "launch.log"
    command = [
        str(root / "scripts" / "tb4"),
        "sim",
        f"use_gazebo:={'true' if use_gazebo else 'false'}",
        f"target_x:={target_x}",
        f"target_y:={target_y}",
        f"output_dir:={trial_dir}",
    ]

    print(
        f"[{trial_id}] target={target_name} x={target_x:.2f} y={target_y:.2f}",
        flush=True,
    )
    with log_path.open("w", encoding="utf-8") as log_file:
        process = subprocess.Popen(
            command,
            cwd=root,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
        )
        started = time.monotonic()
        summary_path: Path | None = None
        summary: dict[str, str] | None = None
        try:
            while time.monotonic() - started < timeout_sec:
                summary_path = latest_summary(trial_dir)
                if summary_path is not None:
                    summary = read_summary(summary_path)
                    if summary is not None and summary.get("success") == "1":
                        break
                if process.poll() is not None:
                    break
                time.sleep(1.0)
        finally:
            stop_process(process)

    summary_path = latest_summary(trial_dir)
    summary = read_summary(summary_path) if summary_path else None
    if summary is None:
        summary = {field: "" for field in SUMMARY_FIELDS}
        summary["success"] = "0"

    return {
        "trial_id": trial_id,
        "target_name": target_name,
        "target_x": f"{target_x:.3f}",
        "target_y": f"{target_y:.3f}",
        **{field: summary.get(field, "") for field in SUMMARY_FIELDS},
        "summary_path": str(summary_path.relative_to(root)) if summary_path else "",
        "launch_log": str(log_path.relative_to(root)),
    }


def write_aggregate(path: Path, rows: list[dict[str, str]]) -> None:
    fieldnames = [
        "trial_id",
        "target_name",
        "target_x",
        "target_y",
        *SUMMARY_FIELDS,
        "summary_path",
        "launch_log",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run paper-aligned stable simulation trials and aggregate metrics."
    )
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument(
        "--output-dir", default="metrics/paper_sim_trials", help="aggregate output root"
    )
    parser.add_argument(
        "--target",
        action="append",
        type=parse_target,
        help="NAME,X,Y or X,Y. Can be passed multiple times.",
    )
    parser.add_argument(
        "--no-gazebo",
        action="store_true",
        help="start only ROS nodes; useful for dry-run checks, not paper data",
    )
    args = parser.parse_args(argv)

    if args.repeats < 1:
        parser.error("--repeats must be >= 1")

    root = project_dir()
    output_dir = (root / args.output_dir).resolve()
    targets = args.target if args.target else DEFAULT_TARGETS
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    aggregate_path = output_dir / f"aggregate_{stamp}.csv"

    rows: list[dict[str, str]] = []
    run_clean(root)
    for repeat in range(1, args.repeats + 1):
        for index, (name, x, y) in enumerate(targets, start=1):
            trial_id = f"r{repeat:02d}_t{index:02d}_{name}"
            row = run_trial(
                root=root,
                trial_id=trial_id,
                target_name=name,
                target_x=x,
                target_y=y,
                output_dir=output_dir / stamp,
                timeout_sec=args.timeout_sec,
                use_gazebo=not args.no_gazebo,
            )
            rows.append(row)
            write_aggregate(aggregate_path, rows)
            print(
                f"[{trial_id}] success={row.get('success', '')} "
                f"duration_s={row.get('duration_s', '')}",
                flush=True,
            )
            run_clean(root)

    successes = sum(1 for row in rows if row.get("success") == "1")
    print(
        f"Finished {len(rows)} trials: {successes} success, "
        f"{len(rows) - successes} failed/timeout",
        flush=True,
    )
    print(f"Aggregate CSV: {aggregate_path.relative_to(root)}", flush=True)
    return 0 if successes == len(rows) else 1


if __name__ == "__main__":
    sys.exit(main())
