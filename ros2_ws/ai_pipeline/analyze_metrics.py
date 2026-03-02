#!/usr/bin/env python3
"""Analyze ROS-free AI pipeline CSV metrics for paper reporting."""

from __future__ import annotations

import argparse
import csv
import math
import os
from collections import Counter, defaultdict
from typing import Dict, List, Sequence


def _read_csv(path: str) -> List[Dict[str, str]]:
    if not os.path.exists(path):
        return []
    with open(path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def _to_float(v: str) -> float:
    try:
        return float(v)
    except Exception:
        return math.nan


def _valid(vals: Sequence[float]) -> List[float]:
    return [v for v in vals if not math.isnan(v)]


def _pct(vals: Sequence[float], q: float) -> float:
    xs = sorted(_valid(vals))
    if not xs:
        return math.nan
    if len(xs) == 1:
        return xs[0]
    idx = q * (len(xs) - 1)
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return xs[lo]
    t = idx - lo
    return xs[lo] * (1.0 - t) + xs[hi] * t


def _stats(vals: Sequence[float]) -> Dict[str, float]:
    xs = _valid(vals)
    if not xs:
        return {}
    return {
        "n": float(len(xs)),
        "mean": sum(xs) / len(xs),
        "p50": _pct(xs, 0.50),
        "p95": _pct(xs, 0.95),
        "p99": _pct(xs, 0.99),
        "min": min(xs),
        "max": max(xs),
    }


def _fmt_stats(label: str, s: Dict[str, float], unit: str = "") -> str:
    if not s:
        return f"{label}: n=0"
    u = f" {unit}".rstrip()
    return (
        f"{label}: n={int(s['n'])}, mean={s['mean']:.1f}{u}, p50={s['p50']:.1f}{u}, "
        f"p95={s['p95']:.1f}{u}, p99={s['p99']:.1f}{u}, min={s['min']:.1f}{u}, max={s['max']:.1f}{u}"
    )


def analyze_ailinker(rows: List[Dict[str, str]]) -> List[str]:
    out = ["## AILinker Metrics"]
    if not rows:
        out.append("No rows.")
        return out

    by_event = Counter(r.get("event", "") for r in rows)
    out.append(f"rows={len(rows)}, events={dict(by_event)}")

    gw_req = [r for r in rows if r.get("event") == "gw_req"]
    by_type: Dict[str, List[float]] = defaultdict(list)
    for r in gw_req:
        by_type[r.get("req_type", "unknown")].append(_to_float(r.get("rtt_us", "")))
    for req_type in sorted(by_type):
        out.append(_fmt_stats(f"gw_req.rtt_us[{req_type}]", _stats(by_type[req_type]), "us"))

    cmd_exec = [r for r in rows if r.get("event") == "cmd_exec"]
    out.append(_fmt_stats("cmd_exec.latency_us", _stats([_to_float(r.get("latency_us", "")) for r in cmd_exec]), "us"))
    out.append(
        _fmt_stats(
            "cmd_exec.keepalive_count",
            _stats([_to_float(r.get("keepalive_count", "")) for r in cmd_exec]),
            "",
        )
    )
    ok_rate = 0.0
    if cmd_exec:
        ok_rate = sum(1 for r in cmd_exec if str(r.get("ok", "")).lower() == "true") / len(cmd_exec)
    out.append(f"cmd_exec.ok_rate={ok_rate*100:.2f}%")

    telem = [r for r in rows if r.get("event") == "telemetry_rx" and str(r.get("ok", "")).lower() == "true"]
    if len(telem) >= 2:
        t0 = _to_float(telem[0].get("mono_us", ""))
        t1 = _to_float(telem[-1].get("mono_us", ""))
        duration_s = max(1e-9, (t1 - t0) / 1_000_000.0)
        hz = len(telem) / duration_s
        out.append(f"telemetry_rx.achieved_hz={hz:.3f} over {duration_s:.2f}s")
    else:
        out.append("telemetry_rx.achieved_hz=insufficient_data")
    return out


def analyze_behavior(rows: List[Dict[str, str]]) -> List[str]:
    out = ["## Behavior Generator Metrics"]
    if not rows:
        out.append("No rows.")
        return out

    by_event = Counter(r.get("event", "") for r in rows)
    out.append(f"rows={len(rows)}, events={dict(by_event)}")

    llm_calls = [r for r in rows if r.get("event") == "llm_call"]
    out.append(_fmt_stats("llm_call.latency_us", _stats([_to_float(r.get("latency_us", "")) for r in llm_calls]), "us"))
    status_counts = Counter(r.get("http_status", "") for r in llm_calls if r.get("http_status"))
    if status_counts:
        out.append(f"llm_call.http_status_counts={dict(status_counts)}")

    tool_calls = [r for r in rows if r.get("event") == "tool_call"]
    by_tool: Dict[str, List[float]] = defaultdict(list)
    for r in tool_calls:
        by_tool[r.get("tool", "unknown")].append(_to_float(r.get("latency_us", "")))
    for tool in sorted(by_tool):
        out.append(_fmt_stats(f"tool_call.latency_us[{tool}]", _stats(by_tool[tool]), "us"))

    tasks = Counter(r.get("event", "") for r in rows if r.get("event", "").startswith("task_"))
    if tasks:
        out.append(f"task_events={dict(tasks)}")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Analyze ai_pipeline metric CSV files")
    ap.add_argument("--ailinker-csv", default="/tmp/minitb_ai_pipeline/ailinker_metrics.csv")
    ap.add_argument("--behavior-csv", default="/tmp/minitb_ai_pipeline/behavior_metrics.csv")
    args = ap.parse_args()

    ail_rows = _read_csv(args.ailinker_csv)
    beh_rows = _read_csv(args.behavior_csv)

    print(f"AILinker CSV: {args.ailinker_csv}")
    for line in analyze_ailinker(ail_rows):
        print(line)
    print()
    print(f"Behavior CSV: {args.behavior_csv}")
    for line in analyze_behavior(beh_rows):
        print(line)


if __name__ == "__main__":
    main()

