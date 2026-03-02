#!/usr/bin/env python3
"""Interactive ROS-free AI control CLI."""

from __future__ import annotations

import argparse
import os
import sys
import time
from types import SimpleNamespace

import requests

from ai_behavior_gen import Orchestrator


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Interactive AI control CLI (ROS-free)")
    p.add_argument("--ailinker-url", default="http://127.0.0.1:7070")
    p.add_argument("--gemini-model", default="gemini-2.5-flash-lite")
    p.add_argument("--gemini-api-key", default=os.environ.get("GEMINI_API_KEY", ""))
    p.add_argument("--max-steps", type=int, default=2)
    p.add_argument("--loop-sleep-ms", type=int, default=900)
    p.add_argument("--llm-max-retries", type=int, default=2)
    p.add_argument("--llm-backoff-initial-ms", type=int, default=1500)
    p.add_argument("--llm-backoff-max-ms", type=int, default=20000)
    p.add_argument("--metrics-csv-path", default="/tmp/minitb_ai_pipeline/behavior_metrics.csv")
    p.add_argument("--temperature", type=float, default=0.1)
    p.add_argument("--gemini-timeout-s", type=float, default=20.0)
    p.add_argument("--ailinker-timeout-s", type=float, default=3.0)
    p.add_argument("--task", default="", help="If provided, run one task and exit")
    return p.parse_args()


def _health(ailinker_url: str) -> bool:
    try:
        r = requests.get(f"{ailinker_url.rstrip('/')}/context", timeout=2.0)
        return r.ok
    except Exception:
        return False


def _print_json(url: str) -> None:
    try:
        r = requests.get(url, timeout=2.0)
        print(r.text)
    except Exception as exc:
        print(f"Request failed: {exc}")


def _post_json(url: str) -> None:
    try:
        r = requests.post(url, json={}, timeout=2.0)
        print(r.text)
    except Exception as exc:
        print(f"Request failed: {exc}")


def _run_task(base: argparse.Namespace, task_text: str) -> int:
    task_id = f"task_{int(time.time())}"
    ns = SimpleNamespace(
        task=task_text,
        task_id=task_id,
        ailinker_url=base.ailinker_url,
        ailinker_timeout_s=base.ailinker_timeout_s,
        gemini_model=base.gemini_model,
        gemini_api_key=base.gemini_api_key,
        gemini_timeout_s=base.gemini_timeout_s,
        temperature=base.temperature,
        max_steps=base.max_steps,
        loop_sleep_ms=base.loop_sleep_ms,
        llm_max_retries=base.llm_max_retries,
        llm_backoff_initial_ms=base.llm_backoff_initial_ms,
        llm_backoff_max_ms=base.llm_backoff_max_ms,
        metrics_csv_path=base.metrics_csv_path,
        stop_on_exit=True,
    )
    return Orchestrator(ns).run()


def main() -> None:
    args = _args()
    if not args.gemini_api_key:
        print("Missing GEMINI_API_KEY. Set environment variable or pass --gemini-api-key.")
        sys.exit(2)
    if not _health(args.ailinker_url):
        print(f"AILinker is not reachable at {args.ailinker_url}")
        print("Start stack first using start_rosfree_stack.sh")
        sys.exit(2)

    if args.task:
        sys.exit(_run_task(args, args.task))

    print("MiniTB AI Control CLI (ROS-free)")
    print(f"AILinker: {args.ailinker_url}")
    print(f"Model: {args.gemini_model}")
    print("Type a task and press Enter.")
    print("Commands: :state :caps :metrics :stop :estop :quit")

    while True:
        try:
            line = input("task> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break
        if not line:
            continue
        if line in (":q", ":quit"):
            break
        if line == ":state":
            _print_json(f"{args.ailinker_url.rstrip('/')}/state")
            continue
        if line == ":caps":
            _print_json(f"{args.ailinker_url.rstrip('/')}/capabilities")
            continue
        if line == ":metrics":
            _print_json(f"{args.ailinker_url.rstrip('/')}/metrics")
            continue
        if line == ":stop":
            _post_json(f"{args.ailinker_url.rstrip('/')}/stop")
            continue
        if line == ":estop":
            _post_json(f"{args.ailinker_url.rstrip('/')}/estop")
            continue

        rc = _run_task(args, line)
        if rc not in (0, 130):
            print(f"Task failed (exit={rc}).")


if __name__ == "__main__":
    main()

