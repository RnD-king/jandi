#!/usr/bin/env python3

import argparse
import csv
import glob
import os
import sys
import time

import matplotlib.pyplot as plt


LOG_DIR = "/tmp/forward_walk_logs"
JOINTS = ["R0", "R1", "R2", "R3", "R4", "R5", "L0", "L1", "L2", "L3", "L4", "L5"]


def find_latest_csv() -> str:
    candidates = sorted(glob.glob(os.path.join(LOG_DIR, "leg_torque_*.csv")))
    if not candidates:
        raise FileNotFoundError(f"CSV 로그를 찾을 수 없습니다: {LOG_DIR}")
    return candidates[-1]


def load_csv(path: str, value_type: str):
    time_sec = []
    series = {joint: [] for joint in JOINTS}

    suffix = "torque_nm" if value_type == "torque" else "current_mA"

    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            time_sec.append(float(row["ros_time_sec"]))
            for joint in JOINTS:
                series[joint].append(float(row[f"{joint}_{suffix}"]))

    if not time_sec:
        raise ValueError(f"CSV에 데이터가 없습니다: {path}")

    t0 = time_sec[0]
    rel_time = [t - t0 for t in time_sec]
    return rel_time, series


def draw(path: str, value_type: str):
    rel_time, series = load_csv(path, value_type)

    ylabel = "Torque [Nm]" if value_type == "torque" else "Current [mA]"
    title = "Leg Joint Torque" if value_type == "torque" else "Leg Joint Current"

    plt.clf()
    fig = plt.gcf()
    fig.suptitle(f"{title}\n{path}", fontsize=12)

    for idx, joint in enumerate(JOINTS, start=1):
        ax = fig.add_subplot(3, 4, idx)
        ax.plot(rel_time, series[joint], linewidth=1.5)
        ax.set_title(joint)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0.03, 1, 0.95])


def main():
    parser = argparse.ArgumentParser(description="forward_walk 다리 조인트 토크/전류 CSV 플롯")
    parser.add_argument("csv_path", nargs="?", help="플롯할 CSV 경로. 생략하면 최신 로그 사용")
    parser.add_argument("--current", action="store_true", help="토크 대신 전류[mA]를 그림")
    parser.add_argument("--watch", action="store_true", help="파일을 주기적으로 다시 읽어서 실시간 갱신")
    parser.add_argument("--interval", type=float, default=0.5, help="watch 갱신 주기(초), 기본값 0.5")
    args = parser.parse_args()

    csv_path = args.csv_path or find_latest_csv()
    value_type = "current" if args.current else "torque"

    plt.figure(figsize=(16, 10))

    if not args.watch:
        draw(csv_path, value_type)
        plt.show()
        return

    plt.ion()
    while plt.fignum_exists(1):
        try:
            draw(csv_path, value_type)
            plt.pause(args.interval)
            time.sleep(args.interval)
        except Exception as exc:
            print(f"[plot_leg_torque] {exc}", file=sys.stderr)
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
