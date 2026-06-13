#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import yaml
from typing import Dict, List, Optional

from rosbag2_py import (
    SequentialReader,
    StorageOptions,
    ConverterOptions,
    StorageFilter
)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def detect_storage_id_from_input(input_path: str) -> str:
    """
    入力パスからストレージID(mcap or sqlite3)を自動判定する
    """
    if os.path.isdir(input_path):
        meta = os.path.join(input_path, "metadata.yaml")
        if os.path.exists(meta):
            try:
                with open(meta, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                si = None
                if isinstance(data, dict):
                    info = data.get("rosbag2_bagfile_information") or data.get("bagfile_information")
                    if isinstance(info, dict):
                        si = info.get("storage_identifier")
                if si:
                    return si
            except Exception:
                pass
        return "mcap"
    if input_path.endswith(".mcap"):
        return "mcap"
    return "sqlite3"

def main():
    parser = argparse.ArgumentParser(description="rosbagのトピックごとにタイムスタンプのジャンプ（欠損）を検出します。")
    parser.add_argument("input", help="入力rosbagのパス（ディレクトリまたは.mcapファイル）")
    parser.add_argument("--output", "-o", default="jumps.txt", help="出力テキストファイル名 (デフォルト: jumps.txt)")
    parser.add_argument("--multiplier", "-m", type=float, default=1.5, help="周期の中央値の何倍以上をジャンプとみなすか (デフォルト: 1.5)")
    parser.add_argument("--topic", action="append", help="チェックするトピック名（複数指定可）。未指定なら全トピックを対象にします。")
    parser.add_argument("--storage", "-s", default=None, help="ストレージID (mcap/sqlite3)。未指定なら自動判定。")
    parser.add_argument("--use-bag-time", action="store_true", help="Header時刻の代わりにBag受信時刻を使用します。")

    args = parser.parse_args()

    in_path = os.path.abspath(args.input)
    storage_id = args.storage or detect_storage_id_from_input(in_path)

    print(f"Opening bag: {in_path} (storage: {storage_id})")

    # 1) Readerの初期化
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=in_path, storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    # 2) 存在するトピックの確認と型情報の読み込み
    all_topics_and_types = reader.get_all_topics_and_types()
    target_topics = []
    msg_classes = {}

    type_map = {t.name: t.type for t in all_topics_and_types}
    
    if args.topic:
        for t_name in args.topic:
            if t_name in type_map:
                target_topics.append(t_name)
            else:
                print(f"[WARN] 指定されたトピックが見つかりません: {t_name}")
    else:
        target_topics = [t.name for t in all_topics_and_types]

    if not target_topics:
        print("[ERROR] 対象となるトピックが見つかりません。")
        sys.exit(1)

    # 型情報をロード (Header取得のため)
    if not args.use_bag_time:
        for t_name in target_topics:
            try:
                msg_classes[t_name] = get_message(type_map[t_name])
            except Exception as e:
                print(f"[WARN] 型のロードに失敗しました: {t_name} [{type_map[t_name]}] ({e})")

    # 3) フィルタの設定
    reader.set_filter(StorageFilter(topics=target_topics))

    # 4) 時刻データの収集
    first_timestamps = {} # topic -> first_ns
    last_timestamps = {} # topic -> last_ns
    intervals = {}       # topic -> list of (start_ns, end_ns, diff_ns)
    
    count = 0
    print(f"Collecting timestamps...")

    try:
        while reader.has_next():
            topic_name, data, bag_timestamp = reader.read_next()
            
            current_ns = bag_timestamp
            
            if not args.use_bag_time and topic_name in msg_classes:
                try:
                    msg = deserialize_message(data, msg_classes[topic_name])
                    if hasattr(msg, 'header'):
                        current_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                except Exception:
                    pass # デシリアライズ失敗は受信時刻で代用
            
            if topic_name not in first_timestamps:
                first_timestamps[topic_name] = current_ns

            if topic_name in last_timestamps:
                diff_ns = current_ns - last_timestamps[topic_name]
                if topic_name not in intervals:
                    intervals[topic_name] = []
                intervals[topic_name].append((last_timestamps[topic_name], current_ns, diff_ns))

            last_timestamps[topic_name] = current_ns
            count += 1
            if count % 1000 == 0:
                print(f"Processed {count} messages...", end='\r')

    finally:
        print(f"\nData collection complete. Processed {count} messages.")

    # 5) ジャンプの分析
    import statistics

    jumps_detected = {} # topic -> list of (start_ns, end_ns, diff_ns, time_since_last_jump_ns)
    median_periods = {}

    print("Analyzing jumps based on median intervals...")

    for topic, data in intervals.items():
        if not data:
            continue
        
        diffs = [d[2] for d in data if d[2] > 0] # 逆転や重複を除いて中央値を計算
        if not diffs:
            continue

        median_ns = statistics.median(diffs)
        median_periods[topic] = median_ns
        threshold_ns = median_ns * args.multiplier

        last_jump_end_ns = None

        for start, end, diff in data:
            if diff > threshold_ns or diff < 0:
                if topic not in jumps_detected:
                    jumps_detected[topic] = []
                
                time_since_last_jump_ns = None
                if last_jump_end_ns is not None:
                    time_since_last_jump_ns = start - last_jump_end_ns
                
                jumps_detected[topic].append((start, end, diff, time_since_last_jump_ns))
                last_jump_end_ns = end

    # 6) 結果の書き出し
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(f"ROS2 Bag Timestamp Jump Report (Median-based)\n")
        f.write(f"============================================\n")
        f.write(f"Input: {in_path}\n")
        f.write(f"Multiplier: {args.multiplier}x median period\n")
        f.write(f"Time Source: {'Bag Time' if args.use_bag_time else 'Header Time (fallback to Bag Time)'}\n")
        f.write(f"--------------------------------------------\n\n")

        total_jumps = 0
        for topic in sorted(target_topics):
            jumps = jumps_detected.get(topic, [])
            median_ns = median_periods.get(topic)
            first_ns = first_timestamps.get(topic)
            
            if first_ns is not None:
                f.write(f"Topic: {topic}\n")
                f.write(f"  First message timestamp: {first_ns / 1e9:.6f}s\n")
                
                if median_ns is not None:
                    f.write(f"  Median period: {median_ns / 1e9:.6f}s ({1e9/median_ns:.2f} Hz)\n")
                    
                    if not jumps:
                        f.write(f"  No jumps detected (all gaps < {median_ns * args.multiplier / 1e9:.6f}s).\n\n")
                        continue

                    total_jumps += len(jumps)
                    for i, (start, end, diff, since_last) in enumerate(jumps):
                        start_s = start / 1e9
                        end_s = end / 1e9
                        diff_s = diff / 1e9
                        
                        since_last_str = ""
                        if since_last is not None:
                            since_last_str = f", since last jump: {since_last/1e9:.3f}s"

                        if diff > 0:
                            f.write(f"  [{i+1}] Jump: {diff_s:.6f}s ({diff/median_ns:.2f}x median) at {start_s:.6f}{since_last_str}\n")
                        else:
                            f.write(f"  [{i+1}] Backward Jump: {diff_s:.6f}s at {start_s:.6f}{since_last_str}\n")
                    f.write("\n")
                else:
                    f.write(f"  (Not enough messages to calculate median period)\n\n")

        if total_jumps == 0:
            f.write("Overall: No significant jumps detected.\n")
        else:
            f.write(f"Overall: Total jumps detected: {total_jumps}\n")

    print(f"Results saved to: {args.output}")

if __name__ == "__main__":
    main()
