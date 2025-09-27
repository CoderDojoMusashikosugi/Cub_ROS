#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 rosbag2_py topic filter (include / exclude both supported)
- 指定トピックのみ残す もしくは 指定トピックを除外して他は残す
- MCAP / SQLite3 両対応

使い方:
  ## 1) 包含: 指定トピックだけ残す
  python3 filter_rosbag_topics.py \
    --input /path/to/in.mcap --output /path/to/out_dir \
    -t /fix -t /camera/image_raw --storage-in mcap --storage-out mcap

  ## 2) 除外: 指定トピックを除き、他は全部残す
  python3 filter_rosbag_topics.py \
    --input /path/to/in.mcap --output /path/to/out_dir \
    --exclude /tf --exclude /tf_static --storage-in mcap --storage-out mcap

  ## 3) 正規表現（/camera配下を全部除外）
  python3 filter_rosbag_topics.py \
    --input /path/to/in_dir_or_file --output /path/to/out_dir \
    --exclude '^/camera(/|$)' --regex

  ## 4) 包含と除外の併用（包含→除外の順で適用）
  # まず /camera配下 + /fix を対象にし、その中から /camera/camera_info を除外
  python3 filter_rosbag_topics.py \
    --input in.mcap --output out_dir \
    --regex \
    -t '^/camera(/|$)' -t '^/fix$' \
    --exclude '^/camera/camera_info$'
"""

import argparse
import os
import re
import sys
import yaml
from typing import List, Set

from rosbag2_py import (
    SequentialReader, SequentialWriter,
    StorageOptions, ConverterOptions,
    TopicMetadata, StorageFilter
)

# ---------- utils --------------------------------------------------------------

def detect_storage_id_from_input(input_path: str) -> str:
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


def load_patterns(cli_list: List[str], file_path: str | None) -> List[str]:
    patterns = list(cli_list) if cli_list else []
    if file_path:
        with open(file_path, "r", encoding="utf-8") as f:
            for line in f:
                s = line.strip()
                if s and not s.startswith("#"):
                    patterns.append(s)
    return patterns


def expand_with_regex(available: Set[str], patterns: List[str]) -> List[str]:
    if not patterns:
        return []
    out: Set[str] = set()
    compiled = []
    for p in patterns:
        try:
            compiled.append(re.compile(p))
        except re.error as e:
            raise ValueError(f"正規表現が不正です: {p} ({e})")
    for name in available:
        if any(rx.search(name) for rx in compiled):
            out.add(name)
    return sorted(out)


# ---------- main ---------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="rosbag2_py でトピック抽出（包含/除外対応）")
    ap.add_argument("--input", "-i", required=True, help="入力bag（ディレクトリ or .mcapファイル）")
    ap.add_argument("--output", "-o", required=True, help="出力bag（新規ディレクトリ名を推奨）")
    ap.add_argument("--storage-in", default=None, help='入力ストレージID（mcap / sqlite3）。未指定なら自動判定')
    ap.add_argument("--storage-out", default="mcap", help='出力ストレージID（デフォルト: mcap）')

    # 包含指定（keep only）
    ap.add_argument("-t", "--topic", dest="include_topics", action="append", default=[],
                    help="包含するトピック（複数可、完全一致。--regex 指定時は正規表現として扱う）")
    ap.add_argument("--topics-file", dest="include_file", help="包含トピック一覧ファイル（1行1件、--regex可）")

    # 除外指定（drop）
    ap.add_argument("--exclude", dest="exclude_topics", action="append", default=[],
                    help="除外するトピック（複数可、完全一致。--regex 指定時は正規表現として扱う）")
    ap.add_argument("--exclude-file", dest="exclude_file",
                    help="除外トピック一覧ファイル（1行1件、--regex可）")

    # 正規表現フラグ（包含/除外の両方に適用）
    ap.add_argument("--regex", action="store_true", help="トピック指定を正規表現として展開する")

    args = ap.parse_args()

    in_path = os.path.abspath(args.input)
    out_path = os.path.abspath(args.output)

    if os.path.exists(out_path) and os.path.isdir(out_path) and os.listdir(out_path):
        print(f"[ERROR] 出力ディレクトリが既に存在し中身があります: {out_path}", file=sys.stderr)
        sys.exit(2)

    storage_in = args.storage_in or detect_storage_id_from_input(in_path)
    storage_out = args.storage_out

    # 1) reader open
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=in_path, storage_id=storage_in),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    # 2) available topics
    topics_and_types = reader.get_all_topics_and_types()
    available: Set[str] = {t.name for t in topics_and_types}
    type_map = {t.name: t.type for t in topics_and_types}

    # 3) collect include / exclude patterns
    include_patterns = load_patterns(args.include_topics, args.include_file)
    exclude_patterns = load_patterns(args.exclude_topics, args.exclude_file)

    # 4) expand patterns (if --regex)
    if args.regex:
        include_expanded = set(expand_with_regex(available, include_patterns))
        exclude_expanded = set(expand_with_regex(available, exclude_patterns))
    else:
        include_expanded = set(include_patterns)
        exclude_expanded = set(exclude_patterns)

    # 5) compute keep set
    # ルール:
    # - include が空なら「全トピック」を起点にする（= 除外モード）
    # - include が非空なら「include されたもの」から開始（= 包含 or 併用）
    if include_expanded:
        base_keep = include_expanded & available
        # 警告: include指定のうち存在しないもの
        missing_inc = sorted(include_expanded - available)
        if missing_inc:
            print("[WARN] 存在しない包含トピック:", *missing_inc, sep="\n  - ", file=sys.stderr)
    else:
        base_keep = set(available)  # 除外モード

    # 除外適用
    missing_exc = sorted(exclude_expanded - available)
    if missing_exc:
        print("[WARN] 存在しない除外トピック:", *missing_exc, sep="\n  - ", file=sys.stderr)

    keep_topics = sorted((base_keep - exclude_expanded) & available)

    if not keep_topics:
        print("[ERROR] 抽出後のトピック集合が空です。指定を見直してください。", file=sys.stderr)
        sys.exit(3)

    # 6) writer open
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=out_path, storage_id=storage_out),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    # 7) register topics
    for t in keep_topics:
        writer.create_topic(TopicMetadata(
            name=t,
            type=type_map[t],
            serialization_format="cdr",
        ))

    # 8) copy with filter
    reader.set_filter(StorageFilter(topics=keep_topics))

    msg_count = 0
    try:
        while reader.has_next():
            topic, data, t = reader.read_next()
            writer.write(topic, data, t)
            msg_count += 1
    finally:
        reader.set_filter(StorageFilter())

    print(f"[DONE] 出力: {out_path}")
    print(f"  storage_in={storage_in}, storage_out={storage_out}")
    print(f"  topics_kept={len(keep_topics)}")
    for t in keep_topics:
        print(f"    - {t}")
    print(f"  messages_copied={msg_count}")


if __name__ == "__main__":
    main()
