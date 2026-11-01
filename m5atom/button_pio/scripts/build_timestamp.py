"""Regenerate the displayed application build time, including incremental builds."""
from datetime import datetime, timedelta, timezone
from pathlib import Path

from SCons.Script import COMMAND_LINE_TARGETS

Import("env")

build_dir = Path(env.subst("$BUILD_DIR"))
env.Append(CPPPATH=[str(build_dir)])

metadata_or_clean = {
    "_idedata", "__idedata", "idedata", "clean", "clean_microros", "clean_libmicroros"
}
if not metadata_or_clean.intersection(COMMAND_LINE_TARGETS):
    built_at = datetime.now(timezone(timedelta(hours=9)))
    timestamp = built_at.strftime("%Y-%m-%d %H:%M:%S JST")
    contents = (
        '#pragma once\n'
        f'#define DIAL_BUILD_DATE "{built_at:%Y-%m-%d}"\n'
        f'#define DIAL_BUILD_TIME "{built_at:%H:%M:%S} JST"\n'
    )
    build_dir.mkdir(parents=True, exist_ok=True)
    header = build_dir / "build_timestamp.h"
    if not header.exists() or header.read_text() != contents:
        header.write_text(contents)
    print("M5Dial build: " + timestamp)
