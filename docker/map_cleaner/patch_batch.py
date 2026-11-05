"""Small batch-only changes to pinned upstream; filtering algorithm is unchanged."""
from pathlib import Path
import sys

root = Path(sys.argv[1])
path = root / 'src/map_cleaner.cpp'
source = path.read_text()
old = '    ros::Rate rate(100);'
assert source.count(old) == 1
source = source.replace(old, '    if (!nh_.param<bool>("/map_cleaner/keep_alive", false)) return;\n' + old)
for line in source.splitlines():
    if line.strip().startswith('saveCloud(save_dir_'):
        call = line.strip().rstrip(';')
        source = source.replace(line, '    if (!' + call + ') throw std::runtime_error("PCD output failed");')
path.write_text(source)
