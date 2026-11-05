"""Independently compare all exported points with ground.pcd + static.pcd."""
import argparse
import itertools
import json
from pathlib import Path

import numpy as np


def binary_cloud(path, marker):
    with path.open('rb') as f:
        header = []
        for line in f:
            header.append(line)
            if line == marker:
                break
        else:
            raise ValueError(f'Missing binary header: {path}')
        offset = f.tell()
    if (path.stat().st_size-offset) % 16:
        raise ValueError(f'Invalid binary payload: {path}')
    count = (path.stat().st_size-offset)//16
    key = b'POINTS ' if marker == b'DATA binary\n' else b'element vertex '
    declared = [int(line.split()[-1]) for line in header if line.startswith(key)]
    if declared != [count]:
        raise ValueError(f'Header count differs from payload: {path}')
    return np.memmap(path, dtype='<f4', mode='r', offset=offset, shape=(count, 4)) if count else np.empty((0, 4))


def verify(prefix, ground, stationary):
    sources = [binary_cloud(p, b'DATA binary\n') for p in (ground, stationary)]
    pcd = binary_cloud(Path(str(prefix)+'.pcd'), b'DATA binary\n')
    ply = binary_cloud(Path(str(prefix)+'.ply'), b'end_header\n')
    total = sum(len(a) for a in sources)
    metadata = json.loads(Path(str(prefix)+'.json').read_text())
    if not total or len(pcd) != total or len(ply) != total or metadata['points'] != total:
        raise ValueError('Point counts differ')
    offset, max_error = 0, 0.0
    with Path(str(prefix)+'.xyz').open() as xyz:
        for source in sources:
            for start in range(0, len(source), 100000):
                expected = source[start:start+100000]
                stop = offset+len(expected)
                if not np.array_equal(pcd[offset:stop], expected) or not np.array_equal(ply[offset:stop], expected):
                    raise ValueError(f'Binary coordinates/intensity differ at chunk {offset}')
                text = np.loadtxt(itertools.islice(xyz, len(expected)), ndmin=2)
                if text.shape != expected.shape or not np.isfinite(text).all():
                    raise ValueError(f'Invalid XYZ rows at chunk {offset}')
                error = float(np.abs(text-expected.astype(np.float64)).max())
                max_error = max(max_error, error)
                if error > 5.1e-10:
                    raise ValueError(f'XYZ coordinates/intensity changed: max error {error}')
                offset = stop
        if xyz.readline():
            raise ValueError('Unexpected trailing XYZ rows')
    return dict(success=True, points=total, binary_values_exact=True,
                xyz_max_absolute_error=max_error, columns=['x', 'y', 'z', 'intensity'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True, type=Path, help='Export stem, without extension')
    parser.add_argument('--ground', required=True, type=Path)
    parser.add_argument('--static', required=True, type=Path)
    args = parser.parse_args()
    result = verify(args.prefix, args.ground, args.static)
    print(json.dumps(result, indent=2))
