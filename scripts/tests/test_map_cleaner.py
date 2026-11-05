"""Preflight tests: bad inputs must fail before Docker starts or output is touched."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('map_cleaner_cli', Path(__file__).parents[1]/'internal/map_cleaner.py')
CLI = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CLI)


class Preflight(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.source = self.root/'map with spaces'
        f = self.source/'frames'
        f.mkdir(parents=True)
        (self.source/'traj_lidar.txt').write_text('100 60 30 2 0 0 0 1\n101 61 30 2 0 0 0 1\n')
        (f/'report.json').write_text(json.dumps(dict(success=True, frames=2, sensor='mid360')))
        with (f/'manifest.csv').open('w') as out:
            writer = csv.writer(out); writer.writerow(['row','timestamp','frame_id','points'])
            for i in range(2):
                writer.writerow([i,100+i,30+i,1])
                d = f/f'{i:08d}'; d.mkdir()
                (d/'points_compact.bin').write_bytes(b'\0'*12)
                (d/'intensities_compact.bin').write_bytes(b'\0'*4)

    def tearDown(self):
        self.temp.cleanup()

    def test_valid_interval(self):
        info = CLI.validate_frames(self.source, 0, -1)
        self.assertEqual(info['frames_selected'], 2)
        self.assertEqual(info['selected_scan_points'], 2)

    def test_bad_interval(self):
        for interval in [(-1,1),(1,1),(0,3),(0,-2)]:
            with self.assertRaises(ValueError): CLI.validate_frames(self.source,*interval)

    def test_truncated_frame(self):
        (self.source/'frames/00000001/points_compact.bin').write_bytes(b'broken')
        with self.assertRaises(ValueError): CLI.validate_frames(self.source,0,-1)

    def test_invalid_rotation(self):
        (self.source/'traj_lidar.txt').write_text('100 60 30 2 0 0 0 0\n101 61 30 2 0 0 0 1\n')
        with self.assertRaises(ValueError): CLI.validate_frames(self.source,0,-1)

    def test_wrong_sensor_and_existing_output(self):
        output = self.root/'result'
        args = argparse.Namespace(input=str(self.source),output=str(output),sensor='velodyne',start=0,end=-1)
        with patch.object(CLI,'command') as command:
            with self.assertRaises(ValueError): CLI.run(args)
            command.assert_not_called()
        self.assertFalse(output.exists())
        output.mkdir(); (output/'precious.txt').write_text('keep')
        args.sensor='mid360'
        with patch.object(CLI,'command') as command:
            with self.assertRaises(ValueError): CLI.run(args)
            command.assert_not_called()
        self.assertEqual((output/'precious.txt').read_text(),'keep')


if __name__ == '__main__':
    unittest.main()
