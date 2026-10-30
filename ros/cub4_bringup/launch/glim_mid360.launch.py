"""Build a Cub4 mid360 GLIM map, optionally constrained by RTK GNSS."""

from pathlib import Path
import runpy


def generate_launch_description():
    common = runpy.run_path(str(Path(__file__).with_name('glim_mapping_common.py')))
    return common['generate_mapping_launch_description']('mid360')
