import sys
from pathlib import Path

# tests から app/ と tools/ を import できるように
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
