"""Make the simulation package importable for source-tree tests on ROS Humble."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent / 'src/mj_bridge'))
