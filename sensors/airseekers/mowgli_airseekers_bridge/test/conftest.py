# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""The pure modules import without ROS; make the package importable from the
source tree (python3 -m pytest sensors/airseekers/mowgli_airseekers_bridge/test)."""
import os
import sys

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG not in sys.path:
    sys.path.insert(0, PKG)
