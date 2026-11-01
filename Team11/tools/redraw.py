#!/usr/bin/env python3
import sys
from pathlib import Path
from compare_plot import update_plots
root=Path(__file__).resolve().parents[1]/"results"
folders=[Path(x) for x in sys.argv[1:]] or [p for p in root.glob("route_*") if p.is_dir()]
for folder in folders:
 try:print(folder,update_plots(folder))
 except Exception as e:print(folder,"ERROR",str(e))
