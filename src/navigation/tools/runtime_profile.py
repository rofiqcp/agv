#!/usr/bin/env python3
"""Lightweight CPU/RAM/Jetson tegrastats capture for deployment profiling."""
from __future__ import annotations
import argparse, csv, shutil, subprocess, time
from datetime import datetime
from pathlib import Path

def cpu_snapshot():
    fields=Path('/proc/stat').read_text().splitlines()[0].split()[1:]
    nums=[int(x) for x in fields]; idle=nums[3]+(nums[4] if len(nums)>4 else 0); return sum(nums),idle

def mem_used_mb():
    info={}
    for line in Path('/proc/meminfo').read_text().splitlines():
        k,v=line.split(':',1); info[k]=int(v.strip().split()[0])
    return (info.get('MemTotal',0)-info.get('MemAvailable',0))/1024.0

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--seconds',type=float,default=60); ap.add_argument('--hz',type=float,default=2); ap.add_argument('--output',default='~/.ros/agv_gui_reports')
    a=ap.parse_args(); root=Path(a.output).expanduser(); root.mkdir(parents=True,exist_ok=True)
    out=root/f"runtime_profile_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    total0,idle0=cpu_snapshot(); rows=[]; start=time.monotonic(); dt=1/max(0.2,a.hz)
    while time.monotonic()-start<a.seconds:
        time.sleep(dt); total,idle=cpu_snapshot(); dtotal=max(1,total-total0); didle=idle-idle0; cpu=100*(1-didle/dtotal); total0,idle0=total,idle
        row={'t_sec':time.monotonic()-start,'cpu_percent':cpu,'ram_used_mb':mem_used_mb()}
        if shutil.which('tegrastats'):
            try:
                txt=subprocess.check_output(['tegrastats','--interval','100','--count','1'],text=True,timeout=1).strip(); row['tegrastats']=txt
            except Exception: pass
        rows.append(row)
    with out.open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=sorted({k for r in rows for k in r})); w.writeheader(); w.writerows(rows)
    print(out)
if __name__=='__main__': main()
