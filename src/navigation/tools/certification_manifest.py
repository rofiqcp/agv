#!/usr/bin/env python3
"""Create immutable hashes for AGV navigation software/config/calibration evidence."""
from __future__ import annotations
import argparse,datetime,hashlib,json,subprocess
from pathlib import Path

def sha(p):
    h=hashlib.sha256();
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
def git(root,*args):
    r=subprocess.run(['git','-C',str(root),*args],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT);return r.stdout.strip()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--workspace',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--evidence',type=Path,nargs='*',default=[]);a=ap.parse_args();ws=a.workspace.resolve()
    files=[]
    for rel in ['src/navigation/config','src/esc/config','src/stmf4/config','calibration']:
        d=ws/rel
        if d.exists():files += [p for p in d.rglob('*') if p.is_file() and not p.name.endswith('.web.baseline')]
    files += [p.resolve() for p in a.evidence if p.is_file()]
    unique=sorted(set(files))
    manifest={'generated_at':datetime.datetime.now().astimezone().isoformat(timespec='seconds'),'workspace':str(ws),'git_head':git(ws,'rev-parse','HEAD'),
      'git_branch':git(ws,'rev-parse','--abbrev-ref','HEAD'),'git_status_porcelain':git(ws,'status','--porcelain'),'files':{str(p.relative_to(ws)) if ws in p.parents else str(p):sha(p) for p in unique}}
    sub=ws/'F4gateway'
    if sub.exists():manifest['f4gateway_head']=git(sub,'rev-parse','HEAD');manifest['f4gateway_status']=git(sub,'status','--porcelain')
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n');print(json.dumps({'out':str(a.out),'file_count':len(unique),'git_head':manifest['git_head']},indent=2))
if __name__=='__main__':raise SystemExit(main())
