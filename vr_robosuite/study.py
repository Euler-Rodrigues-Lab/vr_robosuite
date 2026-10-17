"""Counterbalanced trial plans, atomic results and explicit interrupted runs."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
from datetime import datetime,timezone
from .recording import atomic_json

ENVS={'1':'ExpCabinet','2':'ExpGlassGap','3':'ExpHandover'}
CONTROLLERS={'1':'sew_mimic','2':'mink'}


def make_plan(participant,order,controllers,seed):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}',participant):raise ValueError('use a pseudonymous ID containing letters, digits, _ or -')
    if not order or any(x not in ENVS for x in order):raise ValueError('environment order uses codes 1, 2, 3')
    if len(controllers)!=2*len(order) or any(x not in CONTROLLERS for x in controllers):raise ValueError('provide exactly two controller codes (1=SEW, 2=MINK) per environment')
    return {'participant':participant,'order':order,'controllers':controllers,'seed':seed,'runs':[
        {'environment':ENVS[e],'controller':CONTROLLERS[controllers[2*i+j]],'seed':seed+i,'status':'pending','attempts':[]}
        for i,e in enumerate(order) for j in range(2)]}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--participant',required=True);p.add_argument('--order',default='132')
    p.add_argument('--controllers',default='121221');p.add_argument('--seed',type=int,default=0)
    p.add_argument('--output',default='recordings/studies');p.add_argument('--plan-only',action='store_true')
    p.add_argument('--auto',action='store_true',help='Proceed between trials without operator Enter')
    args,extra=p.parse_known_args()
    expected=make_plan(args.participant,args.order,args.controllers,args.seed)
    expected['trial_arguments']=extra
    directory=Path(args.output)/args.participant;path=directory/'plan.json'
    if path.exists():
        plan=json.loads(path.read_text())
        for key in ['participant','order','controllers','seed','trial_arguments']:
            if plan[key]!=expected[key]:raise ValueError('existing plan differs; use a new participant/output directory')
    else:plan=expected;atomic_json(path,plan)
    if args.plan_only:print(path);return
    for i,run in enumerate(plan['runs']):
        if run['status'] in ('success','timeout','terminated'):continue
        if not args.auto:input(f"Trial {i+1}: {run['environment']} / {run['controller']}. Press Enter when ready.")
        trial=directory/f"trial_{i+1:02d}_{datetime.now(timezone.utc):%Y%m%dT%H%M%S_%fZ}"
        result=directory/'result.json';result.unlink(missing_ok=True)
        run['status']='running';atomic_json(path,plan)
        cmd=[sys.executable,'-m','vr_robosuite.cli',*extra,'--environment',run['environment'],
             '--controller',run['controller'],'--seed',str(run['seed']),'--output',str(trial),'--result-file',str(result)]
        try:
            process=subprocess.run(cmd)
            outcome=json.loads(result.read_text()) if result.exists() else {'status':'error','error':'worker produced no result'}
            if process.returncode:outcome['status']='error';outcome['returncode']=process.returncode
        except KeyboardInterrupt:outcome={'status':'aborted'}
        run['status']=outcome['status'];run['attempts'].append({'directory':str(trial),'result':outcome})
        atomic_json(path,plan)
        if run['status'] not in ('success','timeout','terminated'):
            print(f"Stopped after {run['status']}; resume this plan to retry the unfinished trial.");break
if __name__=='__main__':main()
