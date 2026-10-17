"""Run a robosuite trial with shared XR input and synchronized stereo output."""
import argparse
from contextlib import ExitStack
from datetime import datetime,timezone
import json
import time
from pathlib import Path


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--environment',default='ExpCabinet')
    p.add_argument('--variant',choices=['robotiq','psyonic'],default='robotiq')
    p.add_argument('--controller',choices=['sew_mimic','mink'],default='sew_mimic')
    p.add_argument('--device',choices=['xr','replay','idle'],default='xr')
    p.add_argument('--sample',choices=['ipman_roll','picking_up_mustard'],default='ipman_roll')
    p.add_argument('--frames');p.add_argument('--csv')
    p.add_argument('--host',default='0.0.0.0');p.add_argument('--port',type=int,default=8080)
    p.add_argument('--rate',type=int,default=30);p.add_argument('--steps',type=int,default=0)
    p.add_argument('--duration',type=float,default=120.)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--width',type=int,default=640,help='Per-eye pixels')
    p.add_argument('--height',type=int,default=480)
    p.add_argument('--ipd',type=float,default=.065)
    p.add_argument('--fovy',type=float,default=75.)
    p.add_argument('--stream',default='udp://127.0.0.1:9000')
    p.add_argument('--no-stream',action='store_true')
    p.add_argument('--bitrate',type=int,default=12000,help='SBS H.264 kbps')
    p.add_argument('--no-safety-filter',action='store_true')
    p.add_argument('--no-head-tracking',action='store_true')
    p.add_argument('--output',default=None,help='New trial directory (never overwritten)')
    p.add_argument('--snapshot',help='Save final side-by-side PNG')
    p.add_argument('--result-file')
    p.add_argument('--plugin',action='append',default=[],help='Import an environment registration module, e.g. dexmimicgen')
    p.add_argument('--env-kwargs',default='{}',help='JSON environment options for additional robosuite tasks')
    return p


def run(args):
    import numpy as np
    from .factory import make_env,native_sim
    from .control import RobotControl
    from .recording import TrialRecorder,atomic_json
    from .stereo import StereoRenderer
    from dual_kinova3_teleop import SAMPLE_MOTIONS
    from xrt_devices.integrations.geo_kin import XRDeviceAdapter,open_motion_source
    from xrt_devices.video.gstreamer import GStreamerPublisher
    if args.rate<=0 or args.steps<0 or not np.isfinite(args.duration) or args.duration<=0:
        raise ValueError('rate/duration must be positive; steps nonnegative')
    if args.frames and args.csv:raise ValueError('choose --frames or --csv')
    if args.device!='replay' and (args.frames or args.csv):raise ValueError('recorded input requires --device replay')
    extra=json.loads(args.env_kwargs)
    if not isinstance(extra,dict):raise ValueError('--env-kwargs must be a JSON object')
    directory=Path(args.output or ('recordings/'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')))
    env=None;recorder=None;result=None;status='error';count=0
    try:
        with ExitStack() as stack:
            env=make_env(args.environment,args.variant,args.rate,args.seed,args.plugin,**extra)
            stack.callback(env.close)
            model,data=native_sim(env)
            control=RobotControl(env,args.variant,args.controller,args.rate,not args.no_safety_filter) if args.device!='idle' else None
            metadata={**vars(args),'source':'vr_robosuite','safety_filter_active':args.controller=='sew_mimic' and not args.no_safety_filter,
                      'versions':versions(),'model_nq':model.nq,'model_nv':model.nv}
            recorder=TrialRecorder(directory,metadata,model)
            # Write exact model for audit; simulation playback also uses the recorded seed/options.
            (directory/'model.xml').write_text(env.model.get_xml())
            renderer=StereoRenderer(env,args.width,args.height,args.ipd,fovy=args.fovy)
            stack.callback(renderer.close)
            publisher=None
            if not args.no_stream:
                publisher=stack.enter_context(GStreamerPublisher(2*args.width,args.height,args.rate,args.stream,args.bitrate,directory/'gstreamer.log'))
            source=None
            if args.device=='xr':
                source=XRDeviceAdapter(host=args.host,port=args.port,record_data=True,output_dir=str(directory/'input'))
                stack.callback(source.cleanup)
            elif args.device=='replay':
                source=open_motion_source(frames=args.frames or (str(SAMPLE_MOTIONS[args.sample]) if not args.csv else None),csv_file=args.csv)
            print(f'{args.environment}: {args.variant}, {args.controller}; stereo {args.width}x{args.height}/eye, IPD={args.ipd}; output={directory}',flush=True)
            start=time.monotonic();active_start=None;last_image=None
            status='preview' if args.device=='idle' else 'aborted'
            while not args.steps or count<args.steps:
                tick=time.monotonic();elapsed=tick-start
                frame=None
                if args.device=='replay':
                    if elapsed>=source.duration:status='source_exhausted';break
                    frame=source.frame_at_time(elapsed)
                elif source is not None:frame=source.get_frame()
                if frame is not None:
                    if active_start is None:active_start=tick
                    action=control.action(frame)
                    _,_,done,info=env.step(action)
                    recorder.append(data,action,True)
                    if not args.no_head_tracking:renderer.track_head(frame)
                    if (info['success'] if 'success' in info else env._check_success()):status='success';break
                    if done:status='terminated';break
                else:
                    # Pause task physics on disconnected/stale input; preserve the scene for reconnection.
                    action=control.last if control else np.zeros(env.action_dim)
                    recorder.append(data,action,False)
                last_image=renderer.render()
                if publisher:publisher.send_frame(last_image)
                count+=1
                if active_start is not None and tick-active_start>=args.duration:status='timeout';break
                if args.device=='idle' and elapsed>=args.duration:break
                time.sleep(max(0.,1/args.rate-(time.monotonic()-tick)))
            if args.snapshot:
                from PIL import Image
                path=Path(args.snapshot);path.parent.mkdir(parents=True,exist_ok=True)
                Image.fromarray(renderer.render()).save(path)
            if publisher:
                recorder.metadata['stream_counts']={k:getattr(publisher,k) for k in ['submitted','written','dropped']}
    except KeyboardInterrupt:status='aborted'
    except BaseException as exc:
        status='error'
        if recorder:recorder.metadata['error']=f'{type(exc).__name__}: {exc}'
        raise
    finally:
        if recorder:
            result=recorder.finish(status)
            if args.result_file:atomic_json(args.result_file,result)
    print(f'Trial finished: {status}; frames={count}',flush=True)
    return result


def versions():
    from importlib.metadata import version,PackageNotFoundError
    result={}
    for name in ['vr-robosuite','robosuite','mujoco','dual-kinova3-teleop','xrt-devices','geo-kin','mink']:
        try:result[name]=version(name)
        except PackageNotFoundError:pass
    import subprocess
    root=Path(__file__).resolve().parents[1]
    for label,path in [('revision',root),('dual_kinova_revision',root/'external/dual_kinova3_teleop'),('xrt_revision',root/'external/dual_kinova3_teleop/external/XRT_devices')]:
        p=subprocess.run(['git','-C',str(path),'rev-parse','HEAD'],capture_output=True,text=True)
        if p.returncode==0:result[label]=p.stdout.strip()
    return result


def main():run(parser().parse_args())
if __name__=='__main__':main()
