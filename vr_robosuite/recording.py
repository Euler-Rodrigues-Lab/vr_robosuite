"""Incremental trial data; outcomes are never inferred from a missing result."""
from pathlib import Path
import json,os,time
import h5py
import numpy as np


def atomic_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w') as f:
        json.dump(value,f,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(temporary,path)


class TrialRecorder:
    def __init__(self,directory,metadata,model):
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=False)
        self.metadata={**metadata,'status':'running','started_ns':time.time_ns()}
        atomic_json(self.directory/'trial.json',self.metadata)
        self.file=h5py.File(self.directory/'trajectory.h5','x');self.count=0
        self.file.attrs['metadata']=json.dumps(metadata)
        self.datasets={}
        for name,size in [('qpos',model.nq),('qvel',model.nv),('ctrl',model.nu),('time',3)]:
            self.datasets[name]=self.file.create_dataset(name,shape=(0,size),maxshape=(None,size),chunks=True,dtype='f8')
    def append(self,data,action,valid):
        if 'action' not in self.datasets:
            self.datasets['action']=self.file.create_dataset('action',shape=(0,len(action)),maxshape=(None,len(action)),chunks=True,dtype='f8')
        values={'qpos':data.qpos,'qvel':data.qvel,'ctrl':data.ctrl,'time':[time.monotonic(),data.time,float(valid)],'action':action}
        for name,value in values.items():
            d=self.datasets[name];d.resize(self.count+1,axis=0);d[self.count]=value
        self.count+=1
        if self.count%30==0:self.file.flush()
    def finish(self,status,**result):
        self.file.flush();self.file.close()
        self.metadata.update(status=status,frames=self.count,finished_ns=time.time_ns(),**result)
        atomic_json(self.directory/'trial.json',self.metadata)
        return self.metadata
