import json
import numpy as np
import pytest
from types import SimpleNamespace
from vr_robosuite.study import make_plan
from vr_robosuite.recording import TrialRecorder,atomic_json


def test_counterbalanced_plan_and_validation():
    plan=make_plan('P01','132','121221',10)
    assert [r['environment'] for r in plan['runs']]==['ExpCabinet']*2+['ExpHandover']*2+['ExpGlassGap']*2
    assert [r['controller'] for r in plan['runs']]==['sew_mimic','mink','sew_mimic','mink','mink','sew_mimic']
    assert [r['seed'] for r in plan['runs']]==[10,10,11,11,12,12]
    with pytest.raises(ValueError):make_plan('../P01','123','121212',0)
    with pytest.raises(ValueError):make_plan('P01','123','12',0)


def test_trial_records_error_without_fabricating_failure(tmp_path):
    model=SimpleNamespace(nq=2,nv=2,nu=1)
    r=TrialRecorder(tmp_path/'trial',{'seed':5},model)
    r.append(SimpleNamespace(qpos=np.ones(2),qvel=np.zeros(2),ctrl=np.zeros(1),time=.1),np.zeros(3),False)
    result=r.finish('error',error='test interruption')
    assert result['status']=='error' and result['frames']==1
    assert json.loads((tmp_path/'trial/trial.json').read_text())['error']=='test interruption'
    with pytest.raises(FileExistsError):TrialRecorder(tmp_path/'trial',{},model)
