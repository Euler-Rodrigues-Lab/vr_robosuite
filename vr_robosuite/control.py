"""Shared Dual Kinova sessions -> robosuite absolute joint-position actions."""
import numpy as np
from .factory import native_sim


def pinch_distance(frame, side):
    """Thumb/index tip separation in metres; keep raw distance fallback for replays."""
    fingers=getattr(frame,side+'_fingers',None) or {}
    thumb=fingers.get('thumb',{}).get('thumb_tip')
    index=fingers.get('index',{}).get('index_finger_tip')
    if thumb is not None and index is not None:
        thumb,index=np.asarray(thumb,dtype=float),np.asarray(index,dtype=float)
        if thumb.shape==(3,) and index.shape==(3,) and np.isfinite([thumb,index]).all():
            return float(np.linalg.norm(index-thumb))
    distance=getattr(frame,side+'_gripper_val',None)
    if distance is not None and np.isfinite(distance) and distance>=0:
        return float(distance)
    return None


class RobotControl:
    def __init__(self,env,variant='robotiq',controller='sew_mimic',rate=30,safety=True,retarget_mode='pose'):
        from dual_kinova3_teleop.session import make_session
        if retarget_mode not in ('pose','tcp'):raise ValueError('retarget_mode must be pose or tcp')
        if controller!='sew_mimic' and retarget_mode!='pose':
            raise ValueError('TCP retargeting requires --controller sew_mimic')
        self.env,self.variant=env,variant
        self.retarget_mode=retarget_mode
        self.model,self.data=native_sim(env)
        self.robot=env.robots[0];self.prefix=self.robot.robot_model.naming_prefix
        self.addresses={s:np.array([self.model.joint(f'{self.prefix}{s}_Actuator{i}').qposadr[0] for i in range(1,8)]) for s in ['right','left']}
        self.grips={s:np.zeros(10 if variant=='psyonic' else 1) for s in self.addresses}
        self.hand_session=None
        if controller=='sew_mimic':
            self.session=make_session(hand='psyonic' if variant=='psyonic' else None,
                retarget_mode=retarget_mode,collision_avoidance=safety,limited=True,shoulder_center_fallback=False)
        else:
            self.session=self._mink(rate)
            if variant=='psyonic':self.hand_session=make_session(hand='psyonic',retarget_mode='pose')
        q=self.feedback();self.session.reset(q['right'],q['left'])
        self.last=self.robot.create_action_vector({**q,**{s+'_gripper':v for s,v in self.grips.items()}})
    def feedback(self):return {s:self.data.qpos[a].copy() for s,a in self.addresses.items()}
    def action(self,frame):
        if frame is None:return self.last.copy()
        from geo_kin_core.types import RetargetFrame
        q=self.feedback()
        out=self.session.solve(frame,q_current_right=q['right'],q_current_left=q['left'])
        hands=out
        if self.hand_session is not None:
            hands=self.hand_session.solve(RetargetFrame(left_fingers=frame.left_fingers,right_fingers=frame.right_fingers))
        commands={}
        for side in self.addresses:
            target=getattr(out,'q_goal_'+side,None)
            commands[side]=q[side] if target is None else target
            if self.variant=='psyonic':
                hand=getattr(hands,'q_goal_'+side+'_hand',None)
                if hand is not None:self.grips[side]=np.asarray(hand)
            else:
                distance=pinch_distance(frame,side)
                # XRT *_gripper_val is a distance, not the legacy +/-1 action.
                # Match SEW's 5 cm pinch threshold and the absolute 0..0.8 actuator.
                if distance is not None:self.grips[side]=np.array([.8 if distance<=.05 else 0.])
            # Same absolute joint range clipping as the shared simulation mapping.
            if self.variant=='psyonic':
                gripper=self.robot.gripper[side]
                bounds=np.array([self.model.joint(j).range for j in gripper.joints])
                self.grips[side]=np.clip(self.grips[side],bounds[:,0],bounds[:,1])
            commands[side+'_gripper']=self.grips[side]
        action=self.robot.create_action_vector(commands)
        if not np.isfinite(action).all():raise ValueError('non-finite robot action')
        self.last=action.copy()
        return action
    def _mink(self,rate):
        import mujoco
        from scipy.spatial.transform import Rotation
        from dual_kinova3_teleop import ASSETS,scene_path
        from geo_kin_core.fallback import MinkFallbackSession
        path=scene_path(self.variant)
        model=mujoco.MjModel.from_xml_path(str(path));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
        shoulders=np.mean([data.body(s+'_shoulder_link').xpos for s in ['right','left']],axis=0)
        sides={};offsets={}
        with np.load(ASSETS/'specs/dual_kinova3_v1.npz') as spec:
            for side in ['right','left']:
                sides[side]={'wrist_body':side+'_end_effector','joints':[f'{side}_Actuator{i}' for i in range(1,8)]}
                if self.variant=='psyonic':sides[side]['palm_body']=side+'_gripper_index_L1'
                legacy=Rotation.from_euler('yx',[np.pi/2,-np.pi if side=='right' else 0]).as_matrix() if self.variant=='psyonic' else Rotation.from_euler('ZYX',[np.pi/2,0,np.pi/2]).as_matrix()
                offsets[side]=legacy@Rotation.from_euler('x',np.pi).as_matrix().T@spec[side+'_arm/R'][7]
        return MinkFallbackSession(path,sides,torso=False,wrist_target_mode='palm',
            wrist_rotation_offsets=offsets,preprocess={'mocap_offset':tuple(shoulders)},
            max_iters=5,ik_dt=1/rate,enable_joint_limits=True)
