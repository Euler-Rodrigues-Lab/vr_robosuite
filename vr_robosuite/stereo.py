"""Render synchronized left/right images from one simulation state."""
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from .factory import native_sim


class StereoRenderer:
    def __init__(self,env,width=640,height=480,ipd=.065,camera='robot0_robotview',fovy=75):
        if not 0 < ipd < .2:raise ValueError('IPD must be between 0 and 0.2 metres')
        self.model,self.data=native_sim(env)
        self.model.vis.global_.offwidth=max(self.model.vis.global_.offwidth,width)
        self.model.vis.global_.offheight=max(self.model.vis.global_.offheight,height)
        self.cam=self.model.camera(camera).id
        self.model.cam_fovy[self.cam]=fovy
        self.ipd=ipd
        self.renderer=mujoco.Renderer(self.model,height=height,width=width)
        self.opt=mujoco.MjvOption();self.opt.geomgroup[0]=0
        self.base=self.model.body(env.robots[0].robot_model.naming_prefix+'base').id
        self.reference_rotation=self.data.cam_xmat[self.cam].reshape(3,3).copy()
        self.head_offset=Rotation.from_euler('ZYX',[-90,0,65],degrees=True).as_matrix()
    def track_head(self,frame):
        if frame is None or frame.head_rotation is None:return
        # Source experiment mapping: yaw/pitch, suppress roll, fixed neck origin.
        euler=Rotation.from_matrix(frame.head_rotation).as_euler('ZYX');euler[2]=0
        base_r=self.data.xmat[self.base].reshape(3,3)
        world_r=base_r@Rotation.from_euler('ZYX',euler).as_matrix()@self.head_offset
        world_p=self.data.xpos[self.base]+base_r@np.array([.1,0,.55])
        parent=self.model.cam_bodyid[self.cam];r=self.data.xmat[parent].reshape(3,3)
        self.model.cam_pos[self.cam]=r.T@(world_p-self.data.xpos[parent])
        q=Rotation.from_matrix(r.T@world_r).as_quat()
        self.model.cam_quat[self.cam]=q[[3,0,1,2]]
        mujoco.mj_forward(self.model,self.data)
    def eyes(self):
        # Offsets in camera-parent coordinates, along camera +x (image right).
        position=self.model.cam_pos[self.cam].copy()
        q=self.model.cam_quat[self.cam]
        axis=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()[:,0]
        frames=[]
        try:
            for sign in [-1,1]:
                self.model.cam_pos[self.cam]=position+sign*self.ipd/2*axis
                mujoco.mj_forward(self.model,self.data)
                self.renderer.update_scene(self.data,camera=self.cam,scene_option=self.opt)
                frames.append(self.renderer.render().copy())
        finally:
            self.model.cam_pos[self.cam]=position
            mujoco.mj_forward(self.model,self.data)
        return tuple(frames)
    def render(self):return np.concatenate(self.eyes(),axis=1)
    def close(self):self.renderer.close()
