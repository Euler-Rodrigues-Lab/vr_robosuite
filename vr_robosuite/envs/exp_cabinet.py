from collections import OrderedDict

import numpy as np
import robosuite as suite
import robosuite.utils.transform_utils as T
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
# from robosuite.models.arenas import TableArena
from robosuite.models.objects import BoxObject, PotWithHandlesObject #, BottleObject
# from vr_robosuite.objects import BigCubeObject
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import UniformRandomSampler
from robosuite.utils.placement_samplers import SequentialCompositeSampler
from robosuite.utils.transform_utils import convert_quat

from vr_robosuite.objects import CabinetArena, VisualGoalZone
from vr_robosuite.contact_logger import ContactLogger, setup_collision_tracking

import mujoco
import time
import h5py
import os
from collections import OrderedDict

class ExpCabinet(ManipulationEnv):
    def __init__(
        self,
        robots,
        env_configuration="default",
        controller_configs=None,
        gripper_types="default",
        initialization_noise="default",
        table_full_size=(1, 1, 0.05),
        table_friction=(1.0, 5e-3, 1e-4),
        use_camera_obs=True,
        use_object_obs=True,
        reward_scale=1.0,
        reward_shaping=False,
        placement_initializer=None,
        has_renderer=False,
        has_offscreen_renderer=True,
        render_camera="agentview",
        render_collision_mesh=False,
        render_visual_mesh=True,
        render_gpu_device_id=-1,
        control_freq=20,
        lite_physics=True,
        horizon=1000,
        ignore_done=False,
        hard_reset=True,
        camera_names="agentview",
        camera_heights=256,
        camera_widths=256,
        camera_depths=False,
        camera_segmentations=None,
        log_data=True, # for creating HDF5 log files
        log_filename=None,
    ):
        # settings for table top
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.table_offset = np.array((0, 0, 0.4))  # made changes

        # Omron LD-60 Mobile Base setting
        self.init_torso_height = 0.342

        # reward configuration
        self.reward_scale = reward_scale
        self.reward_shaping = reward_shaping

        # whether to use ground-truth object states
        self.use_object_obs = use_object_obs

        # object placement initializer
        self.placement_initializer = placement_initializer
        self.mujoco_arena = None  # for logging shelf position

        # HDF5 Log File initializer

        # Generate filename if not provided
        if log_filename is None:
            import datetime
            current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_filename = f"explog_cabinet{current_time}.h5"

        self.log_filename = log_filename or self.log_filename
        self.log_data = log_data
        self.contact_logger = ContactLogger(
            log_filename=self.log_filename,
            control_freq=control_freq
        )

        # goal achieved check related variables
        self.goal_achieved = False
        self.step_count = 0
        self.completion_time = 0

        super().__init__(
            robots=robots,
            env_configuration=env_configuration,
            controller_configs=controller_configs,
            gripper_types=gripper_types,
            initialization_noise=initialization_noise,
            use_camera_obs=use_camera_obs,
            has_renderer=has_renderer,
            has_offscreen_renderer=has_offscreen_renderer,
            render_camera=render_camera,
            render_collision_mesh=render_collision_mesh,
            render_visual_mesh=render_visual_mesh,
            render_gpu_device_id=render_gpu_device_id,
            control_freq=control_freq,
            lite_physics=lite_physics,
            horizon=horizon,
            ignore_done=ignore_done,
            hard_reset=hard_reset,
            camera_names=camera_names,
            camera_heights=camera_heights,
            camera_widths=camera_widths,
            camera_depths=camera_depths,
            camera_segmentations=camera_segmentations,
        )

    def _load_model(self):
        """
        Loads an xml model, puts it in self.model
        """
        super()._load_model()

        # Adjust base pose accordingly
        xpos = self.robots[0].robot_model.base_xpos_offset["table"](self.table_full_size[0])
        self.robots[0].robot_model.set_base_xpos(xpos)

        # load model for table top workspace
        mujoco_arena = CabinetArena(
            table_full_size=self.table_full_size,
            table_friction=self.table_friction,
            table_offset=self.table_offset,
        )

        # Arena always gets set to zero origin
        mujoco_arena.set_origin([0, 0, 0])

        self.mujoco_arena = mujoco_arena  # store for later

        # self.cube1 = BigCubeObject(name="cube1")
        # self.cube2 = BigCubeObject(name="cube2")

        self.box = BoxObject(size=[0.025,0.025,0.025], name="Box")
        self.visual_goal = VisualGoalZone(name="vis_goal")

        self.objs = [self.box]
        self.objs_cls = [BoxObject]
        self.objs_xpos = [[-0.1, -0.1]]
        self.objs_ypos = [[0.0, 0.0]]
        self.objs_rots = [[np.pi/2, np.pi/2]]
        self.objs_zpos = [0.22]
        # for goal testing only
        # goal_pos = self.mujoco_arena.cabinet_bottom_pos - [0, 0, 0.3]
        # self.objs_xpos = [[goal_pos[0], goal_pos[0]]]
        # self.objs_ypos = [[goal_pos[1], goal_pos[1]]]
        # self.objs_rots = [[0, 0]]
        # self.objs_zpos = [goal_pos[2]]

        # Create placement initializer
        if self.placement_initializer is None:
            self.placement_initializer = SequentialCompositeSampler(name="ObjectSampler")

            for obj, x_val, y_val, rot_val, z_val in zip(self.objs, self.objs_xpos, self.objs_ypos, self.objs_rots, self.objs_zpos):
                self.placement_initializer.append_sampler(sampler=UniformRandomSampler(
                    name=f"{obj.name}Sampler",
                    x_range=x_val,
                    y_range=y_val,
                    rotation=rot_val,
                    rotation_axis="z",
                    ensure_object_boundary_in_range=False,
                    ensure_valid_placement= True, # you can turn off for closeness but should be on if object ranges overlap
                    reference_pos=self.table_offset,
                    z_offset=z_val,
                ))

        # Reset sampler before adding any new samplers / objects
        self.placement_initializer.reset()

        for i, (obj_cls, obj_name) in enumerate(
            zip(
                self.objs_cls,
                [obj.name for obj in self.objs]
            )
        ):
            obj = self.objs[i]
            # Add this object to the placement initializer
            if isinstance(self.placement_initializer, SequentialCompositeSampler):
                # assumes we have two samplers so we add objs to them
                self.placement_initializer.add_objects_to_sampler(sampler_name=f"{obj_name}Sampler", mujoco_objects=obj)
            else:
                # This is assumed to be a flat sampler, so we just add all objs to this sampler
                self.placement_initializer.add_objects(obj)

        # task includes arena, robot, and objects of interest
        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=self.objs + [self.visual_goal],
        )

    def _setup_references(self):
        """
        Sets up references to important components
        """
        super()._setup_references()

        # Additional object references from this env

        # two handle box
        self.box_body_id = self.sim.model.body_name2id(self.box.root_body)

        # goal zone visual
        self.visual_box_body_id = self.sim.model.body_name2id(self.visual_goal.root_body)

        # table
        self.table_top_id = self.sim.model.site_name2id("table_top")

        # cabinet
        self.cabinet_id = self.sim.model.body_name2id("cabinet")
        self.cabinet_door_id = self.sim.model.body_name2id("door")
        self.cabinet_handle_id = self.sim.model.body_name2id("latch")

        # for collision detection
        environment_geoms = ['table_collision', 'l_frame', 'r_frame', 'back_frame', 'bot_frame', 'top_frame']
        self.collision_to_track_ids, self.robot_geom_ids, self.robot_geom_names = setup_collision_tracking(
            self.sim, self.robots, environment_geoms
        )

    def _reset_internal(self):
        """
        Resets simulation internal configurations.
        """
        super()._reset_internal()

        self.goal_achieved = False
        self.step_count = 0
        self.completion_time = 0

        # Reset all object positions using initializer sampler if we're not directly loading from an xml
        if not self.deterministic_reset:

            # Sample from the placement initializer for all objects
            object_placements = self.placement_initializer.sample()

            for obj_pos, obj_quat, obj in object_placements.values():
                if obj.name == "Door": # need to make a more general way to handle this
                    door_body_id = self.sim.model.body_name2id(self.door.root_body)
                    self.sim.model.body_pos[door_body_id] = obj_pos
                    self.sim.model.body_quat[door_body_id] = obj_quat
                else:
                    self.sim.data.set_joint_qpos(obj.joints[0], np.concatenate([np.array(obj_pos), np.array(obj_quat)]))

            # for obj_pos, obj_quat, obj in object_placements.values():
            #     self.sim.data.set_joint_qpos(obj.joints[0], np.concatenate([np.array(obj_pos), np.array(obj_quat)]))

            # Position goal in cabinet (fixed position)
            cabinet_bottom_pos = self.mujoco_arena.cabinet_bottom_pos
            visual_goal_body_id = self.sim.model.body_name2id(self.visual_goal.root_body)
            self.sim.model.body_pos[visual_goal_body_id] = cabinet_bottom_pos + [0, 0, 0.2]

    def step(self, action):
        """
        Step the environment forward by one timestep, with added logging.
        Args:
            action (np.array): Action to execute within the environment
        """
        # goal detection checking
        box_pos = self.sim.data.get_body_xpos("Box_main")
        vis_goal_pos = self.sim.data.get_body_xpos("vis_goal_main")
        current_time = self.step_count * (1.0 / self.control_freq)

        touching_cabinet = False
        robot_grasping = False

        # Check if the box is being grasped or sitting on the table for goal
        for i in range(self.sim.data.ncon):
            contact = self.sim.data.contact[i]
            geom1_name = self.sim.model.geom_id2name(contact.geom1)
            geom2_name = self.sim.model.geom_id2name(contact.geom2)
            if geom1_name is None or geom2_name is None:
                continue

            if geom1_name is None or geom2_name is None:
                continue

            # bot_frame is bottom of cabinet
            if ("Box" in geom1_name and "bot_frame" in geom2_name) or ("Box" in geom2_name and "bot_frame" in geom1_name):
                touching_cabinet = True
            if ("Box" in geom1_name and "gripper" in geom2_name) or ("Box" in geom2_name and "gripper" in geom1_name):
                robot_grasping = True

        if np.linalg.norm(box_pos - vis_goal_pos) < 0.40 and not self.goal_achieved and touching_cabinet and not robot_grasping:
            # need to add check for closing the door
            print(f"Goal achieved at time {current_time:.2f}!")
            print("Object position: ", box_pos, "Goal position: ", vis_goal_pos)
            self.goal_achieved = True
            self.contact_logger.goal_set(self.goal_achieved, current_time)
            self.completion_time = current_time
        if self.goal_achieved and not (np.linalg.norm(box_pos - vis_goal_pos) < 0.40):
            print(f"Object moved out of goal position at time {current_time:.2f}!")
            self.goal_achieved = False
            self.contact_logger.goal_set(self.goal_achieved, current_time)

        if self.log_data:
            self.contact_logger.collect_step_data(
                self.sim,
                self.collision_to_track_ids,
                self.robot_geom_ids,
                self.robot_geom_names,
                print_contacts=True
            )

        self.step_count += 1

        task_success = self.goal_achieved and (current_time-self.completion_time>2)

        obs, reward, done, info = super().step(action)

        if task_success:
            done = True
        info["success"] = bool(task_success)

        return obs, reward, done, info

    def reward(self, action):
        """
        Placeholder reward function
        Args:
            action (np.array): Action to execute within the environment
        Returns:
            float: Reward from environment
        """
        # For now, return a constant reward of 0 since we're just observing
        return 0.0

    def close(self):
        if self.log_data:
            self.contact_logger.finalize()
        super().close()
