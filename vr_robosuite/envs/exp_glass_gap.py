from collections import OrderedDict

import numpy as np
import robosuite as suite
import robosuite.utils.transform_utils as T
from robosuite.environments.manipulation.manipulation_env import ManipulationEnv
from robosuite.models.arenas import TableArena
from robosuite.models.objects import BoxObject
from robosuite.models.tasks import ManipulationTask
from robosuite.utils.observables import Observable, sensor
from robosuite.utils.placement_samplers import UniformRandomSampler
from robosuite.utils.placement_samplers import SequentialCompositeSampler
from robosuite.utils.transform_utils import convert_quat
from vr_robosuite.objects import WineGlassObject, ShelfObject, TableShadowArena, VisualGoalZone

import mujoco
import time
import h5py
import os
from collections import OrderedDict
from vr_robosuite.contact_logger import ContactLogger, setup_collision_tracking

class ExpGlassGap(ManipulationEnv):
    def __init__(
        self,
        robots,
        env_configuration="default",
        controller_configs=None,
        gripper_types="default",
        initialization_noise="default",
        use_latch = True,
        table_full_size=(1.2, 1.2, 0.05),
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
        goal_side="right",
    ):
        # settings for table top
        self.table_full_size = table_full_size
        self.table_friction = table_friction
        self.table_offset = np.array((0, 0, 0.6))

        # settings for door
        self.use_latch = use_latch
        self.goal_side = goal_side

        # Omron LD-60 Mobile Base setting
        self.init_torso_height = 0.342

        # reward configuration
        self.reward_scale = reward_scale
        self.reward_shaping = reward_shaping

        # whether to use ground-truth object states
        self.use_object_obs = use_object_obs

        # object placement initializer
        self.placement_initializer = placement_initializer

        # HDF5 Log File initializer
        # Generate filename if not provided
        if log_filename is None:
            import datetime
            current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_filename = f"explog_glassgap{current_time}.h5"

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
        mujoco_arena = TableShadowArena(
            table_full_size=self.table_full_size,
            table_friction=self.table_friction,
            table_offset=self.table_offset,
        )

        # Arena always gets set to zero origin
        mujoco_arena.set_origin([0, 0, 0])

        # object to grab
        self.box = BoxObject(size=[0.025,0.025,0.025], name="Box")

        # visual of object to move
        self.visual_goal_l = VisualGoalZone(name="vis_goal_l")
        self.visual_goal_r = VisualGoalZone(name="vis_goal_r")

        # shelf
        self.shelf = ShelfObject(name="Shelf")

        # first pyramid
        self.wineglass1 = WineGlassObject(name="WineGlass1")
        self.wineglass2 = WineGlassObject(name="WineGlass2")
        self.wineglass3 = WineGlassObject(name="WineGlass3")
        self.wineglass4 = WineGlassObject(name="WineGlass4")
        self.wineglass5 = WineGlassObject(name="WineGlass5")

        # second pyramid
        self.wineglass6 = WineGlassObject(name="WineGlass6")
        self.wineglass7 = WineGlassObject(name="WineGlass7")
        self.wineglass8 = WineGlassObject(name="WineGlass8")
        self.wineglass9 = WineGlassObject(name="WineGlass9")
        self.wineglass10 = WineGlassObject(name="WineGlass10")

        self.objs = [self.box,
                     self.wineglass1, self.wineglass2, self.wineglass3, self.wineglass4, self.wineglass5,
                     self.wineglass6, self.wineglass7, self.wineglass8, self.wineglass9, self.wineglass10]
        self.objs_cls = [BoxObject,
                         WineGlassObject, WineGlassObject, WineGlassObject, WineGlassObject, WineGlassObject,
                         WineGlassObject, WineGlassObject, WineGlassObject, WineGlassObject, WineGlassObject]
        # position and rotation ranges (same value means place in fixed location)
        self.objs_xpos = [[-0.1,0.05],
                          [-0.2, -0.2], [-0.1, -0.1], [0.0, 0.0], [-0.15, -0.15], [-0.05, -0.05],
                          [-0.2, -0.2], [-0.1, -0.1], [0.0, 0.0], [-0.15, -0.15], [-0.05, -0.05]]
        self.objs_ypos = [[0,0], # [0.4, 0.4], # second option is for goal checking
                          [-0.17,-0.17], [-0.17,-0.17], [-0.17,-0.17], [-0.17,-0.17], [-0.17,-0.17],
                          [0.17,0.17], [0.17,0.17], [0.17,0.17], [0.17,0.17], [0.17,0.17]]
        self.objs_rots = [[0,0],
                          [0, 0], [0, 0], [0, 0], [0, 0], [0, 0],
                          [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
        self.objs_zpos = np.array([0.1,
                          0.001, 0.001, 0.001, 0.211, 0.211,
                          0.001, 0.001, 0.001, 0.211, 0.211]) - 0.08  # adjust for Table height

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

        goals = []
        if self.goal_side == "left" or self.goal_side == "both":
            goals.append(self.visual_goal_l)
        if self.goal_side == "right" or self.goal_side == "both":
            goals.append(self.visual_goal_r)

        # task includes arena, robot, and objects of interest
        self.model = ManipulationTask(
            mujoco_arena=mujoco_arena,
            mujoco_robots=[robot.robot_model for robot in self.robots],
            mujoco_objects=self.objs + goals # + [self.shelf],
        )

    def _setup_references(self):
        """
        Sets up references to important components
        """
        super()._setup_references()

        # Additional object references from this env

        # table
        self.table_top_id = self.sim.model.site_name2id("table_top")

        # for collision detection
        environment_geoms = []

        for geom in self.sim.model.geom_names:
            if "WineGlass" in geom or "table" in geom:
                environment_geoms.append(geom)

        self.collision_to_track_ids, self.robot_geom_ids, self.robot_geom_names = setup_collision_tracking(
            self.sim, self.robots, environment_geoms
        )


    def _reset_internal(self):
        """
        Resets simulation internal configurations.
        """
        super()._reset_internal()

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
                    # print("Setting object", obj.name, "to pos", obj_pos, "quat", obj_quat)
                    self.sim.data.set_joint_qpos(obj.joints[0], np.concatenate([np.array(obj_pos), np.array(obj_quat)]))

            if self.goal_side == "left" or self.goal_side == "both":
                visual_goal_l_body_id = self.sim.model.body_name2id(self.visual_goal_l.root_body)
                self.sim.model.body_pos[visual_goal_l_body_id] = self.table_offset + [-0.1, 0.4, 0.2]
                self.sim.model.body_quat[visual_goal_l_body_id] = [1, 0, 0, 0]

            if self.goal_side == "right" or self.goal_side == "both":
                visual_goal_r_body_id = self.sim.model.body_name2id(self.visual_goal_r.root_body)
                self.sim.model.body_pos[visual_goal_r_body_id] = self.table_offset + [-0.1, -0.4, 0.2]
                self.sim.model.body_quat[visual_goal_r_body_id] = [1, 0, 0, 0]

    def step(self, action):
        """
        Step the environment forward by one timestep, with added logging.
        Args:
            action (np.array): Action to execute within the environment
        """
        ### Goal detection checking ###
        obj_pos = self.sim.data.get_body_xpos("Box_main")

        current_time = self.step_count * (1.0 / self.control_freq)
        touching_table = False
        robot_grasping = False

        # Check if the box is being grasped or sitting on the table for goal
        for i in range(self.sim.data.ncon):
            contact = self.sim.data.contact[i]
            geom1_name = self.sim.model.geom_id2name(contact.geom1)
            geom2_name = self.sim.model.geom_id2name(contact.geom2)
            if geom1_name is None or geom2_name is None:
                continue

            if ("Box" in geom1_name and "table" in geom2_name) or ("Box" in geom2_name and "table" in geom1_name):
                touching_table = True
            if ("Box" in geom1_name and "gripper" in geom2_name) or ("Box" in geom2_name and "gripper" in geom1_name):
                robot_grasping = True

        # cases for goal completion or disqualification
        goal_l_achieved = False
        goal_r_achieved = False

        if self.goal_side == "left" or self.goal_side == "both":
            vis_goal_l_pos = self.sim.data.get_body_xpos("vis_goal_l_main")
            if np.linalg.norm(obj_pos - vis_goal_l_pos) < 0.2 and not self.goal_achieved and touching_table and not robot_grasping:
                print(f"Goal achieved at time {current_time:.2f}!")
                print("Object position: ", obj_pos, "Goal position: ", vis_goal_l_pos)
                self.goal_achieved = True
                self.contact_logger.goal_set(self.goal_achieved, current_time)
                self.completion_time = current_time
            if self.goal_achieved and (np.linalg.norm(obj_pos - vis_goal_l_pos) < 0.2):
                goal_l_achieved = True

        if self.goal_side == "right" or self.goal_side == "both":
            vis_goal_r_pos = self.sim.data.get_body_xpos("vis_goal_r_main")
            if np.linalg.norm(obj_pos - vis_goal_r_pos) < 0.2 and not self.goal_achieved and touching_table and not robot_grasping:
                print(f"Goal achieved at time {current_time:.2f}!")
                print("Object position: ", obj_pos, "Goal position: ", vis_goal_r_pos)
                self.goal_achieved = True
                self.contact_logger.goal_set(self.goal_achieved, current_time)
                self.completion_time = current_time
            if self.goal_achieved and (np.linalg.norm(obj_pos - vis_goal_r_pos) < 0.2):
                goal_r_achieved = True

        if self.goal_achieved:
            still_achieved = False
            if (self.goal_side == "left" or self.goal_side == "both") and goal_l_achieved:
                still_achieved = True
            if (self.goal_side == "right" or self.goal_side == "both") and goal_r_achieved:
                still_achieved = True

            if not still_achieved:
                print(f"Object moved out of goal position at time {current_time:.2f}!")
                self.goal_achieved = False
                self.contact_logger.goal_set(self.goal_achieved, current_time)

        # send data to contact logger for writing
        if self.log_data:
            self.contact_logger.collect_step_data(
                self.sim,
                self.collision_to_track_ids,
                self.robot_geom_ids,
                self.robot_geom_names
            )

        self.step_count += 1

        task_success = self.goal_achieved and (current_time-self.completion_time>2)

        ### Regular function ###
        obs, reward, done, info = super().step(action)
        info["success"] = bool(task_success)
        return obs, reward, done or task_success, info

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
