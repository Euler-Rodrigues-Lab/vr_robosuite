import h5py
import numpy as np
import mujoco
import os
import datetime
from typing import List, Optional, Dict, Any


class ContactLogger:
    """
    A reusable contact logger for experiment environments.

    This class handles:
    - Buffer management for contact data
    - HDF5 file creation and writing
    - Contact detection and force calculation
    - Automatic file flushing every 1000 entries
    """

    def __init__(
        self,
        log_filename: Optional[str] = None,
        data_dir: Optional[str] = None,
        control_freq: float = 20.0,
        buffer_size: int = 10
    ):
        """
        Initialize the ContactLogger.

        Args:
            log_filename: Custom filename for the log file (auto-generated if None)
            data_dir: Directory to save log files (defaults to ../data relative to caller)
            control_freq: Control frequency for time calculation
            buffer_size: Number of entries before flushing to file
        """
        self.control_freq = control_freq
        self.buffer_size = buffer_size
        self.step_count = 0

        # Initialize data buffer
        self.data_buffer = {
            'time': [],
            'contacts': [],
            'contact_geom1': [],
            'contact_geom2': [],
            'contact_position': [],
            'contact_force': []
        }

        # Setup data directory
        if data_dir is None:
            # Default to ../data relative to the calling file
            data_dir = os.environ.get("VR_ROBOSUITE_CONTACT_DIR", "recordings/contacts")
        os.makedirs(data_dir, exist_ok=True)

        # Generate filename if not provided
        if log_filename is None:
            current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_filename = f"contact_log_{current_time}.h5"

        self.log_filepath = os.path.join(data_dir, log_filename)

        self.goal_achieved = False
        self.goal_time = 0
        print("[DEBUG] Contact logger created!")

    def collect_step_data(
        self,
        sim,
        collision_to_track_ids: List[int],
        robot_geom_ids: List[int],
        robot_geom_names: List[str],
        print_contacts: bool = False
    ) -> None:
        """
        Collect contact data for the current simulation step.

        Args:
            sim: MuJoCo simulation object
            collision_to_track_ids: List of geometry IDs to track for collisions
            robot_geom_ids: List of robot geometry IDs
            robot_geom_names: List of robot geometry names (must match robot_geom_ids order)
            print_contacts: Whether to print contact events to console
        """
        current_time = self.step_count * (1.0 / self.control_freq)
        contact_geom1 = "None"
        contact_geom2 = "None"
        contact_position = "None"
        contact_force = 0.0
        obj_contact = False
        # print("[DEBUG] Contact logger collecting data!")

        # Check all active contacts
        for i in range(sim.data.ncon):
            contact = sim.data.contact[i]
            geom1_name = sim.model.geom_id2name(contact.geom1)
            geom2_name = sim.model.geom_id2name(contact.geom2)

            # Check if contact involves tracked objects and robot parts
            robot_geom = None
            if contact.geom1 in collision_to_track_ids and contact.geom2 in robot_geom_ids:
                obj_contact = True
                robot_geom_idx = robot_geom_ids.index(contact.geom2)
                robot_geom = robot_geom_names[robot_geom_idx]
                if print_contacts:
                    print(f"Time: {current_time:.3f}; Contact between {geom1_name} and {geom2_name}")
            elif contact.geom2 in collision_to_track_ids and contact.geom1 in robot_geom_ids:
                obj_contact = True
                robot_geom_idx = robot_geom_ids.index(contact.geom1)
                robot_geom = robot_geom_names[robot_geom_idx]
                if print_contacts:
                    print(f"Time: {current_time:.3f}; Contact between {geom1_name} and {geom2_name}")
            elif contact.geom1 in robot_geom_ids and contact.geom2 in robot_geom_ids:
                if not 'gripper' in geom1_name and 'gripper' in geom2_name:
                    obj_contact = True
                    robot_geom_idx = robot_geom_ids.index(contact.geom1)
                    robot_geom = robot_geom_names[robot_geom_idx]
                    if print_contacts:
                        print(f"Time: {current_time:.3f}; Contact between {geom1_name} and {geom2_name}")

            if robot_geom:
                # Calculate contact force
                force_vector = np.zeros(6)
                mujoco.mj_contactForce(sim.model._model, sim.data._data, i, force_vector)
                force = np.linalg.norm(force_vector[:3])

                # Store individual contact components
                contact_geom1 = sim.model.geom_id2name(contact.geom1)
                contact_geom2 = sim.model.geom_id2name(contact.geom2)
                contact_position = f"[{contact.pos[0]:.6f}, {contact.pos[1]:.6f}, {contact.pos[2]:.6f}]"
                contact_force = float(force)
                break  # Take first contact for now

        # Append data to buffer
        self.data_buffer['time'].append(current_time)
        self.data_buffer['contacts'].append(obj_contact)
        self.data_buffer['contact_geom1'].append(contact_geom1)
        self.data_buffer['contact_geom2'].append(contact_geom2)
        self.data_buffer['contact_position'].append(contact_position)
        self.data_buffer['contact_force'].append(contact_force)

        self.step_count += 1

        # Flush buffer if it reaches the specified size
        if len(self.data_buffer['time']) % self.buffer_size == 0:
            self.write_buffer_to_file()

    def write_buffer_to_file(self) -> None:
        """
        Write the current data buffer to the HDF5 file.
        """
        if len(self.data_buffer['time']) == 0:
            return

        # print("[DEBUG] Contact logger writing data!")

        # Use 'a' mode for append, 'w' only for first write
        file_exists = os.path.exists(self.log_filepath)
        mode = 'a' if file_exists else 'w'

        with h5py.File(self.log_filepath, mode) as f:
            if not file_exists or 'time' not in f:
                # Create initial datasets
                dt = h5py.string_dtype(encoding='utf-8')
                f.create_dataset('time', data=np.array(self.data_buffer['time']),
                                chunks=True, maxshape=(None,), dtype='float')
                f.create_dataset('contacts', data=np.array(self.data_buffer['contacts']),
                                chunks=True, maxshape=(None,), dtype='bool')
                f.create_dataset('contact_geom1', data=np.array(self.data_buffer['contact_geom1'], dtype=dt),
                                chunks=True, maxshape=(None,), dtype=dt)
                f.create_dataset('contact_geom2', data=np.array(self.data_buffer['contact_geom2'], dtype=dt),
                                chunks=True, maxshape=(None,), dtype=dt)
                f.create_dataset('contact_position', data=np.array(self.data_buffer['contact_position'], dtype=dt),
                                chunks=True, maxshape=(None,), dtype=dt)
                f.create_dataset('contact_force', data=np.array(self.data_buffer['contact_force']),
                                chunks=True, maxshape=(None,), dtype='float')
                f.create_dataset('goal_achieved', data=np.array([self.goal_achieved]),
                                maxshape=(None,), dtype='bool')
                f.create_dataset('goal_time', data=np.array([self.goal_time]),
                                chunks=True, maxshape=(None,), dtype='float')
            else:
                # Append to existing datasets
                datasets = ['time', 'contacts', 'contact_geom1', 'contact_geom2', 'contact_position', 'contact_force']

                f['goal_achieved'][0] = np.array(self.goal_achieved)
                f['goal_time'][0] = np.array(self.goal_time)

                for dataset_name in datasets:
                    dset = f[dataset_name]
                    old_size = dset.shape[0]
                    new_size = old_size + len(self.data_buffer[dataset_name])
                    dset.resize((new_size,))

                    if dataset_name in ['contact_geom1', 'contact_geom2', 'contact_position']:
                        # Handle string encoding properly
                        dt = h5py.string_dtype(encoding='utf-8')
                        dset[old_size:] = np.array(self.data_buffer[dataset_name], dtype=dt)
                    else:
                        dset[old_size:] = np.array(self.data_buffer[dataset_name])

        # Clear buffer
        for key in self.data_buffer:
            self.data_buffer[key] = []

    def finalize(self) -> None:
        """
        Finalize logging by writing any remaining buffered data.
        Call this when the environment is closed.
        """
        self.write_buffer_to_file()

    def get_log_filepath(self) -> Optional[str]:
        """
        Get the path to the log file.

        Returns:
            Path to the log file if logging is enabled, None otherwise
        """
        return self.log_filepath

    def get_step_count(self) -> int:
        """
        Get the current step count.

        Returns:
            Current step count
        """
        return self.step_count

    def reset_step_count(self) -> None:
        """
        Reset the step count to 0.
        Useful when resetting the environment.
        """
        self.step_count = 0

    def goal_check(self, obj_pos, goal_pos) -> None:
        """
        Check and print if the object is within the goal threshold.
        """
        current_time = self.step_count * (1.0 / self.control_freq)
        if np.linalg.norm(obj_pos - goal_pos) < 0.2 and not self.goal_achieved:
            print(f"Goal achieved at time {current_time:.2f}!")
            print("Object position: ", obj_pos, "Goal position: ", goal_pos)
            self.goal_achieved = True
            self.goal_time = current_time
        if self.goal_achieved and not (np.linalg.norm(obj_pos - goal_pos) < 0.2):
            print(f"Object moved out of goal position at time {current_time:.2f}!")
            self.goal_achieved = False

    def goal_set(self, goal_bool, goal_time=0.0) -> None:
        """
        Manually set the goal for logging (for pick place due to different goal behavior)

        :param goal_bool: bool for whether the goal is achieved or not
        :param goal_time: float for time goal is achieved
        """
        self.goal_achieved = goal_bool
        self.goal_time = goal_time

def setup_collision_tracking(
    sim,
    robots: List,
    environment_geom_names: List[str]
) -> tuple[List[int], List[int], List[str]]:
    """
    Helper function to set up collision tracking for common robot configurations.

    Args:
        sim: MuJoCo simulation object
        robots: List of robot objects
        environment_geom_names: List of environment geometry names to track

    Returns:
        Tuple of (collision_to_track_ids, robot_geom_ids, robot_geom_names)
    """
    # Get environment geometry IDs
    collision_to_track_ids = []
    for geom_name in environment_geom_names:
        try:
            geom_id = sim.model.geom_name2id(geom_name)
            collision_to_track_ids.append(geom_id)
        except:
            print(f"Warning: Could not find geometry '{geom_name}'")

    # Get robot geometry IDs and names
    robot_geom_ids = []
    robot_geom_names = []

    for robot in robots:
        # Get robot body geoms
        for geom_name in robot.robot_model.contact_geoms:
            try:
                geom_id = sim.model.geom_name2id(geom_name)
                robot_geom_ids.append(geom_id)
                robot_geom_names.append(geom_name)
            except:
                continue

        # Get gripper geoms
        if hasattr(robot, 'gripper') and isinstance(robot.gripper, dict):
            # Multi-arm robot (e.g., dual arm)
            for arm_name, gripper in robot.gripper.items():
                for geom_name in gripper.contact_geoms:
                    try:
                        geom_id = sim.model.geom_name2id(geom_name)
                        robot_geom_ids.append(geom_id)
                        robot_geom_names.append(geom_name)
                    except:
                        continue
        elif hasattr(robot, 'gripper'):
            # Single-arm robot
            for geom_name in robot.gripper.contact_geoms:
                try:
                    geom_id = sim.model.geom_name2id(geom_name)
                    robot_geom_ids.append(geom_id)
                    robot_geom_names.append(geom_name)
                except:
                    continue

    return collision_to_track_ids, robot_geom_ids, robot_geom_names
