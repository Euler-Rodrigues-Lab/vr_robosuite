import numpy as np


from robosuite.models.arenas import Arena


from robosuite.utils.mjcf_utils import array_to_string, string_to_array, find_elements


from pathlib import Path


_HERE = Path(__file__).parent


_TO_ARENA = _HERE / "assets" / "arenas"


class CabinetArena(Arena):
    """
    Workspace that contains a table and a cabinet.

    Args:
        table_full_size (3-tuple): (L,W,H) full dimensions of the table
        table_friction (3-tuple): (sliding, torsional, rolling) friction parameters of the table
        table_offset (3-tuple): (x,y,z) offset from center of arena when placing table.
            Note that the z value sets the upper limit of the table
        has_legs (bool): whether the table has legs or not
        xml (str): xml file to load arena
    """

    def __init__(
        self,
        table_full_size=(0.8, 0.8, 0.05),
        table_friction=(1, 0.005, 0.0001),
        table_offset=(0, 0, 0.8),
        has_legs=True,
        lock=True,
        friction=None,
        damping=None,
        xml="cabinet_arena.xml",
    ):
        xml_path = _TO_ARENA / xml
        super().__init__(xml_path)

        self.table_full_size = np.array(table_full_size)
        self.table_half_size = self.table_full_size / 2
        self.table_friction = table_friction
        self.table_offset = table_offset
        self.center_pos = self.bottom_pos + np.array([0, 0, -self.table_half_size[2]]) + self.table_offset

        self.table_body = self.worldbody.find("./body[@name='table']")
        self.table_collision = self.table_body.find("./geom[@name='table_collision']")
        self.table_visual = self.table_body.find("./geom[@name='table_visual']")
        self.table_top = self.table_body.find("./site[@name='table_top']")

        self.has_legs = has_legs
        self.table_legs_visual = [
            self.table_body.find("./geom[@name='table_leg1_visual']"),
            self.table_body.find("./geom[@name='table_leg2_visual']"),
            self.table_body.find("./geom[@name='table_leg3_visual']"),
            self.table_body.find("./geom[@name='table_leg4_visual']"),
        ]

        self.lock = lock
        self.friction = friction
        self.damping = damping
        if self.friction is not None:
            self._set_door_friction(self.friction)
        if self.damping is not None:
            self._set_door_damping(self.damping)

        self.configure_location()

    def configure_location(self):
        """Configures correct locations for this arena"""
        self.floor.set("pos", array_to_string(self.bottom_pos))

        self.table_body.set("pos", array_to_string(self.center_pos))
        self.table_collision.set("size", array_to_string(self.table_half_size))
        self.table_collision.set("friction", array_to_string(self.table_friction))
        self.table_visual.set("size", array_to_string(self.table_half_size))

        self.table_top.set("pos", array_to_string(np.array([0, 0, self.table_half_size[2]])))

        # If we're not using legs, set their size to 0
        if not self.has_legs:
            for leg in self.table_legs_visual:
                leg.set("rgba", array_to_string([1, 0, 0, 0]))
                leg.set("size", array_to_string([0.0001, 0.0001]))
        else:
            # Otherwise, set leg locations appropriately
            delta_x = [0.1, -0.1, -0.1, 0.1]
            delta_y = [0.1, 0.1, -0.1, -0.1]
            for leg, dx, dy in zip(self.table_legs_visual, delta_x, delta_y):
                # If x-length of table is less than a certain length, place leg in the middle between ends
                # Otherwise we place it near the edge
                x = 0
                if self.table_half_size[0] > abs(dx * 2.0):
                    x += np.sign(dx) * self.table_half_size[0] - dx
                # Repeat the same process for y
                y = 0
                if self.table_half_size[1] > abs(dy * 2.0):
                    y += np.sign(dy) * self.table_half_size[1] - dy
                # Get z value
                z = (self.table_offset[2] - self.table_half_size[2]) / 2.0
                # Set leg position
                leg.set("pos", array_to_string([x, y, -z]))
                # Set leg size
                leg.set("size", array_to_string([0.025, z]))

    def _set_door_friction(self, friction):
        """
        Helper function to override the door friction directly in the XML

        Args:
            friction (3-tuple of float): friction parameters to override the ones specified in the XML
        """
        hinge = find_elements(root=self.worldbody, tags="joint", attribs={"name": self.hinge_joint}, return_first=True)
        hinge.set("frictionloss", array_to_string(np.array([friction])))

    def _set_door_damping(self, damping):
        """
        Helper function to override the door damping directly in the XML

        Args:
            damping (float): damping parameter to override the ones specified in the XML
        """
        hinge = find_elements(root=self.worldbody, tags="joint", attribs={"name": self.hinge_joint}, return_first=True)
        hinge.set("damping", array_to_string(np.array([damping])))

    @property
    def cabinet_bottom_pos(self):
        """
        Returns the bottom position of the cabinet

        Returns:
            np.array: (x,y,z) bottom position of cabinet
        """
        cabinet = find_elements(root=self.worldbody, tags="body", attribs={"name": "cabinet"}, return_first=True)
        cabinet_bottom = find_elements(root=self.worldbody, tags="site", attribs={"name": "cabinet_bottom"}, return_first=True)
        return string_to_array(self.floor.get("pos")) + string_to_array(cabinet.get("pos")) + string_to_array(cabinet_bottom.get("pos"))

    @property
    def table_top_abs(self):
        """
        Grabs the absolute position of table top

        Returns:
            np.array: (x,y,z) table position
        """
        return string_to_array(self.floor.get("pos")) + self.table_offset


class TableShadowArena(Arena):
    """
    Workspace that contains an empty table with shadow.


    Args:
        table_full_size (3-tuple): (L,W,H) full dimensions of the table
        table_friction (3-tuple): (sliding, torsional, rolling) friction parameters of the table
        table_offset (3-tuple): (x,y,z) offset from center of arena when placing table.
            Note that the z value sets the upper limit of the table
        has_legs (bool): whether the table has legs or not
        xml (str): xml file to load arena
    """

    def __init__(
        self,
        table_full_size=(0.8, 0.8, 0.05),
        table_friction=(1, 0.005, 0.0001),
        table_offset=(0, 0, 0.8),
        has_legs=True,
    ):
        xml_path = _TO_ARENA / "table_shadow_arena.xml"
        super().__init__(xml_path)

        self.table_full_size = np.array(table_full_size)
        self.table_half_size = self.table_full_size / 2
        self.table_friction = table_friction
        self.table_offset = table_offset
        self.center_pos = self.bottom_pos + np.array([0, 0, -self.table_half_size[2]]) + self.table_offset

        self.table_body = self.worldbody.find("./body[@name='table']")
        self.table_collision = self.table_body.find("./geom[@name='table_collision']")
        self.table_visual = self.table_body.find("./geom[@name='table_visual']")
        self.table_top = self.table_body.find("./site[@name='table_top']")

        self.has_legs = has_legs
        self.table_legs_visual = [
            self.table_body.find("./geom[@name='table_leg1_visual']"),
            self.table_body.find("./geom[@name='table_leg2_visual']"),
            self.table_body.find("./geom[@name='table_leg3_visual']"),
            self.table_body.find("./geom[@name='table_leg4_visual']"),
        ]

        self.configure_location()

    def configure_location(self):
        """Configures correct locations for this arena"""
        self.floor.set("pos", array_to_string(self.bottom_pos))

        self.table_body.set("pos", array_to_string(self.center_pos))
        self.table_collision.set("size", array_to_string(self.table_half_size))
        self.table_collision.set("friction", array_to_string(self.table_friction))
        self.table_visual.set("size", array_to_string(self.table_half_size))

        self.table_top.set("pos", array_to_string(np.array([0, 0, self.table_half_size[2]])))

        # If we're not using legs, set their size to 0
        if not self.has_legs:
            for leg in self.table_legs_visual:
                leg.set("rgba", array_to_string([1, 0, 0, 0]))
                leg.set("size", array_to_string([0.0001, 0.0001]))
        else:
            # Otherwise, set leg locations appropriately
            delta_x = [0.1, -0.1, -0.1, 0.1]
            delta_y = [0.1, 0.1, -0.1, -0.1]
            for leg, dx, dy in zip(self.table_legs_visual, delta_x, delta_y):
                # If x-length of table is less than a certain length, place leg in the middle between ends
                # Otherwise we place it near the edge
                x = 0
                if self.table_half_size[0] > abs(dx * 2.0):
                    x += np.sign(dx) * self.table_half_size[0] - dx
                # Repeat the same process for y
                y = 0
                if self.table_half_size[1] > abs(dy * 2.0):
                    y += np.sign(dy) * self.table_half_size[1] - dy
                # Get z value
                z = (self.table_offset[2] - self.table_half_size[2]) / 2.0
                # Set leg position
                leg.set("pos", array_to_string([x, y, -z]))
                # Set leg size
                leg.set("size", array_to_string([0.025, z]))

    @property
    def table_top_abs(self):
        """
        Grabs the absolute position of table top

        Returns:
            np.array: (x,y,z) table position
        """
        return string_to_array(self.floor.get("pos")) + self.table_offset
