import numpy as np


from robosuite.models.objects import MujocoXMLObject


from robosuite.utils.mjcf_utils import array_to_string, find_elements, xml_path_completion


from robosuite.models.objects import PotWithHandlesObject


from robosuite.models.objects import CompositeObject


from robosuite.utils.mjcf_utils import BLUE, GREEN, RED, CustomMaterial, add_to_dict, array_to_string


import robosuite.utils.transform_utils as T


from pathlib import Path


_HERE = Path(__file__).parent


_TO_OBJ = _HERE / "assets" / "objects"


class VisualGoalZone(MujocoXMLObject):
    """
    Visual only green translucent goalzone. No collision detection.
    """

    def __init__(self, name):
        xml_path = _TO_OBJ / "visual_goalzone.xml"
        super().__init__(
            xml_path,
            name=name,
            joints=None,  # No joints - static object
            obj_type="visual",
            duplicate_collision_geoms=False
        )

    @property
    def important_sites(self):
        """
        Returns:
            dict: In addition to any default sites for this object, also provides the following entries

                :`'center'`: Name of cube center site
                :`'top'`: Name of cube top site
                :`'front_face'`: Name of cube front face site
                :`'left_face'`: Name of cube left face site
                :`'right_face'`: Name of cube right face site
        """
        # Get dict from super call and add to it
        dic = super().important_sites
        dic.update({
            "center": self.naming_prefix + "center",
            "top": self.naming_prefix + "top",
            "front_face": self.naming_prefix + "front_face",
            "left_face": self.naming_prefix + "left_face",
            "right_face": self.naming_prefix + "right_face"
        })
        return dic


class WineGlassObject(MujocoXMLObject):
    """
    Wine glass object

    Args:
        name (str): Name of this object instance
    """

    def __init__(self, name, friction=None, mass=None):
        xml_path = _TO_OBJ / "wine_glass.xml"
        super().__init__(
            xml_path,
            name=name,
            joints=[dict(type="free", damping="0.0005")],
            obj_type="all",
            duplicate_collision_geoms=True,
        )


class ShelfObject(MujocoXMLObject):
    """
    Shelf object

    Args:
        name (str): Name of this object instance
    """

    def __init__(self, name):
        xml_path = _TO_OBJ / "shelf.xml"
        super().__init__(
            xml_path,
            name=name,
            joints=None,
            obj_type="all",
            duplicate_collision_geoms=True,
        )

    @property
    def important_sites(self):
        """
        Returns:
            dict: Important sites for the shelf

                :`'top_site'`: Name of shelf top site
                :`'bottom_site'`: Name of shelf bottom site
        """
        return {
            "top": self.naming_prefix + "top_site",
            "bottom": self.naming_prefix + "bottom_site"
        }
