import re
import threading

from .ros_compatibility import CompatibleRosRunner, get_ros_info

def create_css_id(name: str) -> str:
    """
    Clean and convert a string to a valid CSS identifier.
    identifiers must contain only letters, numbers, underscores, or hyphens, and must not begin with a number.
    """
    css_id = re.sub(r'[^a-zA-Z0-9_-]', '_', name)
    css_id = css_id.lower()

    if css_id and css_id[0].isdigit():
        css_id = "_" + css_id 

    return css_id 


# Use the compatible ROS runner
RosRunner = CompatibleRosRunner