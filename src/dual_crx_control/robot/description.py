"""Build driver descriptions with the selected real or mock hardware plugin."""
import xml.etree.ElementTree as ET
import xacro

from .initial_pose import load_initial_radians


def arm_description(xacro_path, side, robot_ip, mock, read_only=False):
    description = xacro.process_file(xacro_path, mappings={
        'robot_ip': robot_ip, 'use_mock': str(mock).lower(),
        'prefix': f'{side}_', 'child_link': f'{side}_ee_mount',
        'motion_control': '0' if read_only else '1',
    }).toxml()
    if not mock:
        return description
    # The driver's mock macro does not expose initial-position arguments.
    # Add ros2_control's standard initial_value parameter to the expanded XML.
    root = ET.fromstring(description)
    for index, position in enumerate(load_initial_radians()[side], start=1):
        interface = root.find(
            f"ros2_control/joint[@name='{side}_J{index}']/state_interface[@name='position']")
        if interface is None:
            raise ValueError(f'Missing mock position interface for {side}_J{index}')
        initial = interface.find("param[@name='initial_value']")
        if initial is None:
            initial = ET.SubElement(interface, 'param', name='initial_value')
        initial.text = str(position)
    return ET.tostring(root, encoding='unicode')
