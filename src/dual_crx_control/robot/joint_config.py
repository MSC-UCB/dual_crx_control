"""Joint names and initial software-mock configuration (radians at interfaces)."""
SIDES = ('left', 'right')
JOINT_NAMES = {side: [f'{side}_J{i}' for i in range(1, 7)] for side in SIDES}
# Compatibility export for motion consumers; values come only from YAML.
from .initial_pose import load_initial_degrees

INITIAL_JOINTS_DEG = load_initial_degrees()

# The control stack is intentionally a single, fixed CRX5IA instance.  Keep
# these names absolute so motion tools work from the root ROS namespace and do
# not depend on a launch-time namespace or a caller's remapping.
TOPIC_NAMESPACE = '/crx5ia'
JOINT_TARGETS_TOPIC = f'{TOPIC_NAMESPACE}/joint_targets'
JOINT_STATES_TOPIC = f'{TOPIC_NAMESPACE}/joint_states'
INTERPOLATED_COMMANDS_TOPIC = f'{TOPIC_NAMESPACE}/interpolated_joint_commands'
ROBOT_DESCRIPTION_TOPIC = f'{TOPIC_NAMESPACE}/robot_description'
INTERPOLATION_NODE = f'{TOPIC_NAMESPACE}/joint_interpolation'


def validate_active_arms(arms):
    """Validate explicit selection and keep the established left/right ordering."""
    if (not isinstance(arms, (list, tuple)) or not arms
            or any(side not in SIDES for side in arms)
            or len(set(arms)) != len(arms)):
        raise ValueError('active_arms must contain left and/or right, without duplicates')
    return tuple(side for side in SIDES if side in arms)


def declare_active_arms(node):
    from rcl_interfaces.msg import ParameterDescriptor
    return validate_active_arms(node.declare_parameter(
        'active_arms', list(SIDES), ParameterDescriptor(read_only=True)).value)


def arm_topic(side, suffix):
    """Return an absolute topic for one fixed CRX5IA arm."""
    side = canonical_side(side)
    return f'{TOPIC_NAMESPACE}/{side}/{suffix.lstrip("/")}'


def canonical_side(namespace):
    aliases = {'robot1': 'right', 'robot2': 'left', '': 'left'}
    side = aliases.get(namespace.strip('/'), namespace.strip('/'))
    if side not in SIDES:
        raise ValueError(f'Unsupported arm namespace: {namespace}')
    return side


def ordered_feedback(message, arm):
    """Order joint feedback by arm; reject malformed names/positions."""
    if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
        raise ValueError('malformed joint feedback')
    values = dict(zip(message.name, message.position))
    return [values[f'{arm}_J{i}'] for i in range(1, 7)]
