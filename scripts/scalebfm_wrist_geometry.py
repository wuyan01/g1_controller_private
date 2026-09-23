"""Named-joint wrist FK for the ScaleBFM deployment metadata; no ROS dependency.

All quaternions use WXYZ. Results are world/odom poses of the two
``*_wrist_yaw_link`` origins, not palm centers or fingertips. This computes
geometry only and does not establish contact, reachability or actor tracking.
"""

from collections.abc import Mapping, Sequence
import math


_UNIT_TOLERANCE = 0.01  # Same validation tolerance as the C++ FK metadata/state.


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _vector(value, size, name):
    if (not isinstance(value, Sequence) or isinstance(value, (str, bytes))
            or len(value) != size):
        raise ValueError(f"{name} must have {size} components")
    return [_number(component, name) for component in value]


def _unit(value, size, name):
    value = _vector(value, size, name)
    norm = math.hypot(*value)
    if abs(norm - 1.0) >= _UNIT_TOLERANCE:
        raise ValueError(f"{name} must be unit length")
    return [component / norm for component in value]


def _multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw]


def _rotate(quaternion, vector):
    # q * [0, v] * conjugate(q), using an already validated unit quaternion.
    conjugate = [quaternion[0], *(-value for value in quaternion[1:])]
    return _multiply(_multiply(quaternion, [0.0, *vector]), conjugate)[1:]


def _names(value, name):
    if (not isinstance(value, Sequence) or isinstance(value, (str, bytes))
            or len(value) != 29
            or any(not isinstance(item, str) or not item for item in value)
            or len(set(value)) != 29):
        raise ValueError(f"{name} must contain 29 unique joint names")
    return set(value)


def wrist_poses(metadata, joint_positions, pelvis_position, pelvis_quaternion_wxyz):
    """Return ``left``/``right`` dicts with position and quaternion_wxyz lists.

    ``joint_positions`` must map exactly the metadata's 29 body-joint names to
    measured angles in radians; insertion order is irrelevant. A caller reading
    JointState must reject duplicate names before constructing this dictionary.
    Pelvis position/quaternion must describe the same odom frame as the target.
    All malformed, incomplete, nonfinite or incompatible inputs raise ValueError.
    """
    if not isinstance(metadata, Mapping):
        raise ValueError("metadata must be a mapping")
    joint_names = _names(metadata.get("joint_names"), "joint_names")
    sdk_names = _names(metadata.get("sdk_joint_names"), "sdk_joint_names")
    if sdk_names != joint_names:
        raise ValueError("actor and SDK joint names must describe the same 29 joints")
    if not isinstance(joint_positions, Mapping):
        raise ValueError("joint_positions must be a mapping of measured joint names")
    if set(joint_positions) != joint_names:
        missing = sorted(joint_names - set(joint_positions))
        extra = sorted(str(name) for name in set(joint_positions) - joint_names)
        raise ValueError(f"joint_positions must contain exactly all 29 body joints; missing={missing}, extra={extra}")
    measured = {name: _number(joint_positions[name], name) for name in joint_names}
    root_position = _vector(pelvis_position, 3, "pelvis_position")
    root_rotation = _unit(pelvis_quaternion_wxyz, 4, "pelvis_quaternion_wxyz")
    bodies = metadata.get("kinematics")
    if not isinstance(bodies, Sequence) or len(bodies) != 30:
        raise ValueError("kinematics must contain the pelvis-rooted 30-body tree")

    poses = []
    body_names = set()
    used_joints = set()
    wrists = {}
    for index, body in enumerate(bodies):
        if not isinstance(body, Mapping):
            raise ValueError("each kinematic body must be a mapping")
        name = body.get("name")
        if not isinstance(name, str) or not name or name in body_names:
            raise ValueError("kinematic body names must be nonempty and unique")
        body_names.add(name)
        parent = body.get("parent")
        joint = body.get("joint_name")
        translation = _vector(body.get("translation"), 3, f"{name}.translation")
        rotation = _unit(body.get("rotation_wxyz"), 4, f"{name}.rotation_wxyz")
        reference = _number(body.get("joint_reference"), f"{name}.joint_reference")
        if isinstance(parent, bool) or not isinstance(parent, int):
            raise ValueError(f"{name}.parent must be an integer index")
        if index == 0:
            if name != "pelvis" or parent != -1 or joint != "":
                raise ValueError("kinematic root must be pelvis with parent=-1 and no joint")
            # C++ supplies the root pose directly rather than composing this
            # transform; reject metadata that could imply a different origin.
            if (any(abs(value) > 1e-9 for value in translation)
                    or any(abs(value) > 1e-9 for value in rotation[1:])
                    or reference != 0.0
                    or any(_vector(body.get("axis"), 3, "pelvis.axis"))):
                raise ValueError("pelvis metadata must have an identity root transform")
            position, orientation = root_position, root_rotation
        else:
            if not 0 <= parent < index:
                raise ValueError(f"{name}: FK parent must precede its child")
            if not isinstance(joint, str) or joint not in joint_names or joint in used_joints:
                raise ValueError(f"{name}: FK joints must name every body joint exactly once")
            used_joints.add(joint)
            axis = _unit(body.get("axis"), 3, f"{name}.axis")
            parent_position, parent_rotation = poses[parent]
            # Match Controller::forward_kinematics exactly: the child's local
            # translation is rotated only by its parent's orientation.
            offset = _rotate(parent_rotation, translation)
            position = [origin + delta for origin, delta in zip(parent_position, offset)]
            angle = measured[joint] - reference
            if not math.isfinite(angle):
                raise ValueError(f"{joint}: angle minus reference must be finite")
            sine = math.sin(angle / 2.0)
            hinge = [math.cos(angle / 2.0), *(component * sine for component in axis)]
            orientation = _unit(_multiply(_multiply(parent_rotation, rotation), hinge),
                                4, f"{name}.world_rotation")
        if not all(math.isfinite(value) for value in position):
            raise ValueError(f"{name}: computed world position must be finite")
        poses.append((position, orientation))
        for side in ("left", "right"):
            if name == f"{side}_wrist_yaw_link":
                wrists[side] = {"position": list(position),
                                "quaternion_wxyz": list(orientation)}
    if used_joints != joint_names:
        raise ValueError("kinematics must use every measured body joint exactly once")
    if set(wrists) != {"left", "right"}:
        raise ValueError("kinematics must contain exact left/right_wrist_yaw_link names")
    return wrists
