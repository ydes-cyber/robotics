"""Local joint transforms and root-to-link forward kinematics."""
from transforms import (axis_angle, compose, flatten_column_major, identity,
                        matrix_to_quaternion, rpy_rotation, translation)


def joint_transform(joint, position=0.0):
    origin = compose(translation(joint.xyz), rpy_rotation(joint.rpy))
    if joint.kind in ("revolute", "continuous"):
        motion = axis_angle(joint.axis, position)
    elif joint.kind == "prismatic":
        motion = translation(tuple(position * value for value in joint.axis))
    else:
        motion = identity()
    return compose(origin, motion)


def local_transforms(robot, positions):
    return [(joint, joint_transform(joint, positions.get(joint.name, 0.0)))
            for joint in robot.joints]


def world_transforms(robot, positions, global_root=None):
    result = {robot.root: global_root if global_root is not None else identity()}
    stack = [robot.root]
    while stack:
        parent = stack.pop()
        for joint in robot.children[parent]:
            result[joint.child] = compose(result[parent],
                                          joint_transform(joint, positions.get(joint.name, 0.0)))
            stack.append(joint.child)
    return result


def tf_message(robot, positions, stamp):
    entries = []
    for joint, matrix in local_transforms(robot, positions):
        x, y, z, w = matrix_to_quaternion(matrix)
        entries.append({
            "header": {"stamp": stamp, "frame_id": joint.parent},
            "child_frame_id": joint.child,
            "transform": {
                "translation": {"x": matrix[0][3], "y": matrix[1][3], "z": matrix[2][3]},
                "rotation": {"x": x, "y": y, "z": z, "w": w},
            },
        })
    return {"transforms": entries}


def matrix_message(transforms, stamp):
    return {"transforms": [
        {"header": {"stamp": stamp, "frame_id": "global_frame"},
         "child_frame_id": name, "matrix": flatten_column_major(matrix)}
        for name, matrix in transforms.items()
    ]}
