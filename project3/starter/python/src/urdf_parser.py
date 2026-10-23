"""Course URDF subset, using only Python's XML parser."""
import re
import math
import xml.etree.ElementTree as ET

from urdf_model import Joint, Link, Robot

NUMBER = re.compile(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?\Z")
SPACE = re.compile(r"[ \t\r\n]+")
KINDS = {"revolute", "continuous", "prismatic", "fixed"}


def tag(element):
    return element.tag.rsplit("}", 1)[-1]


def child(element, name):
    return next((item for item in element if tag(item) == name), None)


def number(value):
    if not isinstance(value, str) or not NUMBER.fullmatch(value):
        raise ValueError(f"invalid number: {value!r}")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"non-finite number: {value!r}")
    return parsed


def vector(value):
    parts = SPACE.split(value.strip(" \t\r\n")) if isinstance(value, str) else []
    if len(parts) != 3:
        raise ValueError(f"expected three numbers: {value!r}")
    return tuple(number(item) for item in parts)


def origin(element):
    if element is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = vector(element.get("xyz")) if "xyz" in element.attrib else (0.0, 0.0, 0.0)
    rpy = vector(element.get("rpy")) if "rpy" in element.attrib else (0.0, 0.0, 0.0)
    return xyz, rpy


def visual(element):
    item = child(element, "visual")
    if item is None:
        return None
    xyz, rpy = origin(child(item, "origin"))
    geometry = child(item, "geometry")
    if geometry is None or len(geometry) == 0:
        raise ValueError("visual has no geometry")
    shape = geometry[0]
    kind = tag(shape)
    if kind == "box":
        fields = {"size": vector(shape.get("size"))}
    elif kind == "cylinder":
        fields = {"radius": number(shape.get("radius")), "length": number(shape.get("length"))}
    elif kind == "sphere":
        fields = {"radius": number(shape.get("radius"))}
    elif kind == "mesh":
        filename = shape.get("filename")
        if filename is None:
            raise ValueError("mesh has no filename")
        fields = {"filename": filename}
        if "scale" in shape.attrib:
            fields["scale"] = vector(shape.get("scale"))
    else:
        raise ValueError(f"unsupported visual geometry: {kind}")
    return {"kind": kind, "fields": fields, "xyz": xyz, "rpy": rpy}


def parse_robot_description(text):
    if not isinstance(text, str) or not text.strip():
        raise ValueError("robot_description must be nonempty XML text")
    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        raise ValueError(f"invalid XML: {exc}") from exc
    if tag(root) != "robot":
        raise ValueError("root element is not robot")
    links = {}
    joints = []
    for item in root:
        if tag(item) == "link":
            name = item.get("name")
            if name is None:
                raise ValueError("link has no name")
            if name in links:
                raise ValueError("duplicate link")
            links[name] = Link(name, visual(item))
    if not links:
        raise ValueError("robot has no links")
    names = set()
    for item in root:
        if tag(item) != "joint":
            continue
        name, kind = item.get("name"), item.get("type")
        if name is None or kind is None or kind not in KINDS:
            raise ValueError("invalid joint name or type")
        if name in names:
            raise ValueError("duplicate joint")
        names.add(name)
        parent, descendant = child(item, "parent"), child(item, "child")
        if parent is None or descendant is None:
            raise ValueError("joint missing parent or child")
        parent_name, child_name = parent.get("link"), descendant.get("link")
        if parent_name is None or child_name is None or parent_name not in links or child_name not in links:
            raise ValueError("joint references missing link")
        xyz, rpy = origin(child(item, "origin"))
        axis_element = child(item, "axis")
        axis = vector(axis_element.get("xyz")) if axis_element is not None and "xyz" in axis_element.attrib else (1.0, 0.0, 0.0)
        lower = upper = 0.0
        if kind in ("revolute", "prismatic"):
            limit = child(item, "limit")
            if limit is None:
                raise ValueError("movable joint missing limit")
            lower = number(limit.get("lower")) if "lower" in limit.attrib else 0.0
            upper = number(limit.get("upper")) if "upper" in limit.attrib else 0.0
        joints.append(Joint(name, kind, parent_name, child_name, xyz, rpy, axis, lower, upper))
    roots = set(links) - {joint.child for joint in joints}
    if len(roots) != 1:
        raise ValueError("robot must have exactly one root")
    robot = Robot(root.get("name", "robot"), links, tuple(joints), roots.pop())
    visited = set()
    stack = [robot.root]
    while stack:
        link = stack.pop()
        if link in visited:
            raise ValueError("robot has a cycle or multiple parents")
        visited.add(link)
        stack.extend(joint.child for joint in robot.children[link])
    if len(visited) != len(links):
        raise ValueError("robot is disconnected")
    return robot
