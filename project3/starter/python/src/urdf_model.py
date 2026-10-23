"""Data model for the direct-link URDF kinematic tree."""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Link:
    name: str
    visual: object = None


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    xyz: tuple
    rpy: tuple
    axis: tuple
    lower: float = 0.0
    upper: float = 0.0


@dataclass
class Robot:
    name: str
    links: dict
    joints: tuple
    root: str
    children: dict = field(default_factory=dict)

    def __post_init__(self):
        self.children = {name: [] for name in self.links}
        for joint in self.joints:
            self.children[joint.parent].append(joint)
