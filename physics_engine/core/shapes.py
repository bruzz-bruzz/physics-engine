from __future__ import annotations
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List
from physics_engine.math.vec import Vec2, rotate

@dataclass(slots=True)
class AABB:
    min: Vec2
    max: Vec2

    def overlaps(self, other: AABB) -> bool:
        if self.max.x < other.min.x or self.min.x > other.max.x: return False
        if self.max.y < other.min.y or self.min.y > other.max.y: return False
        return True

class Shape(ABC):
    @abstractmethod
    def area(self) -> float: ...
    @abstractmethod
    def inertia(self, mass: float) -> float: ...
    @abstractmethod
    def aabb(self, pos: Vec2, angle: float) -> AABB: ...

@dataclass(slots=True)
class Circle(Shape):
    radius: float

    def area(self) -> float:
        return math.pi * self.radius * self.radius

    def inertia(self, mass: float) -> float:
        return 0.5 * mass * self.radius * self.radius

    def aabb(self, pos: Vec2, angle: float) -> AABB:
        r = Vec2(self.radius, self.radius)
        return AABB(pos - r, pos + r)

@dataclass(slots=True)
class Polygon(Shape):
    vertices: List[Vec2]  # local coordinates

    def area(self) -> float:
        area = 0.0
        n = len(self.vertices)
        for i in range(n):
            v1 = self.vertices[i]
            v2 = self.vertices[(i + 1) % n]
            area += v1.cross(v2)
        return abs(area) * 0.5

    def inertia(self, mass: float) -> float:
        # Simplified inertia for convex polygon
        # sum ( (v1 x v2) * (v1^2 + v1.v2 + v2^2) ) / (6 * sum(v1 x v2))
        num = 0.0
        den = 0.0
        n = len(self.vertices)
        for i in range(n):
            v1 = self.vertices[i]
            v2 = self.vertices[(i + 1) % n]
            cross = abs(v1.cross(v2))
            num += cross * (v1.length_sq() + v1.dot(v2) + v2.length_sq())
            den += cross
        return (mass / 6.0) * (num / den) if den != 0 else 0.0

    def _transformed_vertices(self, pos: Vec2, angle: float) -> List[Vec2]:
        return [pos + rotate(v, angle) for v in self.vertices]

    def aabb(self, pos: Vec2, angle: float) -> AABB:
        tv = self._transformed_vertices(pos, angle)
        xs = [v.x for v in tv]
        ys = [v.y for v in tv]
        return AABB(Vec2(min(xs), min(ys)), Vec2(max(xs), max(ys)))

def make_rect(w: float, h: float) -> Polygon:
    hw, hh = w / 2, h / 2
    return Polygon([
        Vec2(-hw, -hh), Vec2(hw, -hh), Vec2(hw, hh), Vec2(-hw, hh)
    ])
