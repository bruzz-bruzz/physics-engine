from __future__ import annotations
import math
from dataclasses import dataclass

@dataclass(frozen=True, slots=True)
class Vec2:
    x: float = 0.0
    y: float = 0.0

    def __add__(self, other: Vec2) -> Vec2:
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vec2) -> Vec2:
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, k: float) -> Vec2:
        return Vec2(self.x * k, self.y * k)

    def __rmul__(self, k: float) -> Vec2:
        return self.__mul__(k)

    def __truediv__(self, k: float) -> Vec2:
        return Vec2(self.x / k, self.y / k)

    def dot(self, other: Vec2) -> float:
        return self.x * other.x + self.y * other.y

    def cross(self, other: Vec2) -> float:
        return self.x * other.y - self.y * other.x

    def length_sq(self) -> float:
        return self.x * self.x + self.y * self.y

    def length(self) -> float:
        return math.sqrt(self.length_sq())

    def normalized(self) -> Vec2:
        l = self.length()
        if l == 0:
            return Vec2(0, 0)
        return self / l

    def perp(self) -> Vec2:
        return Vec2(-self.y, self.x)

    @staticmethod
    def zero() -> Vec2:
        return Vec2(0.0, 0.0)

def rotate(v: Vec2, angle: float) -> Vec2:
    c, s = math.cos(angle), math.sin(angle)
    return Vec2(v.x * c - v.y * s, v.x * s + v.y * c)
