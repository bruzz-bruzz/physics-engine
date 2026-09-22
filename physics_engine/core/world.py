from __future__ import annotations
from typing import List, Optional, Tuple
from dataclasses import dataclass
from physics_engine.math.vec import Vec2
from physics_engine.core.bodies import RigidBody
from physics_engine.core.shapes import Circle, Polygon

@dataclass
class Manifold:
    normal: Vec2
    penetration: float
    contact: Vec2

def _circle_circle(a: RigidBody, b: RigidBody) -> Optional[Manifold]:
    d = b.position - a.position
    dist_sq = d.length_sq()
    r = a.shape.radius + b.shape.radius
    if dist_sq >= r * r: return None
    dist = dist_sq**0.5
    n = d / dist if dist != 0 else Vec2(1, 0)
    pen = r - dist
    contact = a.position + n * (a.shape.radius - 0.5 * pen)
    return Manifold(n, pen, contact)

class World:
    def __init__(self, gravity: Vec2 = Vec2(0, -9.81)):
        self.gravity = gravity
        self.bodies: List[RigidBody] = []

    def add_body(self, body: RigidBody):
        self.bodies.append(body)

    def step(self, dt: float, iterations: int = 1):
        for b in self.bodies:
            b.integrate_forces(dt, self.gravity)

        # Collision detection (simple naive N^2)
        contacts = []
        for i in range(len(self.bodies)):
            for j in range(i + 1, len(self.bodies)):
                a, b = self.bodies[i], self.bodies[j]
                if a.is_static and b.is_static: continue
                # Narrowphase
                m = None
                if isinstance(a.shape, Circle) and isinstance(b.shape, Circle):
                    m = _circle_circle(a, b)
                # (Add more collision types here)
                if m: contacts.append((a, b, m))

        # Resolution
        for _ in range(iterations):
            for a, b, m in contacts:
                self._resolve(a, b, m)

        for b in self.bodies:
            b.integrate_velocity(dt)

    def _resolve(self, A: RigidBody, B: RigidBody, m: Manifold):
        # Basic impulse resolution
        rel_vel = B.velocity - A.velocity
        vel_along_normal = rel_vel.dot(m.normal)
        if vel_along_normal > 0: return

        e = min(A.material.restitution, B.material.restitution)
        j = -(1 + e) * vel_along_normal
        j /= (A.inv_mass + B.inv_mass)
        
        impulse = m.normal * j
        A.apply_impulse(impulse * -1.0)
        B.apply_impulse(impulse)
