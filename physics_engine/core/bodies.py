from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional
from physics_engine.math.vec import Vec2
from physics_engine.core.shapes import Shape, AABB
from physics_engine.core.material import Material

@dataclass
class RigidBody:
    shape: Shape
    position: Vec2 = Vec2.zero()
    angle: float = 0.0
    velocity: Vec2 = Vec2.zero()
    angular_velocity: float = 0.0
    
    force: Vec2 = Vec2.zero()
    torque: float = 0.0
    
    material: Material = field(default_factory=Material)
    is_static: bool = False
    
    mass: float = 0.0
    inv_mass: float = 0.0
    inertia: float = 0.0
    inv_inertia: float = 0.0
    
    linear_damping: float = 0.0
    angular_damping: float = 0.0

    def __post_init__(self):
        if self.is_static:
            self.mass = float('inf')
            self.inv_mass = 0.0
            self.inertia = float('inf')
            self.inv_inertia = 0.0
        else:
            self.mass = self.shape.area() * self.material.density
            self.inv_mass = 1.0 / self.mass if self.mass > 0 else 0.0
            self.inertia = self.shape.inertia(self.mass)
            self.inv_inertia = 1.0 / self.inertia if self.inertia > 0 else 0.0

    def apply_force(self, f: Vec2, point_world: Optional[Vec2] = None):
        if self.is_static: return
        self.force += f
        if point_world:
            r = point_world - self.position
            self.torque += r.cross(f)

    def apply_impulse(self, j: Vec2, point_world: Optional[Vec2] = None):
        if self.is_static: return
        self.velocity += j * self.inv_mass
        if point_world:
            r = point_world - self.position
            self.angular_velocity += r.cross(j) * self.inv_inertia

    def integrate_forces(self, dt: float, gravity: Vec2):
        if self.is_static: return
        acc = gravity + self.force * self.inv_mass
        self.velocity += acc * dt
        self.angular_velocity += (self.torque * self.inv_inertia) * dt

    def integrate_velocity(self, dt: float):
        if self.is_static: return
        # Damping
        self.velocity *= (1.0 / (1.0 + self.linear_damping * dt))
        self.angular_velocity *= (1.0 / (1.0 + self.angular_damping * dt))
        
        self.position += self.velocity * dt
        self.angle += self.angular_velocity * dt
        
        self.force = Vec2.zero()
        self.torque = 0.0

    def aabb(self) -> AABB:
        return self.shape.aabb(self.position, self.angle)
