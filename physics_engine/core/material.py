from __future__ import annotations
from dataclasses import dataclass

@dataclass(slots=True)
class Material:
    density: float = 1.0
    restitution: float = 0.5  # bounciness
    friction: float = 0.5
