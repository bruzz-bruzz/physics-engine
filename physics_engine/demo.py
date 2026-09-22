from physics_engine.core.world import World
from physics_engine.core.bodies import RigidBody
from physics_engine.core.shapes import Circle
from physics_engine.math.vec import Vec2

def main():
    world = World(gravity=Vec2(0, -9.8))
    
    # Ground
    ground = RigidBody(shape=Circle(10.0), position=Vec2(0, -10.0), is_static=True)
    # Ball
    ball = RigidBody(shape=Circle(1.0), position=Vec2(0, 5.0))
    
    world.add_body(ground)
    world.add_body(ball)
    
    for i in range(10):
        world.step(1/60.0)
        print(f"t={i*1/60:.2f}s pos={ball.position}")

if __name__ == "__main__":
    main()
