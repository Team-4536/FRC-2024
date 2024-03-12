from pyfrc.physics.core import PhysicsEngine as PhysicsEngineBase

class PhysicsEngine(PhysicsEngineBase):
    def __init__(self, controller, robot):
        self.robot = robot
        super().__init__(controller)

    def update_sim(self, now, time_diff):
        pass
