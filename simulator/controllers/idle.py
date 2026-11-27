class Controller:
    def reset(self, config):
        pass

    def update(self, robot, dt):
        return robot.stop()
