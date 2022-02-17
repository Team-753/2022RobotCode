import wpilib
import rev

class Shooter:
    def __init__(self, config = dict) -> None:
        self.flywheel = rev.CANSparkMax(config["Shooter"]["motorID"])
    
class Feeder:
    def __init__(self, config = dict) -> None:
        pass

class BallClimber:
    def __init__(self, config = dict) -> None:
        pass

