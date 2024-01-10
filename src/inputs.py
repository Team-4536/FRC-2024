import wpilib


def deadZone(input: float) -> float:
        if(abs(input) < 0.1):
            return 0.0
        else:
            return float(input)
