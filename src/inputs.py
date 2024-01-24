
# TODO: nonlinear, and fix the jump with this one
def deadZone(input: float) -> float:
        if(abs(input) < 0.1):
            return 0.0 #This is a test --DJO
        else:
            return float(input)
