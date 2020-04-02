import math

# Track Parameters
LEFT_ENTRANCE_THETA = 35
RIGHT_ENTRANCE_THETA = 145
CTR_CLK = 1
CLK = -1
NEUTRAL = 0
COOP = 10
NON_COOP = 11

MAX_DEG = 360.0
MAX_RAD = 2 * math.pi

def toRadians(deg):
  return (deg / MAX_DEG) * MAX_RAD

def toDegrees(rad):
  return (rad / MAX_RAD) * MAX_DEG

def toAngular(linSpeed, radius):
  return (linSpeed * MAX_DEG) / (MAX_RAD * radius)

def toLinear(angSpeed, radius):
  return (angSpeed / MAX_DEG) * MAX_RAD * radius

def vehiclesCollide(veh1, veh2):
    veh1Min, veh1Max = min(veh1.getAngleBounds()), max(veh1.getAngleBounds())
    veh2Min, veh2Max = min(veh2.getAngleBounds()), max(veh2.getAngleBounds())
    return (((veh1Min > veh2Min and veh1Min < veh2Max) or
        (veh1Max > veh2Min and veh1Max < veh2Max) or
        (veh2Min > veh1Min and veh2Min < veh1Max) or
        (veh2Max > veh1Min and veh2Max < veh1Max)) and
        veh1.getDirection() == veh2.getDirection())
