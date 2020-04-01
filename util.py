import math

LEFT_ENTRANCE_THETA = 45
RIGHT_ENTRANCE_THETA = 135

def toRadians(deg):
  return (deg / 360.0) * 2 * math.pi

def toDegrees(rad):
  return (rad / (2 * math.pi)) * 360.0

def toAngular(linSpeed, radius):
  return (linSpeed * 360.0) / (2*math.pi * radius)

def toLinear(angSpeed, radius):
  return (angSpeed / 360.0) * 2*math.pi * radius

def vehiclesCollide(veh1, veh2):
    veh1Min, veh1Max = min(veh1.getAngleBounds()), max(veh1.getAngleBounds())
    veh2Min, veh2Max = min(veh2.getAngleBounds()), max(veh2.getAngleBounds())
    return (((veh1Min > veh2Min and veh1Min < veh2Max) or
        (veh1Max > veh2Min and veh1Max < veh2Max) or
        (veh2Min > veh1Min and veh2Min < veh1Max) or
        (veh2Max > veh1Min and veh2Max < veh1Max)) and
        veh1.getDirection() == veh2.getDirection())
