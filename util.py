import math

SCALE = 25 # pixels/m

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
  ang1, ang2 = veh1.getTheta() % MAX_DEG, veh2.getTheta() % MAX_DEG
  diff = abs(ang1 - ang2)
  return (veh1.getDirection() == veh2.getDirection() and 
    veh1.getRadius() == veh2.getRadius() and
    (veh1.getCarAngle() >= diff or (veh1.getCarAngle() >= abs(diff - 360))))
