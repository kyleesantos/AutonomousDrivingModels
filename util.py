import math

def toRadians(deg):
  return (deg / 360.0) * 2 * math.pi

def toDegrees(rad):
  return (rad / (2 * math.pi)) * 360.0

def toAngular(linSpeed, radius):
  return (linSpeed * 360.0) / (2*math.pi * radius)

def toLinear(angSpeed, radius):
  return (angSpeed / 360.0) * 2*math.pi * radius