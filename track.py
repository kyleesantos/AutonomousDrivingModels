from math import sqrt

LEFT = 0
RIGHT = 1
NUM_VEHICLES_PER_TRACK = 3

class Track:

  # Initialized with center points and radii of each circle (h, k, r) and
  # also intersection points of the two circles (x,y)
  def __init__(self, vehiclesLeft, vehiclesRight):
    assert(vehiclesLeft)
    assert(vehiclesRight)
    assert(len(vehiclesLeft) == NUM_VEHICLES_PER_TRACK)
    assert(len(vehiclesRight) == NUM_VEHICLES_PER_TRACK)
    self.initializeTrack(vehiclesLeft, vehiclesRight)


# Computes the equations for the circles and intersection points of track
  def initializeTrack(self, vehiclesLeft, vehiclesRight):
    (h1, k1, r1) = computeCircle([car.get_position() for car in vehiclesLeft])
    (h2, k2, r2) = computeCircle([car.get_position() for car in vehiclesRight])
    points = findIntersection(h1, k1, r1, h2, k2, r2)
    self.intersect1 = points[0]
    self.intersect2 = points[1]
    self.leftCircle = (h1, k1, r1)
    self.rightCircle = (h2, k2, r2)


  # Returns (center_x, center_y, radius) of selected circle
  def getCircle(self, side):
    if (side == LEFT):
      return self.leftCircle
    elif (side == RIGHT):
      return self.rightCircle
    else:
      print("Error: Invalid side argument")
      return None


  # Returns [(x1, y1), (x2, y2)]
  def getIntersections(self):
    return [self.intersect1, self.intersect2]


  # Checks if given point is between the intersection points
  def isInIntersection(self, point):
    intX1 = self.intersect1[0]
    intY1 = self.intersect1[1]
    intX2 = self.intersect2[0]
    intY2 = self.intersect2[1]
    x = point[0]
    y = point[1]

    inX = (x >= intX1 and x <= intX2) or (x >= intX2 and x <= intX1)
    inY = (y >= intY1 and y <= intY2) or (y >= intY2 and y <= intY1)
    return inX and inY



# Returns (center_x, center_y, radius)
def computeCircle(points):
  (x1, y1) = points[0]
  (x2, y2) = points[1]
  (x3, y3) = points[2]

  x12 = x1 - x2
  x13 = x1 - x3

  y12 = y1 - y2
  y13 = y1 - y3

  y31 = y3 - y1
  y21 = y2 - y1

  x31 = x3 - x1
  x21 = x2 - x1

  # x1^2 - x3^
  sx13 = pow(x1, 2) - pow(x3, 2)

  # y1^2 - y3^
  sy13 = pow(y1, 2) - pow(y3, 2)

  sx21 = pow(x2, 2) - pow(x1, 2)
  sy21 = pow(y2, 2) - pow(y1, 2)

  f = (((sx13) * (x12) + (sy13) * 
        (x12) + (sx21) * (x13) + 
        (sy21) * (x13)) // (2 * 
        ((y31) * (x12) - (y21) * (x13))))
            
  g = (((sx13) * (y12) + (sy13) * (y12) + 
        (sx21) * (y13) + (sy21) * (y13)) // 
        (2 * ((x31) * (y12) - (x21) * (y13))))

  c = (-pow(x1, 2) - pow(y1, 2) - 
        2 * g * x1 - 2 * f * y1)

  # eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0  
  # where centre is (h = -g, k = -f) and  
  # radius r as r^2 = h^2 + k^2 - c  
  h = -g
  k = -f
  sqr_of_r = h * h + k * k - c

  # r is the radius  
  r = round(sqrt(sqr_of_r), 5)
  return (h, k, r)


# Helper to find the intersection points given two circles
def findIntersection(h1, k1, r1, h2, k2, r2):
  # rounding issues?
  sub = (pow(r1,2) - pow(r2,2) + pow(h2,2) - pow(h1,2) + pow(k2,2) - pow(k1,2)) / (2*h2 - 2*h1)
  a_y = h2 - h1 + k1 - k2
  b_y = 2*(sub - h1)*(k1 - k2) - 2*k2*(h2 - h1)
  c_y = (h2 - h1)*(pow(k1, 2) + pow(sub, 2) + pow(h1, 2) - pow(r1, 2) - 2*sub*h1)
  d_y = (b_y**2) - (4*a_y*c_y)
  y1 = (-b_y + sqrt(d_y)) / (2*a_y)
  y2 = (-b_y - sqrt(d_y)) / (2*a_y)
  x1 = sub + ((k1 - k2) / (h2 - h1)) * y1
  x2 = sub + ((k1 - k2) / (h2 - h1)) * y2
  return [(x1,y1), (x2,y2)]