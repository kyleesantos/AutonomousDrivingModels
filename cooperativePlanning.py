import math
import numpy as np
import random
from util import *

class Env():
	# Coop_Env houses stats/data on the current state of the environment. initialize NON_COOP OR COOP mode depending on desired control mechanism

	def __init__(self, track_config="figure_8", mode=NON_COOP):
		self.track_config = track_config
		self.mode = mode
		self.intersection = None # should be set by setIntersection if track_config is figure_8
		self.nearIntersectionThreshold = 600 # vehicles with less than this arc distance to intersection are considered as 'approaching' intersection
		self.atIntersectionThreshold = 150 # vehicles with less than this arc distance to intersection are considerted to be 'at' intersectioin
		self.pendingLeft = 0 # number of vehicles pending from left side
		self.pendingRight = 0 # number of vehicles pending from right side
		self.passingVehicle = None # vehicle currently passing through intersection
		self.weights = None
		self.maxPassingCars = 3
		self.vehiclesAtIntersection = None
		self.bufferDistance = 150 # buffer distance between any two cars (should be set appropriately)
		self.maxCarSeparation = self.bufferDistance + 5
		self.decision = None # decision is a list of 'decisions' where a decision is a tuple indicating a side and number of cars to let through
		self.lastPassingVehicle = None
		self.vehicles = None
		self.vehiclesApproachingIntersection = None


	# return value is how far ahead (along circle) that car2 is from car1
	# should only be called with two cars from the same side/circle else
	# set theta to find distance between car1 and a location on its circle
	def getArcDistance(self, car1, car2=None, theta=None):
		if (car1 == car2): return 0

		angle1 = car1.getTheta()
		if (theta is None):
			angle2 = car2.getTheta()
		else:
			angle2 = theta
		circumference = MAX_RAD * car1.getRadius()

		if (car1.getDirection() == CTR_CLK):
			if (angle2 < angle1): arcAngle = MAX_DEG - (angle1 - angle2)
			else: arcAngle = angle2 - angle1
		else:
			if (angle1 < angle2): arcAngle = MAX_DEG - (angle2 - angle1)
			else: arcAngle = angle1 - angle2

		return (arcAngle / MAX_DEG) * circumference


	# gets the euclidean distance of a vehicle to the intersection 
	def getDistanceToIntersection(self, vehicle):
		posX, posY = vehicle.getPos()
		intersectionX, intersectionY = self.intersection

		return math.sqrt((intersectionX - posX)**2 + (intersectionY - posY)**2)

	# returns a feature vector from the given vehicle to use in 
	# determining the vehicle to pass through intersection
	def getFeatureVector(self, vehicle):
		if (vehicle.getDirection() == CLK): pendingCount = self.pendingLeft
		else: pendingCount = self.pendingRight 
		return np.array([1/self.getDistanceToIntersection(vehicle), pendingCount])

	# returns the length of the longest continuous string of cars that are 
	# each within a certain distance from each other, starting from the first car
	def getNumCarsInString(self, distances):
		if len(distances) == 0: return 0
		count = 1
		for i in range(1, len(distances)):
			separation = distances[i] - distances[i - 1]
			if separation > self.maxCarSeparation: break
			else:
				count += 1
		return count

	# assings a score to the left and right sides of the track, indicating priority when choosing 
	# which lane to 'open' at the intersection
	def getSideScore(self, tup):
		vehicleAtIntersection, numCars, distances = tup

		if ((len(distances) == 0) or (not vehicleAtIntersection)): return -1

		minDist = distances[0]
		vector = np.array([1/minDist, numCars])

		if (self.mode == NON_COOP):
			weights = np.array([1, 0])
		elif (self.mode == COOP):
			weights = self.weights

		return np.dot(weights, vector)

	# returns the vehicles approaching the intersection from both lanes, along with their distances to intersection
	# returns the vehicles sorted by distance to intersection
	def getVehiclesApproachingIntersection(self):
		leftVehicles = []
		rightVehicles = []

		for vehicle in self.vehicles:
			if vehicle.getDirection() == CLK:
				arcDistance = self.getArcDistance(car1=vehicle, theta=LEFT_ENTRANCE_THETA)
				if (arcDistance < self.nearIntersectionThreshold):
					leftVehicles.append((vehicle, arcDistance))
			else:
				arcDistance = self.getArcDistance(car1=vehicle, theta=RIGHT_ENTRANCE_THETA)
				if (arcDistance < self.nearIntersectionThreshold):
					rightVehicles.append((vehicle, arcDistance))

		leftVehicles.sort(key=lambda tup: tup[1])
		rightVehicles.sort(key=lambda tup: tup[1])

		return leftVehicles, rightVehicles

	def checkVehiclesAtIntersection(self):
		atLeft, atRight = False, False

		for vehicle in self.vehicles:
			if (vehicle.getDirection() == CLK):
				arcDistance = self.getArcDistance(car1=vehicle, theta=LEFT_ENTRANCE_THETA)
				if (arcDistance < self.atIntersectionThreshold):
					atLeft = True
			else:
				arcDistance = self.getArcDistance(car1=vehicle, theta=RIGHT_ENTRANCE_THETA)
				if (arcDistance < self.atIntersectionThreshold):
					atRight = True

		return atLeft, atRight
	# makes a decision on which cars to let through the intersection
	# returns none if no cars are at the intersection
	def getNextDecision(self):
		leftDecision = None
		rightDecision = None

		# check if any vehicles are at the intersection
		atLeft, atRight = self.checkVehiclesAtIntersection()

		# get number of vehicles close to intersection at both lanes
		leftVehicles, rightVehicles = self.getVehiclesApproachingIntersection()
		leftDistances = [tup[1] for tup in leftVehicles]
		rightDistances = [tup[1] for tup in rightVehicles]

		# one person goes in non coop mode
		if self.mode == NON_COOP:
			numLeft = 1
			numRight = 1
		elif self.mode == COOP:	
			numLeft = self.getNumCarsInString(leftDistances)
			numRight = self.getNumCarsInString(rightDistances)

		leftScore = self.getSideScore((atLeft, numLeft, leftDistances))
		rightScore = self.getSideScore((atRight, numRight, rightDistances))

		# no result case
		if ((leftScore == -1) and (rightScore == -1)): return None

		self.vehiclesApproachingIntersection = leftVehicles, rightVehicles

		if (leftScore >= 0):
			leftDecision = (CLK, min(numLeft, self.maxPassingCars))
		if (rightScore >= 0):
			rightDecision = (CTR_CLK, min(numRight, self.maxPassingCars))

		if self.mode == NON_COOP:
			if (leftScore >= rightScore):
				return [leftDecision, None]
			else:
				return [rightDecision, None]
		elif self.mode == COOP:
			if (leftScore > rightScore):
				return [leftDecision, rightDecision]
			else:
				return [rightDecision, leftDecision]

	def distributeDecision(self):
		#leftVehicles, rightVehicles = self.getVehiclesApproachingIntersection()
		leftVehicles, rightVehicles = self.vehiclesApproachingIntersection
		side = self.decision[0][0]
		numCars = self.decision[0][1]

		if side == CLK: passingVehicles = leftVehicles
		else: passingVehicles = rightVehicles 

		for i in range(numCars):
			vehicle = passingVehicles[i][0]
			vehicle.setPassingIntersection(passing=True)
			if (i == numCars-1):
				self.lastPassingVehicle = vehicle

	# returns vehicle objects each updated with its next position
	def step(self, vehicles):
		self.vehicles = vehicles

		if (not self.decision):
			self.decision = self.getNextDecision()

			if (self.decision):
				self.distributeDecision()

		for vehicle in vehicles:
			if (vehicle.isPassingIntersection()):
				posY = vehicle.getPos()[1]
				if (vehicle.getDirection() == CLK):
					distToIntersection = self.getArcDistance(car1=vehicle, theta=LEFT_ENTRANCE_THETA)
				else:
					distToIntersection = self.getArcDistance(car1=vehicle, theta=RIGHT_ENTRANCE_THETA)
				if ((posY > self.intersection[1]) and (distToIntersection > self.nearIntersectionThreshold)):
					vehicle.setPassingIntersection(passing=False)
					if (self.lastPassingVehicle == vehicle):
						self.lastPassingVehicle = None
						self.decision[0] = self.decision[1]
						self.decision[1] = None
						if (self.decision[0] is None): self.decision = None
						else: 
							self.distributeDecision()
		

	def getRightOfWay(self):
		if self.decision:
			return self.decision[0][0]
		
		return NEUTRAL

	# use this to set the center of the intersection point if track_config is figure_8
	def setIntersection(self, pos):
		self.intersection = pos

	def setWeights(self, weights):
		self.weights = weights
