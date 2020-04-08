import math
import numpy as np
import random
from util import *
import idm

# vehicles with less than this arc distance to intersection are considered as 'approaching' intersection
REAL_NEAR_INTER_THRESH = 24

# vehicles with less than this arc distance to intersection are considerted to be 'at' intersection
REAL_AT_INTER_THRESH = 6
REAL_MAX_CAR_SEPARATION = 10
MAX_PASSING_CARS = 3

NEAR_INTER_THRESH = SCALE * REAL_NEAR_INTER_THRESH
AT_INTER_THRESH = SCALE * REAL_AT_INTER_THRESH
MAX_CAR_SEPARATION = SCALE * REAL_MAX_CAR_SEPARATION

class Env():
	# Coop_Env houses stats/data on the current state of the environment. initialize NON_COOP OR COOP mode depending on desired control mechanism

	def __init__(self, track_config="figure_8", mode=NON_COOP):
		self.track_config = track_config
		self.mode = mode
		self.intersection = None # should be set by setIntersection if track_config is figure_8
		self.nearIntersectionThreshold = NEAR_INTER_THRESH
		self.atIntersectionThreshold = AT_INTER_THRESH
		self.pendingLeft = 0 # number of vehicles pending from left side
		self.pendingRight = 0 # number of vehicles pending from right side
		self.passingVehicle = None # vehicle currently passing through intersection
		self.weights = None
		self.maxPassingCars = MAX_PASSING_CARS
		self.vehiclesAtIntersection = None
		self.maxCarSeparation = MAX_CAR_SEPARATION
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


	def getChainsOfCars(self, vehiclesDistances, following=False):
		if len(vehiclesDistances) == 0:
			return []

		allChains = []
		currChain = [vehiclesDistances[0][0]]
		for i in range(1,len(vehiclesDistances)):
			separation = vehiclesDistances[i][1] - vehiclesDistances[i - 1][1]
			if separation > self.maxCarSeparation:
				allChains.append(currChain)
				currChain = [vehiclesDistances[i][0]]
			else:
				currChain.append(vehiclesDistances[i][0])
		if (following and len(allChains) > 0 and
			allChains[0][0].getDirection() == self.getRightOfWay()):
			lastDist = self.getArcDistance(allChains[0][0], car2=currChain[-1])
			if lastDist < self.maxCarSeparation:
				allChains[0] = currChain + allChains[0]
			else:
				allChains.append(currChain)
		else:
			allChains.append(currChain)
		return allChains


	# assings a score to the left and right sides of the track, indicating priority when choosing
	# which lane to 'open' at the intersection
	def getSideScore(self, tup):
		vehicleAtIntersection, numCars, distances = tup

		if ((len(distances) == 0) or (not vehicleAtIntersection)): return -1

		minDist = distances[0]

		# div by zero prevention
		if (minDist == 0):
			minDist = 0.25

		vector = np.array([1/minDist, numCars])

		if (self.mode == NON_COOP):
			weights = np.array([1, 0])
		elif (self.mode == COOP):
			weights = self.weights

		return np.dot(weights, vector)

	# returns the vehicles approaching the intersection from both lanes, along with their distances to intersection
	# returns the vehicles sorted by distance to intersection
	# returns vehicles and their distance to the intersection
	def getVehiclesApproachingIntersection(self):
		leftVehicles = []
		rightVehicles = []

		for vehicle in self.vehicles:
			if vehicle.getDirection() == CLK:
				arcDistance = self.getArcDistance(vehicle, theta=LEFT_ENTRANCE_THETA)
				if (arcDistance < self.nearIntersectionThreshold):
					leftVehicles.append((vehicle, arcDistance))
			else:
				arcDistance = self.getArcDistance(vehicle, theta=RIGHT_ENTRANCE_THETA)
				if (arcDistance < self.nearIntersectionThreshold):
					rightVehicles.append((vehicle, arcDistance))

		leftVehicles.sort(key=lambda tup: tup[1])
		rightVehicles.sort(key=lambda tup: tup[1])

		return leftVehicles, rightVehicles

	def getDistancesPerLane(self):
		leftVehicles = []
		rightVehicles = []

		for vehicle in self.vehicles:
			if vehicle.getDirection() == CLK:
				arcDistance = self.getArcDistance(vehicle, theta=LEFT_ENTRANCE_THETA)
				leftVehicles.append((vehicle, arcDistance))
			else:
				arcDistance = self.getArcDistance(vehicle, theta=RIGHT_ENTRANCE_THETA)
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

	def setVehiclesInCriticalSection(self):
		# get vehicles close to intersection at both lanes
		leftVehicles, rightVehicles = self.getVehiclesApproachingIntersection()

		# vehicles are in critical section (i.e it is approaching the intersection or passing through the intersection)
		approachingVehicles = [tup[0] for tup in leftVehicles] + [tup[0] for tup in rightVehicles]
		for vehicle in approachingVehicles:
			if (not vehicle.isInCriticalSection()) :
				vehicle.setInCriticalSection(True)


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

	# updates environment state and makes decision on vehicles to pass through the intersection
	def step(self, vehicles):
		self.vehicles = vehicles

		if (not self.decision):
			self.decision = self.getNextDecision()

			if (self.decision):
				self.distributeDecision()

		self.setVehiclesInCriticalSection()

		for vehicle in vehicles:
			if (vehicle.isPassingIntersection()):
				posY = vehicle.getPos()[1]
				if (vehicle.getDirection() == CLK):
					distToIntersection = self.getArcDistance(car1=vehicle, theta=LEFT_ENTRANCE_THETA)
				else:
					distToIntersection = self.getArcDistance(car1=vehicle, theta=RIGHT_ENTRANCE_THETA)
				if ((posY > self.intersection[1]) and (distToIntersection > self.nearIntersectionThreshold)):
					vehicle.setPassingIntersection(passing=False)
					vehicle.setInCriticalSection(False)
					if (self.lastPassingVehicle == vehicle):
						self.lastPassingVehicle = None
						self.decision[0] = self.decision[1]
						self.decision[1] = None
						if (self.decision[0] is None): self.decision = None
						else:
							self.distributeDecision()

		if self.mode == COOP:
			leftDistances, rightDistances = self.getDistancesPerLane()
			chains = (self.getChainsOfCars(leftDistances, following=True) +
						self.getChainsOfCars(rightDistances, following=True))
			leadingCars = [chain[0] for chain in chains]
			idm.updateAccels(vehicles)
			for (i,chain) in enumerate(chains):
				leadingAccel = leadingCars[i].getAcceleration()
				for vehicle in chain:
					if vehicle.isPassingIntersection() or not vehicle.isInCriticalSection():
						tempAccel = vehicle.getAcceleration()
						vehicle.setAcceleration(max(leadingAccel, tempAccel), angular=True)
		else:
			idm.updateAccels(vehicles)


	def getRightOfWay(self):
		if self.decision:
			return self.decision[0][0]

		return NEUTRAL

	# use this to set the center of the intersection point if track_config is figure_8
	def setIntersection(self, pos):
		self.intersection = pos

	def setWeights(self, weights):
		self.weights = weights
