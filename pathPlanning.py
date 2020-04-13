import math
import numpy as np
import random
from util import *
import idm

# vehicles with less than this arc distance to intersection are considered as 'approaching' intersection
REAL_NEAR_INTER_THRESH = 24

# vehicles with less than this arc distance to intersection are considerted to be 'at' intersection
REAL_AT_INTER_THRESH = 8
REAL_MAX_CAR_SEPARATION = 12
REAL_CRITICAL_THRESH = 6
MAX_PASSING_CARS = 3

NEAR_INTER_THRESH = SCALE * REAL_NEAR_INTER_THRESH
AT_INTER_THRESH = SCALE * REAL_AT_INTER_THRESH
MAX_CAR_SEPARATION = SCALE * REAL_MAX_CAR_SEPARATION
CRITICAL_THRESH = SCALE * REAL_CRITICAL_THRESH

class Env():
	# Env houses stats/data on the current state of the environment. initialize NON_COOP OR COOP mode depending on desired control mechanism

	def __init__(self, track_config="figure_8", mode=NON_COOP):
		self.track_config = track_config
		self.mode = mode
		self.intersection = None # should be set by setIntersection if track_config is figure_8
		self.nearIntersectionThreshold = NEAR_INTER_THRESH
		self.atIntersectionThreshold = AT_INTER_THRESH
		self.weights = None
		self.maxPassingCars = MAX_PASSING_CARS
		self.vehiclesAtIntersection = None
		self.maxCarSeparation = MAX_CAR_SEPARATION
		self.decision = None # decision is a list of 'decisions' where a decision is a tuple indicating a side and number of cars to let through
		self.lastPassingVehicle = None
		self.vehicles = None
		self.chains = None


	# assings a score to the left and right sides of the track, indicating priority when choosing
	# which lane to 'open' at the intersection
	def getSideScore(self, tup):
		numCars, minDist = tup

		if numCars == 0: return -1

		# div by zero prevention
		if (minDist == 0):
			minDist = 0.25

		vector = np.array([1/minDist, numCars])

		if (self.mode == NON_COOP):
			weights = np.array([1, 0])
		elif (self.mode == COOP):
			weights = self.weights

		return np.dot(weights, vector)


	def getDistancesPerLane(self):
		leftVehicles = []
		rightVehicles = []

		for vehicle in self.vehicles:
			if vehicle.getDirection() == CLK:
				arcDistance = getArcDistance(vehicle, theta=LEFT_ENTRANCE_THETA)
				leftVehicles.append((vehicle, arcDistance))
			else:
				arcDistance = getArcDistance(vehicle, theta=RIGHT_ENTRANCE_THETA)
				rightVehicles.append((vehicle, arcDistance))

		leftVehicles.sort(key=lambda tup: tup[1])
		rightVehicles.sort(key=lambda tup: tup[1])

		return leftVehicles, rightVehicles


	def getChainsOfCars(self, vehiclesDistances):
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
		if (len(allChains) > 0 and
				(allChains[0][0].getDirection() == self.getRightOfWay())):
			lastDist = getArcDistance(allChains[0][0], car2=currChain[-1])
			if lastDist < self.maxCarSeparation:
				allChains[0] = currChain + allChains[0]
			else:
				allChains.append(currChain)
		else:
			allChains.append(currChain)
		return allChains


	def makeChains(self, vehicles):
		leftDistances, rightDistances = self.getDistancesPerLane()
		self.chains = (self.getChainsOfCars(leftDistances) +
						self.getChainsOfCars(rightDistances))
		for chain in self.chains:
			for vehicle in chain[1::]:
				vehicle.setChain(True)
				vehicle.setLeadVehicle(chain[0])
		# chains = [[c.getID() for c in chain] for chain in self.chains]
		# print(chains)
		# print(self.getRightOfWay())


	def getVehiclesAtIntersection(self):
		minDistLeft = self.atIntersectionThreshold
		minDistRight = self.atIntersectionThreshold
		leftVehicles = []
		rightVehicles = []

		if self.mode == COOP:
			for chain in self.chains:
				leadVehicle = chain[0]
				if leadVehicle.getDirection() == CLK:
					arcDistance = getArcDistance(leadVehicle, theta=LEFT_ENTRANCE_THETA)
					if (arcDistance < self.atIntersectionThreshold):
						leftVehicles.extend(chain)
						minDistLeft = arcDistance
				else:
					arcDistance = getArcDistance(leadVehicle, theta=RIGHT_ENTRANCE_THETA)
					if (arcDistance < self.atIntersectionThreshold):
						rightVehicles.extend(chain)
						minDistRight = arcDistance
		else:
			leftVehicles, rightVehicles = self.getDistancesPerLane()
			if len(leftVehicles) > 0 and leftVehicles[0][1] < self.atIntersectionThreshold:
				minDistLeft = leftVehicles[0][1]
				leftVehicles = [veh[0] for veh in leftVehicles]
				leftVehicles = [leftVehicles[0]]
			if len(rightVehicles) > 0 and rightVehicles[0][1] < self.atIntersectionThreshold:
				minDistRight = rightVehicles[0][1]
				rightVehicles = [veh[0] for veh in rightVehicles]
				rightVehicles = [rightVehicles[0]]

		return minDistLeft, minDistRight, leftVehicles, rightVehicles


	# makes a decision on which cars to let through the intersection
	# returns none if no cars are at the intersection
	def getNextDecision(self):
		leftDecision = None
		rightDecision = None

		self.makeChains(self.vehicles)

		# get number of vehicles close to intersection at both lanes
		minDistLeft, minDistRight, leftVehicles, rightVehicles = self.getVehiclesAtIntersection()
		numLeft,numRight = len(leftVehicles), len(rightVehicles)
		self.vehiclesAtIntersection = leftVehicles, rightVehicles

		leftScore = self.getSideScore((numLeft, minDistLeft))
		rightScore = self.getSideScore((numRight, minDistRight))

		# no result case
		if ((leftScore == -1) and (rightScore == -1)):
			self.decision = None
			return

		if (leftScore >= 0):
			if (rightScore < 0):
				leftDecision = (CLK, numLeft)
			else:
				leftDecision = (CLK, min(numLeft, self.maxPassingCars))
		if (rightScore >= 0):
			if (leftScore < 0):
				rightDecision = (CTR_CLK, numRight)
			else:
				rightDecision = (CTR_CLK, min(numRight, self.maxPassingCars))

		if self.mode == NON_COOP:
			if (leftScore >= rightScore):
				self.decision = [leftDecision, None]
			else:
				self.decision =  [rightDecision, None]
		else:
			if (leftScore > rightScore):
				self.decision =  [leftDecision, rightDecision]
			else:
				self.decision =  [rightDecision, leftDecision]


	def distributeDecision(self):
		leftVehicles, rightVehicles = self.vehiclesAtIntersection
		side = self.decision[0][0]
		numCars = self.decision[0][1]

		if side == CLK: passingVehicles = leftVehicles
		else: passingVehicles = rightVehicles

		for i in range(len(passingVehicles)):
			if (i < numCars):
				vehicle = passingVehicles[i]
				vehicle.setPassingIntersection(passing=True)
				if (i == numCars-1):
					self.lastPassingVehicle = vehicle

			# create new chain
			if (i == numCars):
				# print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~new chain")
				# print("starting at " + str(passingVehicles[i].getID()))
				passingVehicles[i].setChain(False)
			if (i > numCars):
				vehicle.setLeadVehicle(passingVehicles[numCars])


	def updateAccels(self, vehicles):
		idm.updateAccels(vehicles)
		if self.mode == COOP:
			for vehicle in vehicles:
				if vehicle.isInChain():
					currAccel = vehicle.getAcceleration()
					leadAccel = vehicle.getLeadVehicle().getAcceleration()
					vehicle.setAcceleration(max(currAccel, leadAccel), angular=True)


	def updateVehicles(self, vehicles, direction):
		for vehicle in vehicles:
			if vehicle.getDirection() == direction:
				if (vehicle.isPassingIntersection()):
					posY = vehicle.getPos()[1]
					if (vehicle.getDirection() == CLK):
						distToIntersection = getArcDistance(car1=vehicle, theta=LEFT_ENTRANCE_THETA)
					else:
						distToIntersection = getArcDistance(car1=vehicle, theta=RIGHT_ENTRANCE_THETA)
					if ((posY > self.intersection[1]) and (distToIntersection > self.nearIntersectionThreshold)):
						vehicle.setPassingIntersection(passing=False)
						if (self.lastPassingVehicle == vehicle):
							self.lastPassingVehicle = None
							self.decision[0] = self.decision[1]
							self.decision[1] = None
							if (self.decision[0] is None): self.decision = None
							else:
								self.distributeDecision()


	# updates environment state and makes decision on vehicles to pass through the intersection
	def step(self, vehicles):
		self.vehicles = vehicles

		if (not self.decision):
			self.getNextDecision()

			if (self.decision):
				self.distributeDecision()

		if self.decision and self.decision[0][0] == CLK:
			self.updateVehicles(vehicles, CLK)
			self.updateVehicles(vehicles, CTR_CLK)
		else:
			self.updateVehicles(vehicles, CTR_CLK)
			self.updateVehicles(vehicles, CLK)

		self.updateAccels(vehicles)

# for chain in chains:
# 	leadingAccel = chain[0].getAcceleration()
# 	for (i,vehicle) in enumerate(chain):
# 		tempAccel = vehicle.getAcceleration()
# 		if vehicle.isPassingIntersection() or not vehicle.isInCriticalSection():
# 			vehicle.setAcceleration(max(leadingAccel, tempAccel), angular=True)
# 		else:
# 			for j in range(i+1,len(chain)):
# 				currAccel = chain[j].getAcceleration()
# 				chain[j].setAcceleration(max(tempAccel, currAccel), angular=True)
# 			break


	def getRightOfWay(self):
		if self.decision:
			return self.decision[0][0]

		return NEUTRAL

	# use this to set the center of the intersection point if track_config is figure_8
	def setIntersection(self, pos):
		self.intersection = pos

	def setWeights(self, weights):
		self.weights = weights
