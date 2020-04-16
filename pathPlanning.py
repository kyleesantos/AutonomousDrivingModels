import math
import numpy as np
import random
from util import *
import idm

# vehicles with less than this arc distance to intersection are considered as 'approaching' intersection
REAL_NEAR_INTER_THRESH = 20
# vehicles with less than this arc distance to intersection are considerted to be 'at' intersection
REAL_AT_INTER_THRESH = 4

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
		self.newChain = None


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


	def getNextVehicleInfo(self, vehicle):
		closestDiff = MAX_DEG
		closestSpeed = -1
		closestVehicle = None
		for vehicle2 in self.vehicles:
			if vehicle != vehicle2 and vehicle.getDirection() == vehicle2.getDirection():
				dist = getArcDistance(vehicle, car2=vehicle2)
				if dist < closestDiff:
					closestDiff = dist
					closestSpeed = vehicle2.getAngSpeed()
		return closestSpeed, closestDiff


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
		print("Making chains")
		excluded = set()
		if (self.newChain):
			excluded = set(self.newChain)
			self.newChain = None

		leftDistances, rightDistances = self.getDistancesPerLane()
		chains = (self.getChainsOfCars(leftDistances) +
					self.getChainsOfCars(rightDistances))
		for chain in chains:
			for vehicle in chain[1::]:
				if vehicle not in excluded:
					vehicle.setChain(True)
					vehicle.setLeadVehicle(chain[0])
		chains.append(list(excluded))
		cs = [[c.getID() for c in chain] for chain in chains]
		print(cs)
		print(self.getRightOfWay())


	def getVehiclesAtIntersection(self):
		minDistLeft = self.atIntersectionThreshold
		minDistRight = self.atIntersectionThreshold
		leftVehicles = []
		rightVehicles = []

		if self.mode == COOP:
			leftVehicles, rightVehicles = self.getDistancesPerLane()
			if len(leftVehicles) > 0:
				minDistLeft = leftVehicles[0][1]
				for i in range(1, len(leftVehicles)):
					separation = leftVehicles[i][1] - leftVehicles[i - 1][1]
					if separation > self.maxCarSeparation:
						leftVehicles = leftVehicles[0::i]
						break
			if len(rightVehicles) > 0:
				minDistRight = rightVehicles[0][1]
				for i in range(1, len(rightVehicles)):
					separation = rightVehicles[i][1] - rightVehicles[i - 1][1]
					if separation > self.maxCarSeparation:
						rightVehicles = rightVehicles[0::i]
						break
			leftVehicles = [veh[0] for veh in leftVehicles]
			rightVehicles = [veh[0] for veh in rightVehicles]
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
		self.makeChains(self.vehicles)
		leftVehicles, rightVehicles = self.vehiclesAtIntersection
		side = self.decision[0][0]
		numCars = self.decision[0][1]

		if side == CLK: passingVehicles = leftVehicles
		else: passingVehicles = rightVehicles

		for i in range(len(passingVehicles)):
			vehicle = passingVehicles[i]
			if (i < numCars):
				vehicle.setPassingIntersection(passing=True)
				if (i == numCars-1):
					self.lastPassingVehicle = vehicle

			# create new chain
			if (i == numCars):
				self.newChain = [vehicle]
				print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~new chain")
				print("starting at " + str(vehicle.getID()))
				vehicle.setChain(False)
				vehicle.setLeadVehicle(None)
			if (i > numCars):
				self.newChain.append(vehicle)
				vehicle.setLeadVehicle(passingVehicles[numCars])


	def updateAccels(self, vehicles):
		idm.updateAccels(vehicles)
		if self.mode == COOP:
			for vehicle in vehicles:
				if vehicle.isInChain():
					currAccel = vehicle.getAcceleration()
					leadAccel = vehicle.getLeadVehicle().getAcceleration()

					speed2, dist = self.getNextVehicleInfo(vehicle)
					speedDiff = vehicle.getAngSpeed() - speed2
					maxAccel = dist - toAngular(idm.BUFFER_DIST*2,vehicle.getRadius()) - (speedDiff / UPDATE_TIME) + leadAccel
					if speedDiff > 0:
						newAccel = min(max(currAccel, leadAccel), maxAccel)
					else:
						newAccel = max(currAccel, leadAccel)
					vehicle.setAcceleration(newAccel, angular=True)


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


	def getRightOfWay(self):
		if self.decision:
			return self.decision[0][0]

		return NEUTRAL

	# use this to set the center of the intersection point if track_config is figure_8
	def setIntersection(self, pos):
		self.intersection = pos

	def setWeights(self, weights):
		self.weights = weights
