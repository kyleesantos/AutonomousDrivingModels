
import tkinter as tk
import itertools, math, time
import numpy as np

from pathPlanningFig8 import Env
from tkinter import *
from util import *
import vehicle
import idm


CANVAS_WIDTH = 1200
CANVAS_HEIGHT = 800
TRACK_WIDTH = vehicle.VEH_WIDTH * 4 # 120
MARGIN = 100

#size of track
outR = (CANVAS_WIDTH + TRACK_WIDTH - 2 * MARGIN) // 4 # 280
vehR = outR - (TRACK_WIDTH // 2) # 220
# left center track (380, 400)
trackLeftX = MARGIN + outR
trackRightX = CANVAS_WIDTH - MARGIN - outR
# right center track (820, 400)
trackY = CANVAS_HEIGHT // 2
# center track
trackX = CANVAS_WIDTH // 2

timerCounter = 0
trackBCoords, detBCoords, modeBCoords = [], [], []

move = False
testing, testTime = False, 5.0
testList, testLists = [], [] # degrees, direction
testResults, allVehSpeed = [], []
vehicles = [] # Vehicle objects
detectionRadius = False
totalLoops = 0
mode = COOP

def initEnv():
	global env
	env = Env(mode=mode)
	env.setIntersection((610, 367))
	env.setWeights(np.array([350,1]))


def modeName():
	global mode
	if (mode == COOP): return "Cooperative"
	return "Non-Cooperative"


def flatten(l):
	return [item for tup in l for item in tup]

def makeVehicle(theta, direc):
	global vehR
	radius = vehR
	if (direc == CTR_CLK): tX = trackRightX
	else: tX = trackLeftX

	v = vehicle.Vehicle(tX, trackY, radius, theta, direc, len(vehicles))
	vehicles.append(v)


def vehiclesMove():
	global info, totalLoops, testing, timerCounter, move
	if move:
		env.step(vehicles)
		for v in vehicles:
			v.update(1.0)
			if (v.getLooped()): totalLoops += 1
		timerCounter += UPDATE_TIME
	else: lastTime = None


def runTesting():
	global testLists, lastTime, move, testing, mode, testTime
	for i in range(2*len(testLists)):
		mode = COOP if i % 2 == 0 else NON_COOP
		reset()
		for (theta, direc) in testLists[int(i)//2][1]:
			makeVehicle(theta, direc)
		testTime = testLists[int(i)//2][0]
		move, testing = True, True
		while not checkStopTesting():
			vehiclesMove()


def checkStopTesting():
	global timerCounter, testTime, move, mode, testResults, firstLoops
	global allVehSpeed
	if (timerCounter >= testTime):
		move = False
		print(modeName(), totalLoops)
		addAverages()
		if (mode == COOP):
			firstLoops = (totalLoops, allVehSpeed)
		else:
			results = [testTime, firstLoops[0]] + firstLoops[1] + [totalLoops] + allVehSpeed
			results += [(firstLoops[0] - totalLoops)  * 100.0  / totalLoops]
			testResults.append(results)
		return True
	return False

def addAverages():
	global allVehSpeed
	allVehSpeed = [0, 0, 0, 0, 0, 0, 0]
	for v in vehicles:
		speed = [v.getAvgAngSpeed(), v.getWaitingTime(), v.getAvgAngAcceleration(),
			v.getAvgAngDeceleration(), v.getNetAcceleration(), v.getTotCntAcceleration(), 
			v.getTotCntDeceleration()]
		allVehSpeed = [allVehSpeed[i] + speed[i] for i in range(len(allVehSpeed))]
	allVehSpeed = [veh / (1.0 * len(vehicles)) for veh in allVehSpeed]


def reset():
	global vehicles, totalLoops, timerCounter
	totalLoops = 0
	timerCounter = 0
	initEnv()
	vehicles = []


def getTrack(direc):
	if (direc == CLK): return trackLeftX
	return trackRightX

def isValidTest(tList):
	global vehR
	test = tList[1]
	veh = []
	for (theta, direc) in test:
		veh.append(vehicle.Vehicle(getTrack(direc), trackY, vehR, theta, direc, -1))
	for i in range(len(veh) - 1):
		for j in range(i + 1, len(veh)):
			if (vehiclesCollide(veh[i], veh[j]) or
				vehicleIntersectionCollide(veh[i], veh[j])): return False
	return True


def runGraphics(tFlag=False, tLists=None):
	global testLists, testResults
	if tFlag:
		testLists = list(filter(isValidTest, tLists))
		runTesting()
		return testResults
	else:
		print("Cannot run interactive mode with simulation graphics.")
