
import tkinter as tk
import itertools, math, time
import numpy as np

from pathPlanningFig8 import Env
from tkinter import *
from util import *
import vehicle
import idm


root = tk.Tk()

CANVAS_WIDTH = 1200
CANVAS_HEIGHT = 800
TRACK_WIDTH = vehicle.VEH_WIDTH * 4 # 120
MARGIN = 100
TWO_LANE = False
UPDATE_TIME = 0.015

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

canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT)
canvas.pack()

lastTime, timerCounter = None, 0
timerLabel = Label(text = "0.0 s", fg = "red")
infoLabel = Label(text = "")
loopLabel = Label(text = "0", fg = "darkBlue")
modeLabel = Label(text = "", fg = "DarkOrchid4")
testLabel = Label(text = "")
trackBCoords, detBCoords, modeBCoords = [], [], []

move = False
testing, testTime = False, 5.0
testList, testLists = [], [] # degrees, direction
testResults = []
vehicles = []
detectionRadius = False
totalLoops = 0
mode = COOP
i = 0

def initEnv():
	global env
	env = Env(mode=mode)
	env.setIntersection((610, 367))
	#env.setIntersection((trackX, trackY))
	env.setWeights(np.array([50,1]))

def testCase(m):
	global lastTime, move, testing, mode, testList
	mode = m
	reset()
	for (theta, direc) in testList:
		makeVehicle(theta, direc, vehR)
	lastTime = time.time()
	move, testing = True, True

def keyPress(event):
	global move, lastTime, testList, testLists
	if (event.char == "s"):
		if lastTime == None:
			lastTime = time.time()
			timerCounter = 0
		move = not move
	if (event.char == "r"):
		reset()
	if (event.char == "t"):
		runTesting()
	if (event.char == "1"):
		for v in vehicles:
			v.increaseAngSpeed(1)
	if (event.char == "2"):
		for v in vehicles:
			v.decreaseAngSpeed(1)

	# merge
	if (event.char == "m"):
		 for v in vehicles:
		 	if (v.getRadius() <= vehR + TRACK_WIDTH): v.setMerge(True)

def mousePress(event):
	x, y = event.x, event.y
	changeTrack(x, y)
	addDetectionRadius(x, y)
	changeMode(x, y)
	r, direc = inTrack(x, y)
	if (r != None):
		theta = placeOnTrack(x, y, direc, r)
		makeVehicle(theta, direc, r)

def modeName():
	global mode
	if (mode == COOP): return "Cooperative"
	return "Non-Cooperative"

def inTrack(x, y):
	global vehR
	# Check for overlap
	direc, tX, tY, tR = CLK, trackLeftX, trackY, vehR
	if (TWO_LANE): tX = trackX
	elif (x > CANVAS_WIDTH // 2): direc, tX = CTR_CLK, trackRightX
	for v in vehicles:
		x1 = x - tX
		y1 = y - tY
		r = math.sqrt((x1)**2 + (y1)**2)
		a = toDegrees(math.acos(float(x1 / r)))
		if (y1 > 0): a = (a * -1.0) % MAX_DEG
		if (TWO_LANE and r > outR): tR += TRACK_WIDTH
		checkV = vehicle.Vehicle(tX, tY, tR, a, direc, -1)
		vehicleIntersectionCollide(v, checkV)
		if (vehiclesCollide(v, checkV)): return (None, None)

	# Check for 2 lane track first
	r = math.sqrt((tX - x)**2 + (tY - y)**2)
	if (TWO_LANE and ((r <= outR and r >= (outR - TRACK_WIDTH)) or
			(r <= outR + TRACK_WIDTH and r >= outR))): return (r, CLK)
	elif (r <= outR and r >= (outR - TRACK_WIDTH)): return (r, direc)
	return (None, None)

def placeOnTrack(x, y, direc, r):
	if (TWO_LANE): x -= trackX
	elif (direc == CLK): x -= trackLeftX
	else: x -= trackRightX
	y -= trackY
	a = toDegrees(math.acos(float(x / r)))
	if (y > 0): a = (a * -1.0)
	return a

def flatten(l):
	return [item for tup in l for item in tup]

def makeVehicle(theta, direc, r):
	global info, vehR
	# Add Vehicle
	radius = vehR
	if (TWO_LANE):
		tX = trackX
		if (r <= outR + TRACK_WIDTH and r >= outR): radius += TRACK_WIDTH
	elif (direc == CTR_CLK): tX = trackRightX
	else: tX = trackLeftX

	v = vehicle.Vehicle(tX, trackY, radius, theta, direc, len(vehicles))
	vehCanvas = canvas.create_polygon(v.getVehPoints(), fill='red')
	wheelsCanvas = []
	for wP in v.getWheelPoints():
		wheelsCanvas.append(canvas.create_polygon(wP, fill='black'))
	dirCanvas = canvas.create_polygon(v.getDirPoints(), fill = 'yellow')
	idCanvas = canvas.create_text(v.getX(), v.getY(), text = str(v.getID()))
	detCanvas = canvas.create_oval(v.getX() - idm.DETECTION_DIST,
		v.getY() - idm.DETECTION_DIST, v.getX() + idm.DETECTION_DIST,
		v.getY() + idm.DETECTION_DIST, outline = "DarkOrange2")
	if (detectionRadius): canvas.itemconfig(detCanvas, state = tk.NORMAL)
	else: canvas.itemconfig(detCanvas, state = tk.HIDDEN)
	v.setCanvas([vehCanvas, wheelsCanvas, dirCanvas, idCanvas, detCanvas])
	vehicles.append(v)
	infoLabel.configure(text = infoListToText())

def infoListToText():
	txt = ""
	for i in range(len(vehicles)):
		v = vehicles[i]
		txt += "{}. speed = {:.1f}\n".format(i, v.getAngSpeed())
	return txt

def vehiclesMove():
	global lastTime, info, totalLoops, testing, timerCounter
	if move:
		env.step(vehicles)
		currTime = time.time()
		if (lastTime == None): lastTime = currTime - UPDATE_TIME
		elapsedTime = (currTime - lastTime) / UPDATE_TIME
		for v in vehicles:
			if (v.getRadius() > vehR + TRACK_WIDTH): v.setMerge(False)
			v.update(elapsedTime)
			if (v.getLooped()): totalLoops += 1
			infoLabel.configure(text = infoListToText())
			canvas.coords(v.getVehicleCanvas(), *flatten(v.getVehPoints()))
			for i in range(len(v.getWheelsCanvas())):
				w = v.getWheelsCanvas()[i]
				wP = v.getWheelPoints()[i]
				canvas.coords(w, *flatten(wP))
			canvas.coords(v.getDirCanvas(), *flatten(v.getDirPoints()))
			canvas.coords(v.getIDCanvas(), v.getX(), v.getY())
			canvas.coords(v.getDetRadCanvas(), v.getX() - idm.DETECTION_DIST,
				v.getY() - idm.DETECTION_DIST, v.getX() + idm.DETECTION_DIST,
				v.getY() + idm.DETECTION_DIST)
		timerCounter += (currTime - lastTime)
		timerLabel.config(text = "{0:.1f} s".format(round(timerCounter, 1)))
		loopLabel.config(text = totalLoops)
		lastTime = currTime
		if testing:
			stopTesting()
	else: lastTime = None
	if not testing:
		root.after(1, vehiclesMove)

def runTesting():
	global testLists, lastTime, move, testing, mode, i, testTime
	if i >= 2*len(testLists):
		print('Done')
		move = False
		return

	mode = COOP if i % 2 == 0 else NON_COOP
	reset()
	for (theta, direc) in testLists[int(i)//2][1]:
		makeVehicle(theta, direc, vehR)
	testTime = testLists[int(i)//2][0]
	testLabel.configure(text = "Test " + str(int(i)//2))
	lastTime = time.time()
	move, testing = True, True
	root.after(1, vehiclesMove)

def stopTesting():
	global timerCounter, testTime, move, mode, testResults, firstLoops, i
	if (timerCounter != None and timerCounter >= testTime):
		move = False
		print(modeName(), totalLoops)
		if (mode == COOP):
			firstLoops = totalLoops
		else:
			testResults.append((testTime, firstLoops, totalLoops))
		i += 1
		root.after(1000, runTesting())
		lastTime = time.time()
	else:
		root.after(1, vehiclesMove)


def drawTrack(x, y, outR, inR):
	canvas.create_oval(x - outR, y - outR, x + outR, y + outR,
		fill = 'darkGreen', outline = "")
	canvas.create_oval(x - inR, y - inR, x + inR, y + inR,
		fill = 'white', outline = "")

def figure8Track():
	inR =  outR - TRACK_WIDTH
	drawTrack(trackLeftX, trackY, outR, inR)
	drawTrack(trackRightX, trackY, outR, inR)

	# Checkered finish line
	drawFinishLine(CANVAS_WIDTH // 2 - (TRACK_WIDTH // 2), trackY, 13)
	canvas.create_oval(trackRightX - inR, trackY - inR, trackRightX + inR,
		trackY + inR, fill = 'white', outline = "")

def drawFinishLine(x, y, blocks):
	d = TRACK_WIDTH // blocks
	for j in [-1, 0]:
		y1, y2 = y + d * j, y + d * (j + 1)
		for i in range(blocks + 1):
			x1 = x + i * d
			x2 = x + (i + 1) * d
			if ((i + j) % 2 == 0): color = 'white'
			else: color = 'black'
			canvas.create_rectangle(x1, y1, x2, y2, fill = color, outline = "")

def twoLaneTrack():
	drawTrack(trackX, trackY, outR + TRACK_WIDTH, outR - TRACK_WIDTH)
	canvas.create_oval(trackX - outR, trackY - outR, trackX + outR,
		trackY + outR, dash = (10, 7), outline = "white")

	# Checkered finish line
	drawFinishLine(trackX + outR - TRACK_WIDTH, trackY, 13)
	drawFinishLine(trackX + outR, trackY, 13)

def drawTrackButton():
	global trackBCoords
	x1, y1 = CANVAS_WIDTH - 2 * MARGIN, MARGIN // 4
	x2, y2 = CANVAS_WIDTH - MARGIN // 2, MARGIN // 2
	canvas.create_rectangle(x1, y1, x2, y2, fill = "PaleGreen1")
	canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2,
		text = "Change Track", fill = 'darkGreen')
	trackBCoords = [x1, y1, x2, y2]

def drawDetectionButton():
	global detBCoords
	x1, y1 = CANVAS_WIDTH - 2 * MARGIN, MARGIN // 2
	x2, y2 = CANVAS_WIDTH - MARGIN // 2, MARGIN * 3 // 4
	canvas.create_rectangle(x1, y1, x2, y2, fill = "tan1")
	canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2,
		text = "Detection Radius", fill = 'OrangeRed4')
	detBCoords = [x1, y1, x2, y2]

def drawModeButton():
	global modeBCoords
	x1, y1 = CANVAS_WIDTH - 2 * MARGIN, MARGIN * 3 // 4
	x2, y2 = CANVAS_WIDTH - MARGIN // 2, MARGIN
	canvas.create_rectangle(x1, y1, x2, y2, fill = "plum1")
	canvas.create_text((x1 + x2) // 2, (y1 + y2) // 2,
		text = "Mode", fill = 'DarkOrchid4')
	modeBCoords = [x1, y1, x2, y2]

def drawTimer():
	global loopLabel, timerLabel
	loopLabel.configure(text = str(totalLoops))
	timerLabel.place(x = CANVAS_WIDTH/2 - MARGIN, y = MARGIN // 3 * 2)
	loopLabel.place(x = CANVAS_WIDTH/2 + MARGIN, y = MARGIN // 3 * 2)

def changeTrack(x, y):
	global trackBCoords, TWO_LANE
	[x1, y1, x2, y2] = trackBCoords
	if (x > x1 and x < x2 and y > y1 and y < y2):
		TWO_LANE = not TWO_LANE
		reset()

def addDetectionRadius(x, y):
	global detBCoords, detectionRadius
	[x1, y1, x2, y2] = detBCoords
	if (x > x1 and x < x2 and y > y1 and y < y2):
		detectionRadius = not detectionRadius
		for v in vehicles:
			if (detectionRadius):
				canvas.itemconfig(v.getDetRadCanvas(), state = tk.NORMAL)
			else: canvas.itemconfig(v.getDetRadCanvas(), state = tk.HIDDEN)

def changeMode(x, y):
	global mode, modeBCoords, move
	[x1, y1, x2, y2] = modeBCoords
	if (x > x1 and x < x2 and y > y1 and y < y2):
		if (mode == NON_COOP): mode = COOP
		else: mode = NON_COOP
		modeLabel.configure(text = modeName())
		move = False
		reset()

def reset():
	global vehicles, totalLoops, mode, lastTime, timerCounter
	totalLoops = 0
	lastTime = time.time()
	timerCounter = 0
	timerLabel.config(text = "0.0 s")

	initEnv()
	canvas.delete("all")
	canvas.delete("all")  # just in case lul
	# Add title and car information at top and bottom of screen
	canvas.create_text(CANVAS_WIDTH/2, MARGIN // 10,
		text='      Cooperative vs Non-Cooperative Autonomous Driving')
	canvas.create_text(CANVAS_WIDTH/2 - MARGIN * 5 // 6, MARGIN // 2,
		text='Timer:', fill = 'red')
	canvas.create_text(CANVAS_WIDTH/2 + MARGIN, MARGIN // 2,
		text='   Total Loops:', fill = 'darkBlue')
	vehicles = []

	infoLabel.place(x = MARGIN // 9, y = MARGIN // 9)
	infoLabel.configure(text = "")
	modeLabel.place(x = CANVAS_WIDTH / 2 - MARGIN // 3, y = MARGIN // 5)
	modeLabel.configure(text = modeName())
	testLabel.place(x = CANVAS_WIDTH / 2 - MARGIN // 5, y = MARGIN)
	testLabel.configure(text = "")
	drawTrackButton()
	drawDetectionButton()
	drawModeButton()
	drawTimer()
	pickTrack()

def pickTrack():
	if (TWO_LANE):
		twoLaneTrack()
	else:
		figure8Track()

def getTrack(direc):
	if (direc == CLK): return trackLeftX
	return trackRightX

def validTests(tList):
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
		testLists = list(filter(validTests, tLists))
		runTesting()
		root.mainloop()
		return testResults
	else:
		reset()
		drawTrackButton()

		# Pick track
		pickTrack()
		vehiclesMove()

		root.bind("<Key>", keyPress)
		root.bind("<Button-1>", mousePress)
		root.mainloop()
