from track import Track
from vehicle import Vehicle
from cooperativePlanningv2 import Coop_Env
from tkinter import *
import vehicle
import tkinter as tk
import itertools, math, time
import numpy as np

root = tk.Tk()

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

canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT)
canvas.pack()

counter = 0
timerLabel = Label(text = "0")
infoText = []
move = False
# array of veh object, veh canvas, wheels canvas, triangle canvas
vehicles = []
tempSpace = 20

coop_env = Coop_Env()
coop_env.setIntersection((601, 367))  # y was 397 initially
coop_env.setIntersectionThreshold(159) # threshold was 194 initially
coop_env.setWeights(np.array([0,1]))

def keyPress(event):
	global move, counter
	if (event.char == "s"):
		counter = time.time()
		move = not move
	if (event.char == "r"):
		counter = time.time()
		timerLabel.config(text = "0.0 s")
	if (event.char == "1"):
		for v, q, w, e in vehicles:
			v.increaseAngSpeed(1)
	if (event.char == "2"):
		for v, q, w, e in vehicles:
			v.decreaseAngSpeed(1)

def mousePress(event):
	global kylee
	x, y = event.x, event.y
	r, direc = inTrack(x, y)
	if (r != None):
		theta = placeOnTrack(x, y, direc, r)
		makeVehicle(theta, direc)

def inTrack(x, y):
	# checks left track circle first
	r = math.sqrt((trackLeftX - x)**2 + (trackY - y)**2)
	if (r <= outR and r >= (outR - TRACK_WIDTH)): return (r, vehicle.LEFT)
	# then right track
	r = math.sqrt((trackRightX - x)**2 + (trackY - y)**2)
	if (r <= outR and r >= (outR - TRACK_WIDTH)): return (r, vehicle.RIGHT)
	return (None, None)

def placeOnTrack(x, y, direc, r):
	if (direc == vehicle.LEFT): x -= trackLeftX
	else: x -= trackRightX
	y -= trackY
	a = math.acos(float(x / r))
	if (y > 0): a = (a * -1.0) % (2 * math.pi)
	return a

def flatten(l):
	return [item for tup in l for item in tup]

def makeVehicle(theta, direc):
	global infoText
	# Add Vehicle
	if (direc == vehicle.LEFT):
		veh = Vehicle(trackLeftX, trackY, vehR, theta, direc, len(vehicles))
	else:
		veh = Vehicle(trackRightX, trackY, vehR, theta, direc, len(vehicles))
	theta = "{0:.2f}".format(round(veh.getTheta(), 2))
	info = Label(text = "{}. speed = {}, theta = {} \n".format(veh.getID(),
		veh.getAngSpeed(), theta))
	info.place(x = CANVAS_WIDTH // 2,
		y = CANVAS_HEIGHT - MARGIN + veh.getID() * tempSpace)
	infoText.append(info)
	vehCanvas = canvas.create_polygon(veh.getVehPoints(), fill='red')
	wheelsCanvas = []
	for wP in veh.getWheelPoints():
		wheelsCanvas.append(canvas.create_polygon(wP, fill='black'))
	dirCanvas = canvas.create_polygon(veh.getDirPoints(), fill = 'yellow')
	vehicles.append((veh, vehCanvas, wheelsCanvas, dirCanvas))


def vehiclesMove():
	global infoText, counter, kylee
	if move:
		cars = [vehicle[0] for vehicle in vehicles]
		coop_env.step(cars)
		for v, vehCanvas, wheelsCanvas, dirCanvas in vehicles:
			#v.turn()
			# info = infoText[v.getID()]
			# theta = "{0:.2f}".format(round(v.getTheta(), 2))
			# info.configure(text = "{}. speed = {}, theta = {} \n".format(v.getID(), v.getAngSpeed(), theta))
			canvas.coords(vehCanvas, *flatten(v.getVehPoints()))
			for i in range(len(wheelsCanvas)):
				w = wheelsCanvas[i]
				wP = v.getWheelPoints()[i]
				canvas.coords(w, *flatten(wP))
			canvas.coords(dirCanvas, *flatten(v.getDirPoints()))
		timerLabel.config(text = "{0:.1f} s".format(round(time.time() - counter, 1)))
	root.after(10, vehiclesMove)


def drawTrack(x, y, outR, inR):
	canvas.create_oval(x - outR, y - outR, x + outR, y + outR,
		fill = 'darkGreen', outline = "")
	canvas.create_oval(x - inR, y - inR, x + inR, y + inR,
		fill = 'white', outline = "")



if __name__ == "__main__":
	# Add title and car information at top and bottom of screen
	canvas.create_text(CANVAS_WIDTH/2, MARGIN // 3,
		text='Cooperative vs Non-Cooperative Autonomous Driving')
	timerLabel.place(x = CANVAS_WIDTH/2, y = MARGIN // 3 * 2)

	# Add Track
	drawTrack(trackLeftX, trackY, outR, outR - TRACK_WIDTH)
	drawTrack(trackRightX, trackY, outR, outR - TRACK_WIDTH)

	vehiclesMove()

	root.bind("<Key>", keyPress)
	root.bind("<Button-1>", mousePress)
	root.mainloop()
