from track import Track
from vehicle import Vehicle
import vehicle
import tkinter as tk
import itertools, math

root = tk.Tk()
TK_SILENCE_DEPRECATION=1

CANVAS_WIDTH = 1200
CANVAS_HEIGHT = 800
TRACK_WIDTH = vehicle.VEH_WIDTH * 4
MARGIN = 60

outR = (CANVAS_WIDTH + TRACK_WIDTH - 2 * MARGIN) // 4
vehR = outR - (TRACK_WIDTH // 2)
canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT)
move = False
canvas.pack()

vehicles = []

def keyPress(event):
	global move
	move = not move

def flatten(l):
	return [item for tup in l for item in tup]

def makeVehicle(x, y, r, theta):
	# Add Vehicle
	veh = Vehicle(x, y, r, theta)
	vehCanvas = canvas.create_polygon(veh.getVehPoints(), fill='red')
	wheelsCanvas = []
	for wP in veh.getWheelPoints():
		wheelsCanvas.append(canvas.create_polygon(wP, fill='black'))
	dirCanvas = canvas.create_polygon(veh.getDirPoints(), fill = 'yellow')
	vehicles.append((veh, vehCanvas, wheelsCanvas, dirCanvas))


def vehiclesMove():
	if move:
		for v, vehCanvas, wheelsCanvas, dirCanvas in vehicles:
			v.turnLeft()
			canvas.coords(vehCanvas, *flatten(v.getVehPoints()))
			for i in range(len(wheelsCanvas)):
				w = wheelsCanvas[i]
				wP = v.getWheelPoints()[i]
				canvas.coords(w, *flatten(wP))
			canvas.coords(dirCanvas, *flatten(v.getDirPoints()))
	root.after(1, vehiclesMove)


def drawTrack(x, y, outR, inR):
	canvas.create_oval(x - outR, y - outR, x + outR, y + outR, 
		fill = 'darkGreen', outline = "")
	canvas.create_oval(x - inR, y - inR, x + inR, y + inR,
		fill = 'white', outline = "")


if __name__ == "__main__":
	canvas.create_text(CANVAS_WIDTH/2, MARGIN,
		text='Cooperative vs Non-Cooperative Autonomous Driving')

	# Add Track
	trackLeftX = MARGIN + outR
	trackLeftY = CANVAS_HEIGHT // 2
	trackRightX = CANVAS_WIDTH - MARGIN - outR
	drawTrack(trackLeftX, trackLeftY, outR, outR - TRACK_WIDTH)
	drawTrack(trackRightX, trackLeftY, outR, outR - TRACK_WIDTH)

	makeVehicle(trackLeftX, trackLeftY, vehR, 0)
	makeVehicle(trackLeftX, trackLeftY, vehR, math.pi / 2)
	makeVehicle(trackRightX, trackLeftY, vehR, 0)

	vehiclesMove()

	root.bind("<Key>", keyPress)
	root.mainloop()
