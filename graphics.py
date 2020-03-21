from track import Track
from vehicle import Vehicle
import tkinter as tk
import itertools

root = tk.Tk()
TK_SILENCE_DEPRECATION=1

canvasWidth = 1200
canvasHeight = 800
margin = 20
canvas = tk.Canvas(root, width=canvasWidth, height=canvasHeight)
move = False
canvas.pack()

vehicles = []

def keyPress(event):
	global move
	if (move): move = False
	else: 
		move = True


def flatten(list):
	return itertools.chain.from_iterable(list)

def makeVehicle(x, y, r, theta):
	# Add Vehicle
	veh = Vehicle(x, y, r, theta)
	vehCanvas = canvas.create_polygon(veh.getVehPoints(), fill = 'red')
	wheelsCanvas = []
	for wP in veh.getWheelPoints():
		wheelsCanvas.append(canvas.create_polygon(wP, fill = 'black'))
	vehicles.append((veh, vehCanvas, wheelsCanvas))

def vehiclesMove():
	if move:
		for v, vehCanvas, wheelsCanvas in vehicles:
			v.turnLeft()
			canvas.coords(vehCanvas, *flatten(v.getVehPoints()))
			for i in range(len(wheelsCanvas)):
				w = wheelsCanvas[i]
				wP = v.getWheelPoints()[i]
				canvas.coords(w, *flatten(wP))
	# vehiclesMove()
	root.after(1, vehiclesMove)


if __name__ == "__main__":
	canvas.create_text(canvasWidth/2, margin, 
		text='Cooperative vs Non-Cooperative Autonomous Driving')

	# Add Track 
	trackLeftX = canvasWidth//4
	trackLeftY = canvasHeight//2

	makeVehicle(trackLeftX, trackLeftY, margin * 5, 0)

	vehiclesMove()

	root.bind("<Key>", keyPress)
	root.mainloop()
