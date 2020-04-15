#!/usr/bin/env python3 

# Some suitable functions and data structures for drawing a map and particles

import time
import random
import math
import numpy as np

# Functions to generate some dummy particles data:
def calcX():
    return random.gauss(80,3) + 70*(math.sin(t)); # in cm

def calcY():
    return random.gauss(70,3) + 60*(math.sin(2*t)); # in cm

def calcW():
    return random.random();

def calcTheta():
    return random.randint(0,360);

def calDis(ax,ay,bx,by,x,y,theta):
    result = ((by-ay)*(ax-x) - (bx-ax)*(ay-y))/((by-ay)*math.cos(theta)-(bx-ax)*math.sin(theta))
    return result

def myswap(x,y):
    return y,x

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print ("drawLine:" + str((x1,y1,x2,y2)))
        
    def drawParticles(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print ("drawParticles:" + str(display));

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);
    def get_wall(self,index):
        return self.walls[index]

    def findwall(self,x,y,theta,dummy):
        alldis = []
        index = []
        for i in range(len(self.walls)):
            test = calDis(*self.walls[i],x,y,theta)
            print(test)
            if (test<0): # discard negative distance
                print("Negative test")
                continue
            elif (not self.check_within(x+test*math.cos(theta),y+test*math.sin(theta),theta,test)): # check intersection not within endpoints
                print("Not within endpoints ")
                continue
            else: 
                print("Found within endpoints")       
                alldis.append(test)
                index.append(i) 
                print(alldis)
        result = np.array(alldis)        
        return [result.min(),index[result.argmin()]]

    def check_within(self,x,y,theta,dis):
        for i in range(len(self.walls)):
            temp = self.walls[i]
            x_upper = temp[2]
            x_lowwer = temp[0]
            y_upper = temp[3]
            y_lowwer = temp[1]
            if (x_upper < x_lowwer):
                x_upper,x_lowwer = myswap(x_upper,x_lowwer)
            if (y_upper < y_lowwer):
                y_upper,y_lowwer = myswap(y_upper,y_lowwer)
            if (x>= x_lowwer and x<=x_upper and y>= y_lowwer and y<=y_upper):
                return True
        return False
# Simple Particles set
class Particles:
    def __init__(self):
        self.n = 10;    
        self.data = [(200,30,math.radians(100),1)] * self.n

    def update(self):
        self.data = [(calcX(), calcY(), calcTheta(), calcW()) for i in range(self.n)];
    
    def draw(self):
        canvas.drawParticles(self.data);
        
    def get_particles(self):
        return self.data[0]

canvas = Canvas();    # global canvas we are going to draw on

mymap = Map();
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
mymap.draw();
name_list = ["a","b","c","d","e","f","g","h"]

print(mymap.get_wall(0))

particles = Particles();
particles.draw()

try:
    dummy = mymap.findwall(*particles.get_particles())
    print(dummy)
    print(f"Intersect with wall: {name_list[dummy[1]]}")
except ValueError as e:
    print(e)
# t = 0;
# while True:
#     particles.update();
#     particles.draw();
#     t += 0.05;
#     time.sleep(0.05);
# Example code to generate random points


