import numpy as np
import time
from math import cos,sin,radians
#import matplotlib.pyplot as plt
#intialize data points
particles = []
gaus = []
TUPLE =[]
number_particles = 100
initial_point = [0,0,0]
#function def
def move(data):
    gaussian_array = myran(0,0.3)
    temp = [10,10,0]
    adjust_array = []
    for i in range (0,number_particles):
        dummy = [sum(x) for x in zip(temp,gaussian_array[i])]
        dummy_x = dummy[0]*(cos(data[i][2]))
        dummy_y = dummy[1]*(sin(data[i][2]))
        dummy_t = [dummy_x, dummy_y,0]
        adjust_array.append(dummy_t)
    data = np.array(data) + np.array(adjust_array)
    return data

def myran(mean,var):
    gaus = []
    x_ran = np.random.normal(mean,var,number_particles)
    #y_ran = np.random.normal(mean,var,number_particles)
    #theta_ran = np.random.normal(mean,0.01,number_particles)
    for i in range (0,number_particles):
        test = [x_ran[i],x_ran[i],0]
        gaus.append(test)
    return gaus

def turn_ran(mean,var):
    gaus = []
    theta_ran = np.random.normal(mean,var,number_particles)
    for i in range (0,number_particles):
        test = [0,0,theta_ran[i]]
        gaus.append(test)
    return gaus

def turn(data,value):
    gaussian_array = turn_ran(0,0.03)
    data = np.array(data) + np.array(gaussian_array) + np.array([0,0,value])
    return data

def myprint(data):
    TUPLE = []
    for i in range(0,number_particles):
        TUPLE.append((data[i][0]*15+100,data[i][1]*15+100,data[i][2])) #convert list of list => list of tuples, with offset to shift graph
    print("drawParticles:"+str(TUPLE))
    return

def s_print():
    lines = [(100,100),(100,700),(700,700),(700,100)]
    for i in range(0,3):
        line = (*lines[i],*lines[i+1])
        print("drawLine:"+ str(line))
    line = (*lines[3],*lines[0])
    print("drawLine:"+ str(line))
    return
# main
data = [initial_point] * number_particles
print("Initial data points: \n")
#print(data)
s_print() # print lines out
for i in range(4):
    for j in range(4):
        data = move(data)
        myprint(data)
        time.sleep(0.5) # slow down to visualize print
    data = turn(data,radians(90))  
    myprint(data)
myprint(data)    