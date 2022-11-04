#!/usr/bin/env python
from cmath import pi
import termios, sys, os
from functions import *

__author__ = "Sebastian Campiño, Julian Luna"
__credits__ = ["Felipe Gonzalez", "Sebastian Realpe", "Jose Manuel Fajardo", "Robotis"]

TERMIOS = termios

def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd) #retorna lista que contiene los atributos del descriptor
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c.decode("utf-8")

#Esta función crea un publicador para enviar los parámetros asociados a las rutinas que se invocan con las teclas de control.
def ctrlMove():
    pressedKey=""

    while pressedKey!="x":
        pressedKey=getkey()
        if pressedKey == "a":  #Grip marker
            markerRoutine()

def markerRoutine():
    gripMarker()
    points = 5
    points = [i for i in range(points)]
    r = 4
    angles = [(72*i)*math.pi/180 for i in points]
    outer = [[r*math.cos(theta)+20, r*math.sin(theta)] for theta in angles]
    r = 2
    angles = [(72*i+36)*math.pi/180 for i in points]
    inner = [[r*math.cos(theta)+20, r*math.sin(theta)] for theta in angles]
    star = []
    for i in points:
        star.append(outer[i])
        star.append(inner[i])
    print(star)
    drawGeometry(star,12,16)
        
if __name__== '__main__':
    try:
        limitTorque([400,600,400,300,600])
        sendHome()
        print("Press 'a' to start.")
        ctrlMove()
    except rospy.ROSInterruptException:
        pass
