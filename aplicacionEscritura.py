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
            grip(840)
        if pressedKey == "d":  #Drop Marker
            grip(600)
        if pressedKey == "w":  #Draw rectangle
            drawGeometry([[20,5],[20,-5],[24,-5],[24,5]],11,20)
        
if __name__== '__main__':
    try:
        limitTorque([400,450,400,300,600])
        sendHome()
        print("Press 'a' to grip marker.")
        print("Press 'd' to stop gripping marker.")
        print("Press 'w' to draw the figure.")
        ctrlMove()
    except rospy.ROSInterruptException:
        pass
