#!/usr/bin/env python
from cmath import pi
import termios, sys, os
from functions import *

__author__ = "Sebastian Campiño, Julian Luna"
__credits__ = ["Felipe Gonzalez", "Sebastian Realpe", "Jose Manuel Fajardo", "Robotis"]

TERMIOS = termios

motor_id = 1
pos_home = [512, 512, 512, 512]
pos_target = [768, 256, 768, 256]

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
    global current_joint
    global motor_id
    global pos_target
    global pos_home
    pressedKey=""
    val=0
    joints=['waist', 'shoulder', 'elbow', 'wrist']
    while pressedKey!="x":
        pressedKey=getkey()
       # if pressedKey == "w":   #Next Joint
           # motor_id=joints.index(current_joint)+1
           # if motor_id<4:
             #   current_joint=joints[motor_id]
            #    motor_id+=1
           # else: 
           #     current_joint=joints[0]
          #      motor_id=joints.index(current_joint)+1
         #   print("Articulación actual: "+current_joint+"\n")
        #    #print("Motor: "+str(motor_id))
       # if pressedKey == "s":  #Previous Joint
         #   motor_id=joints.index(current_joint)+1
         #   if motor_id>1:
            #    current_joint=joints[motor_id-2]
           #     motor_id-=1
          #  else: 
         #       current_joint=joints[len(joints)-1]
         #       motor_id=joints.index(current_joint)+1
        #    print("Articulación actual: "+current_joint+"\n")
       #     #print("Motor: "+str(motor_id))
       # if pressedKey == "d":   #Go to target position
           # #motor_id=joints.index(current_joint)+1
           # jointCommand('', motor_id, 'Goal_Position', pos_target[motor_id-1], 0.5)
           # print("Articulación "+ current_joint +" a pos_target"+"\n")
        if pressedKey == "a":  #Go to Home
            grip()
        if pressedKey == "d":  #Go to Home
            grip(600)
        if pressedKey == "w":  #Go to Home
            drawGeometry([[20,5],[20,-5],[24,-5],[24,5]],30)
        
if __name__== '__main__':
    try:
        limitTorque()
        sendHome()
        ctrlMove()
        print("Press 'a' to grip marker.")
        print("Press 'd' to stop gripping marker.")
        print("Press 'w' to draw the figure.")
    except rospy.ROSInterruptException:
        pass
