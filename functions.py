import math

import numpy as np
from matplotlib import pyplot as plt
import roboticstoolbox as rtb
from spatialmath import SE3, SO3
from spatialmath.base import rotx, roty, rotz
import rospy
from dynamixel_workbench_msgs.srv import DynamixelCommand


def invKinPhantomX(T):
    """
    Calcula los valores q de la cinematica inversa para el robot Phantom X
    en configuracion codo arriba.
    T: Matriz de transformacion Homogenea
    Salida:
    q: Lista de las valores para las 4 articulaciones en grados. 
    """
    q = [0.0, 0.0, 0.0, 0.0]
    # l = [4.5, 10.5, 10.5, 7.5]
    l = np.array([14.5, 10.7, 10.7, 9])
    try:
        #Desacople de muñeca
        posW = T[0:3, 3] - l[3]*T[0:3, 2]
    
        #Solucion para q1 en rad
        q[0] = math.atan2(T[1,3], T[0,3]) 

        #Solución de mecanismo 2R para q2 y q3
        h = round(posW[2] - l[0],3)
        r = round(math.sqrt(posW[0]**2 + posW[1]**2),3)

        #Solucion para q3
        q[2]= math.acos(round((r**2+h**2-l[1]**2-l[2]**2)/(2*l[1]*l[2]), 3))*(-1)
        #Solucion para q2 usando q3
        q[1] = math.atan2(h,r) + math.atan2(l[2]*math.sin(-q[2]), l[1]+l[2]*math.cos(-q[2]))
        #Se resta el offset de la articulacion
        q[1] = q[1] - math.pi/2   

        #Solucion para q4
        phi = math.atan2(T[2,2], math.sqrt(T[0,2]**2 +T[1,2]**2)) - math.pi/2
        q[3] = phi - q[1] - q[2]
        # Rp = rotz(q[0]).transpose() @ np.array(T[0:3, 0:3])
        # pitch = math.atan2(float(Rp[2,0]),float(Rp[0,0]))
        # q[3] = float(pitch) - q[1] - q[2]
        # while q[3] > (7/6)*math.pi:
        #     q[3] -= 2*math.pi
        q = [value*180/math.pi for value in q]
        return q
        
    except ValueError:
        print("Esta posicion no puede ser alcanzada, o por lo menos no en esta configuracion.")

def drawGeometry(points:list, zDraw=9, lineSteps:int=10):
    #Ubicar en posicion cercana al tablero para dibujar.
    pitchDraw = math.pi/2
    home = (0,0,44.9,0)
    home2 = (10,10,12,pitchDraw) #Calibrar altura!!!
    steps = stepCoordinates(home,home2,lineSteps)
    qList = []
    for x,y,z,pitch in zip(*steps):
        qList.append(findQ(x,y,z,pitch))
    print("Moving towards the board!")
    print(qList)
    linearMove(qList)
    previous=home2
    points.append(points[0]) # Para cerrar la geometria.
    for point in points:
        steps = stepCoordinates(previous,(point[0],point[1],zDraw,pitchDraw),lineSteps)
        qList = []
        for x,y,z,pitch in zip(*steps):
            qList.append(findQ(x,y,z,pitch))
        print(f"Moving towards the point {point}")
        print(qList)
        linearMove(qList)
        previous = (point[0],point[1],zDraw, pitchDraw)
    
    #Retornar a posicion cercana a dibujar y luego a home.
    steps = stepCoordinates(previous,home2,lineSteps)
    qList = []
    for x,y,z,pitch in zip(*steps):
        qList.append(findQ(x,y,z,pitch))
    print("Moving away from the board!")
    print(qList)
    linearMove(qList)
    steps = stepCoordinates(home2,home,lineSteps)
    qList = []
    for x,y,z,pitch in zip(*steps):
        qList.append(findQ(x,y,z,pitch))
    print("Moving towards home!")
    print(qList)
    linearMove(qList)
        
def jointCommand(id_num:int, addr_name:str, value:int, time):
    command = ""
    time = 0.1
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def deg2TenBit(*deg, minValue:int=-150, maxValue:int=150):
    "Returns the value of an angle as ten bits, or a list of them."
    # try:
    for value in deg:
        if not minValue < value < maxValue:
            raise Exception("The value, or one of them, is outside the range of conversion.")
    if len(deg) == 1:
        return round(1023*(deg[0]-minValue)/(maxValue-minValue))
    myList = []
    for value in deg:
        myList.append(round(1023*(value-minValue)/(maxValue-minValue)))
    return myList
    # except Exception:
    #     raise Exception

def stepCoordinates(origin:tuple, endpoint:tuple, steps:int):
    """Retorna una tupla con listas de coordenadas XYZ espaciadas un numero de steps
    desde un origen hasta un destino."""
    if len(origin) != 4 or len(endpoint) != 4:
        raise Exception("Los argumentos deben ser tuplas de puntos tridimensionales y pitch.")
    xStep = (endpoint[0] - origin[0])/steps
    yStep = (endpoint[1] - origin[1])/steps
    zStep = (endpoint[2] - origin[2])/steps
    pitchStep = (endpoint[3] - origin[3])/steps
    x = []
    y = []
    z = []
    pitch = []
    for i in range(1,steps+1):
        x.append(origin[0]+xStep*i)
        y.append(origin[1]+yStep*i)
        z.append(origin[2]+zStep*i)
        pitch.append(origin[3]+pitchStep*i)
    return (x,y,z,pitch)

def findQ(x,y,z,pitch):
    T = np.array(SE3.Trans(x,y,z)*SE3.Ry(pitch))
    #print(T)
    q = invKinPhantomX(T)
    for value in q:
        if not -150 < value < 150:
            raise Exception("One of the values is outside the range of motion.")
    return q
        
def linearMove(qList):
    """Input a list of lists of q values."""
    for q in qList:
        for i, value in enumerate(q):
            jointCommand(i+1,'Goal_Position',deg2TenBit(value),0.05)

def sendHome():
    q = [0,0,0,0,0]
    for i, value in enumerate(q):
        jointCommand(i+1,'Goal_Position',deg2TenBit(value),0.05)

def limitTorque(limits:list=[500,400,300,300,600]):
    jointCommand(1, 'Torque_Limit', limits[0], 0)
    jointCommand(2, 'Torque_Limit', limits[1], 0)
    jointCommand(3, 'Torque_Limit', limits[2], 0)
    jointCommand(4, 'Torque_Limit', limits[3], 0)
    jointCommand(5, 'Torque_Limit', limits[4], 0)

def grip(value=850):
    if not 0 < value < 1023:
        print("The value is outside the range (0 to 1023).")
    else:
        jointCommand(5, 'Goal_Position', value, 0)
