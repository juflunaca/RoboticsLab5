# RoboticsLab5 Cinemática Inversa - Phantom X - ROS
## Por: Sebastian Campiño Figueroa, Julián Felipe Luna Castro y Diego Fernando Mejía Hernández

Development of the fifth robotics laboratory, focused on using ROS to manipulate the Phantom X Pincher trough inverse kinematics.


Este repositorio contiene las memorias y programas desarrollados durante la quinta práctica de laboratorio de la materia Robótica. Esta práctica tiene como objetivo afianzar los conocimientos de cinemática inversa mediante el uso de los robots Phantom X Pincher, determinando el modelo cinemático inverso del robot, generando trayectorias a partir de este modelo e implementando el modelo cinématico en MATLAB o Python (en este caso Python). 

## Requerimientos

* Ubuntu 20.04 LTS o version compatible
ROS Noetic
* Espacio de trabajo para catkin correctamente configurado.
* Paquetes de Dynamixel Workbench. Tomado de: https://github.com/fegonzalez7/rob_unal_clase3
* Paquete del robot Phantom X. Tomado de: https://github.com/felipeg17/px_robot.
* MATLAB 2015b o superior.
* Robotics toolbox de mathworks.
* Toolbox de robótica de Peter Corke.
* Actitud y ganas de aprender.

## 1. Instalación del toolbox de Peter Corke

Primero instalamos la version mas reciente del toolbox de robotica de Peter Corke para python siguiendo las instrucciones en su repositorio ([que se encuentra aqui](https://github.com/petercorke/robotics-toolbox-python.git)), corriendo los siguientes comandos en bash:


    git clone https://github.com/petercorke/robotics-toolbox-python.git
    git clone https://github.com/jhavl/swift
    git clone https://github.com/jhavl/spatialgeometry
    git clone https://github.com/petercorke/spatialmath-python.git
    cd robotics-toolbox-python
    pip3 install -e .
    cd ../swift
    pip3 install -e .
    cd ../spatialgeometry
    pip3 install -e .
    cd ../spatialmath-python
    pip3 install -e .

Posteriormente

## Métodos disponibles del toolbox para determinar la cinemática inversa de un manipulador.

Existen multiples comandos del toolbox de Peter Corke que funcionan para determinar la cinematica inversa de un manipulador, los cuales listamos a continuacion:

* **SerialLink.ikine6s** : Calcula la cinematica inversa de forma analitica para robots de 6 grados de libertad con muñeca esferica. Permite hallar una solucion especifica segun los parametros de configuracion dados.
* **SerialLink.ikine3** : Calcula la cinematica inversa para robots con 3 grados de libertad sin muñeca. Es igual a ikine6s pero sin la muñeca esferica.
* **SerialLink.ikine** : Calcula la cinematica inversa por metodos numericos. Es una solucion general y suele preferirse usar otras soluciones especificas para un caso dado. No funciona bien para robots con 4 o 5 grados de libertad.
* **SerialLink.ikunc** : Calcula la cinematica inversa por metodos numericos, sin tener en cuenta los limites de las articulaciones. Requiere el Toolbox de Optimizacion, pues utiliza la funcion fminunc.
* **SerialLink.ikcon** : Calcula la cinematica inversa por metodos numericos, teniendo en cuenta los limites de las articulaciones. Requiere el Toolbox de Optimizacion, pues utiliza la funcion fmincon.
* **SerialLink.ikine_sym** : Calcula la cinemática inversa de forma simbolica, con multiples celdas dependiendo del numero de configuraciones diferentes que se puedan tener para la solucion. Requiere el Symbolic Toolbox de Matlab y es codigo experimental.


## Modelo de cinemática inversa 
* MTH de la herramienta

$$ T= \begin{bmatrix}
nx & ox & ax & xc\\ 
ny & oy & ay & yc\\ 
nz & oz & az & zc\\ 
0 & 0 & 0 & 1
\end{bmatrix}$$

## Video de la producción de trayectorias desarrolladas a partir de Python con el modelo de cinemática inversa


<video width="320" height="240" controls>
  <source src="./Video/VideoLAB5.mp4" type="video/mp4">
</video>

https://user-images.githubusercontent.com/71235347/188256426-ad711185-05a4-40bc-9e22-77289374c11d.mp4
