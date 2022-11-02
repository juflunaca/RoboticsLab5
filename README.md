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
* Python 3.x.x
* Toolbox de robótica de Peter Corke.
* Marcadores Borrables
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


## Modelo de cinemática inversa 

Primero
* MTH de la herramienta

Sabemos la posición y orientación del efector final que se puede dar en una MTH de la herramienta como la siguiente:


$$ T= \begin{bmatrix}
nx & ox & ax & xc\\ 
ny & oy & ay & yc\\ 
nz & oz & az & zc\\ 
0 & 0 & 0 & 1
\end{bmatrix}$$

Teniendo la posición en términos de xc, yc y zc, es fácil encontrar el valor de la primera articulación, y posteriormente determinar el modelo de cinemática inversa.

En general debemos determinar la configuración articular de un manipulador, dadas la posición y orientación del efector final respecto a la base. Este problema puede resolverse mediante métodos geométricos, algebraicos o numéricos. En el caso particular del robot Phantom X el cual posee 4 GDL, el enfoque más práctico es combinar el método geométrico con el desacople de muñeca.


El modelo geométrico construido se muestra a continuación.

<p align="center"><img src="./Images/CI1.png" width=70%></p>

y en general usando algunas relaciones geométricas tenemos:

$$ q1= tan \left( \frac{y_T}{x_T}\right)$$
$$ \theta_{3}= acos \left( \frac{r²+h²-l_2 ²-l_3 ²}{2l_2 l_3}\right)$$
$$ \beta_{3}= atan2 \left( \frac{l_3 \sin{\theta_{3}}}{l_2 + l_3\cos{\theta_{3}}}\right)$$
$$ \alpha = atan2 \left(\frac{h}{r}\right)$$
$$ \theta_{2} = \alpha - \beta $$


## Métodos disponibles del toolbox para determinar la cinemática inversa de un manipulador.

Existen multiples comandos del toolbox de Peter Corke que funcionan para determinar la cinematica inversa de un manipulador, los cuales listamos a continuacion:

* **SerialLink.ikine6s** : Calcula la cinematica inversa de forma analitica para robots de 6 grados de libertad con muñeca esferica. Permite hallar una solucion especifica segun los parametros de configuracion dados.
* **SerialLink.ikine3** : Calcula la cinematica inversa para robots con 3 grados de libertad sin muñeca. Es igual a ikine6s pero sin la muñeca esferica.
* **SerialLink.ikine** : Calcula la cinematica inversa por metodos numericos. Es una solucion general y suele preferirse usar otras soluciones especificas para un caso dado. No funciona bien para robots con 4 o 5 grados de libertad.
* **SerialLink.ikunc** : Calcula la cinematica inversa por metodos numericos, sin tener en cuenta los limites de las articulaciones. Requiere el Toolbox de Optimizacion, pues utiliza la funcion fminunc.
* **SerialLink.ikcon** : Calcula la cinematica inversa por metodos numericos, teniendo en cuenta los limites de las articulaciones. Requiere el Toolbox de Optimizacion, pues utiliza la funcion fmincon.
* **SerialLink.ikine_sym** : Calcula la cinemática inversa de forma simbolica, con multiples celdas dependiendo del numero de configuraciones diferentes que se puedan tener para la solucion. Requiere el Symbolic Toolbox de Matlab y es codigo experimental.



## Video de la producción de trayectorias desarrolladas a partir de Python con el modelo de cinemática inversa


<video align="center" width="640" height="480" controls>
  <source src="./Video/VideoLAB5.mp4" type="video/mp4">
</video>

* Video (GH) *
https://github.com/juflunaca/RoboticsLab5/blob/ee813425c0f81438fae122e540a658a51742dbe0/Video/VideoLAB5.mp4
* Link video (Drive) *
https://drive.google.com/file/d/1NqF9zoS2SGtQgKIfXYO85F51QDGeslfr/view?usp=sharing

## Exactitud  y precisión

A continuación se muestra los resultados obtenidos a medir el rectángulo realizado por el manipualdor según la trayectoria y los via-points definidos en el script de python.

* Altura del cuadrilatero

Según lo definido inicialmente el la longitud de la "altura" de nuestro rectángulo debería ser de 4 cm, en este caso podemos ver a continuación que se obtiene una medida de aproximadamente 4.1mm, obteniendo el siguiente error:

$$\delta = | \frac{v_A-v_E}{v_E} |\cdot 100 \% $$
donde:


$v_A$ = Valor observado


$v_E$ = Valor Esperado


$\delta$ = Error porcentual

En este caso se obtiene un error del 2.5% o un error absoluto de +/- 1mm.

<p align="center"><img src="./Images/AltoCuadrado.jpeg" width=38%></p>



* Ancho del cuadrilatero


Según lo definido inicialmente el la longitud del "ancho" de nuestro rectángulo debería ser de 10 cm, en este caso podemos ver a continuación que se obtiene una medida de aproximadamente 9.51mm, obteniendo que en este caso el error es de 4.9% y +/-0.49mm de error absoluto.


<p align="center"><img src="./Images/AnchoCuadrado.jpeg" width=38%></p>

Respecto a la precisión se seleccionó 30 via-points entre punto y punto, obteniendo el resultado mostrado en el video, como se evidencia los trazos no siguen completamente una trayectoria recta y se desvía ligeramente en algunos puntos, por lo que podríamos decir que la precisión obtenida fue baja debido a que entre punto y punto había un error presente lo suficientemente grande para desviar los trazos del manipualdor.

## Conclusiones

* La presición y exactitud del robot Phantom X es baja debido a que los movimientos son muy bruscos, incluso bajando el delta entre cada punto intermedio de una trayectoria no podemos obtener una trayectoria muy exacta o precisa, esto además se propaga debido a que varios robots del laboratorio tienen algo de libertad de rotación incluso estando energizados debido a que no están bien ajustados a los soportes.

* Una buena comprensión de la cinemática inversa nos permité hacer nuestra propia implementación del control del movimiento del robot, lo cual fue fundamental para implementar la solución de nuestra alicación.

