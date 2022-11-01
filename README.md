# RoboticsLab5
Development of the fifth robotics laboratory, focused on using ROS to manipulate the Phantom X Pincher trough inverse kinematics.

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
