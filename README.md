# AMORcontrol

En este repositorio se encuentran los packages de ROS diseñados para el control sin contacto del brazo robótico AMOR a través de un dispositivo Leap Motion. 
# Requisitos

-Leap Motion https://www.leapmotion.com/

-Brazo robótico AMOR http://www.amorrobot.com/

-Ordenador con Ubuntu 12 o más

-Es necesario instalar ROS y, en concreto, se utilizó la distribución kinetic (http://wiki.ros.org/kinetic/Installation).

-Para el programa de control de AMOR hay que instalar YARP http://www.yarp.it/index.html y los códigos modificados se en cuentran en la carpeta codigosAMOR

# Instalación

Para instalar la parte de Leap Motion visitar http://wiki.ros.org/leap_motion

El diseño está basado en ROS así que hay que crear primero un espacion de trabajo catkin (para mas información http://wiki.ros.org/catkin). 

La ubicación, que se ha llamado ubicacionX, donde se va a situar el sistema es de elección propia. Para hacerlo hay que ejecutar los siguientes comandos en el terminal de ubuntu:

1. mkdir -p cd ~/ubicacionX/catkin_ws/src

2. cd ~/ubicacionX/catkin_ws/src

3. catkin_init_workspace

4. cd ~/ubicacionX/catkin_ws

5. catkin_make

Con esto el espacio de trabajo catkin ya estaría establecido. Para instalar los packages hay que ejecutar estos comandos:

6. cd ~/ubicacionX/catkin_ws/src

7. git clone https://github.com/gendibal784/AMORcontrol.git

8. cd ~/ubicacionX/catkin_ws

9. catkin_make

# Ejecución

En total hay 3 ejecutables, 2 en el package gesture y 1 en el package amorpos. Para ver cuales son sólo hay que abrir el archivo CMakeLists.txt correspondiente. Para poder ejecutar los programas hay que lanzar en un terminal los siguientes comandos:

1. cd ~/ubicacionX/catkin_ws

2. catkin_make #Para compilar e instalar el espacio catkin

3. source devel/setup.bash        #Para habilitar el uso de los ejecutables

4. rosrun [package] [ejecutable] #para ejecutar

Si se quieres ejecutar varios elementos a la vez sólo hay que abrir otra terminal y lanzar el último comando con el package y ejecutable correspondiente.
