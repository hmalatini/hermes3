# hermes3
Proyecto Integrador Final

Para empiezar a utilizar este proyecto:
- Instalar las siguientes dependencias:
	sudo apt-get install ros-indigo-pid ros-indigo-rtabmap ros-indigo-rtabmap-ros libsqlite3-dev libpcl-1.7-all libopencv-dev
- Realizar un git clone
- Posicionarse en la carpeta ./hermes3/src y realizar el comando "catkin_init_workspace"
- Poisicionarse en la carpeta hermes3 y realizar el comando "catkin_make"

Luego, lo que hacemos es agregar esto al archivo .bashrc para que sea conocido como paquete de ROS. Para eso (lo hacemos para usuario como y para root):
- user@hostname$ echo "source ~/wanderbot_ws/devel/setup.bash" >> ~/.bashrc
- user@hostname$ source ~/.bashrc

-----EXPLICAR COMO CONFIGURAR:
	- HOSTPOT
	- bashrc con las ip tanto en hermes como en la compu
	- conectarse por ssh a hermes
	- que launchfile correr en hermes y cual en la compu
