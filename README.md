# hermes3
Proyecto Integrador Final


Para empiezar a utilizar este proyecto:
- Realizar un git clone
- Posicionarse en la carpeta ./hermes3/src y realizar el comando "catkin_init_workspace"
- Poisicionarse en la carpeta hermes3 y realizar el comando "catkin_make"

Luego, lo que hacemos es agregar esto al archivo .bashrc para que sea conocido como paquete de ROS. Para eso (lo hacemos para usuario como y para root):
- user@hostname$ echo "source ~/wanderbot_ws/devel/setup.bash" >> ~/.bashrc
- user@hostname$ source ~/.bashrc

FALTAN EL SCRIPT PARA COMPILACION DE ALGORITMO DE SLAM