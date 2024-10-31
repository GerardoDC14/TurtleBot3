# TurtleBot3
Concentración 7mo Semestre Sistemas Ciberfísicos

Se dividió el procesamiento en dos partes. La parte del bringup para la inicialización de los tópicos y el lanzamiento de slam_toolbox, esto se realiza dentro de la raspberrypi.

ros2 launch slam_toolbox online_async_launch.py

La segunda parte es la de navegación que se procesa dentro del sistema remoto. 

Se llama primero el paquete de navigation de ROS2 para un mapeo inicial de la zona: ros2 launch nav2_bringup navigation_launch.py 
Luego guardamos el mapa: ros2 run nav2_map_server map_saver_cli -f /home/gerardo/turtlebot3_ws/src/map

Y con ese mapa inicial llamamos al navigation del turtlebot, dentro del worskpace del turtlebot: ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=false use_amcl:=true

Finalmente llamamos nuestro nodo de exploración gerardo@gerardo:~/turtlebot3_group6v2$ ros2 run turtlebot3_explore explore 
