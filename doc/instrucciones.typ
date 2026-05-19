= Anexo: Instrucciones de ejecución



#v(4em)

Se entregará la práctica con el modelo final entrenado en `src/race_navigation/race_navigation/modelo_piloto.pth`. Ambas versiones del piloto cargan este archivo por defecto.
```bash
#Dependencias
apt update && apt install python3-pip
pip install torch torchvision pandas pillow --break-system-packages
pip install ultralytics --break-system-packages --ignore-installed numpy
pip install "numpy<2" --break-system-packages

#Compilación
colcon build --packages-select turtlebot_gazebo_race turtlebot_gazebo_multiple race_navigation
source install/setup.bash

#Ejecución
ros2 launch turtlebot_gazebo_race create_multi_robot_race.launch.py 
ros2 run race_navigation piloto_cnn #Version CNN
ros2 run race_navigation piloto_logica #Version Híbrida
```