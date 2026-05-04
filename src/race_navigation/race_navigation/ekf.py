import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#El robot se mueve en x,y , z es constante (no vuela)
z = 0.002471923828125

#puntos clave, a definir. Para ver un punto hacer click en "Publish point" mientras estas escuchando el topic : ros2 topic echo /clicked_point
waypoints = [
        (2.007901668548584,-0.014654232189059258),(7.9771857261657715,-0.4836463928222656),
        (13.988696098327637,-0.759340226650238),( 16.331212997436523,-0.4010058343410492)
    ]

#Clase que hereda de Node
class ControladorWaypoints(Node):
    def __init__(self):
        super().__init__('filtro_kalman_extendido') #nombre del nodo
        self.get_logger().info('Nodo EKF arrancado')
        self.indice_objetivo = 0 #indice del siguiente waypoint

        self.k_lineal = 0.5 
        self.k_angular = 1.0

        #Distancia mínima para considerar que hemos llegado al waypoint
        self.tolerancia_llegada = 0.2 #metros
    
        self.suscripcion_odom = self.create_subscription( 
            Odometry,          
            '/odom',   
            self.procesar_datos,    
            10                      
        )

        self.publicar_movimiento = self.create_publisher( 
            Twist,          
            '/cmd_vel',    
            10                      
        )

    def procesar_datos(self, msg):
        self.get_logger().info(f'He recibido datos del sistema')

        if self.indice_objetivo > len(waypoints):
            return

        self.get_logger().info(f'posX: {msg.pose.pose.position.x}')
        self.get_logger().info(f'posX: {msg.pose.pose.position.y}')







def main(args=None):
    rclpy.init(args=args)
    nodo_controlador = ControladorWaypoints()

    try:
        rclpy.spin(nodo_controlador)
    except KeyboardInterrupt:
        pass
    finally:
        nodo_controlador.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


"""
// PRÁCTICA 2

// ros2 launch turtlebot_gazebo_race create_multi_robot_race.launch.py 
// ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
// ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=src/mapa_circuito5.yaml
// ros2 run nav2_map_server map_saver_cli -f mapa_circuito
"""