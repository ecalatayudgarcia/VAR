#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node
{
private:
  // Publisher con el tipo de mensaje específico
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

public:
  RobotDriver() : Node("send_velocity_commands")
  {
    // Configuramos el publisher para el tópico "/cmd_vel"
    // El '10' es el tamaño de la cola (history depth)
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  void giro360(){
    geometry_msgs::msg::Twist cmd; //TIpo de dato q hay q enviar a este topico
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.5; //radianes por segundo

    std::cout << "Iniciando giro de 360 grados..." << std::endl;

    //publicar este mismo mensaje durante 12 segundos (2* PI / 0.5)
    //2pi es una vuelta completa y 0.5 es lo que gira al segundo por eso 12 segundos
    auto start = std::chrono::steady_clock::now();
    while(rclcpp::ok()){
      auto now = std::chrono::steady_clock::now();
      std::chrono::duration<double> tiempo_pasado = now - start;
      if(tiempo_pasado.count() >= 14.4){
        break;
      }
      cmd_vel_pub_->publish(cmd); //publicar el mensaje que le manda el giro
      std::this_thread::sleep_for(100ms);
    }

    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd); //cuando termina el bucle mandarle 0 radianes por segundo para que pare de girar.
    std::cout << "Giro completado y robot detenido." << std::endl;
  
  }

  void driveKeyboard()
  {
    std::cout << "Type a command and then press enter. "
                 "Use '+' to move forward, 'l' to turn left, "
                 "'r' to turn right, '.' to exit.\n";

    geometry_msgs::msg::Twist base_cmd;
    char cmd[50];

    // rclcpp::ok() para verificar el estado del contexto
    while (rclcpp::ok()) {
      std::cin.getline(cmd, 50);

      if (cmd[0] != '+' && cmd[0] != 'l' && cmd[0] != 'r' && cmd[0] != '.') {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      // Reiniciamos valores
      base_cmd.linear.x = 0.0;
      base_cmd.linear.y = 0.0;
      base_cmd.angular.z = 0.0;

      if (cmd[0] == '+') {
        base_cmd.linear.x = 0.25;
      }
      else if (cmd[0] == 'l') {
        base_cmd.angular.z = 0.75;
        base_cmd.linear.x = 0.25;
      }
      else if (cmd[0] == 'r') {
        base_cmd.angular.z = -0.75;
        base_cmd.linear.x = 0.25;
      }
      else if (cmd[0] == '.') {
        break;
      }

      // Publicamos el mensaje
      cmd_vel_pub_->publish(base_cmd);
      
      // Procesar callbacks si los hubiera
      rclcpp::spin_some(this->get_node_base_interface());
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Creamos el puntero al nodo (Instancia esta clase de este codigo y hace que sea compartido con todos los nodos de ROS)
  auto girador = std::make_shared<RobotDriver>();
  
  // Ejecutamos la lógica de teclado
  girador->giro360();
  
  rclcpp::shutdown();
  return 0;
}

//compilar, source install/setup.bash
//ros2 run send_velocity_commands driver_node

//EL TURTLEBOT ESTA SUSCRITO A /cmd_vel