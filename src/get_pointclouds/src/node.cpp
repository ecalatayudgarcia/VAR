#include <memory>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>         
#include <pcl/features/normal_3d.h>
#include "nav_msgs/msg/odometry.hpp"
#include <pcl/io/pcd_io.h>

// Usamos std::chrono para definir tiempos (ms, s)
using namespace std::chrono_literals;


pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc(new pcl::PointCloud<pcl::PointXYZRGB>); // Nube de puntos que el objeto CloudViewer muestra en un hilo a parte que se llama desde el main
bool viewer_started = false;                                                            //de esta manera podemos visualizar la nube de puntos en tiempo real en Gazebo

void simpleVis() {
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer_started = true;
    while (!viewer.wasStopped()) {
        if (visu_pc->size() > 0) {
            viewer.showCloud(visu_pc);
        }
        std::this_thread::sleep_for(100ms);
    }
}

class PclSubNode : public rclcpp::Node {
public:
    // Constructor que hereda de la clase nodo y que utiliza el atributo privado subscription_ para suscribirse al topic /camera/depth/points, y con .bind hacemos que se llame al callback cada vez que se reciba un mensaje
    PclSubNode() : Node("sub_pcl") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", 10, 
            std::bind(&PclSubNode::callback, this, std::placeholders::_1));  // 10 es el número de mensajes que se guardan en la cola
        odometrySub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, 
            std::bind(&PclSubNode::callback_odom, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { // Aquí he quitado const

        if(hacer_toma){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // Creamos una nube de puntos vacía
        
            // Convertimos el mensaje de ROS a formato que entiende la librería pcl, y lo guarda en cloud antes de suscribirse
            pcl::fromROSMsg(*msg, *cloud);

            std::cout << "Puntos capturados: " << cloud->size() << std::endl;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_no_nan(new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<int> indices; // Guarda los índices de los puntos válidos (lo pide la función)
            pcl::removeNaNFromPointCloud(*cloud, *cloud_no_nan, indices);

            std::cout << "Puntos tras eliminar NaN: " << cloud_no_nan->size() << std::endl;
            
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); //Creamos otra nube de puntos vacía
            pcl::VoxelGrid<pcl::PointXYZRGB> vGrid; // Creamos un objeto VoxelGrid, para el tipo de punto XYZRGB
            vGrid.setInputCloud(cloud_no_nan); // Le metemos la nube de puntos
            vGrid.setLeafSize(0.05f, 0.05f, 0.05f); // Divide en cubos de 5x5x5 cm
            vGrid.filter(*cloud_filtered);  // Aplicamos el filtro a cloud y lo guardamos en cloud_filtered

            std::cout << "Puntos tras VG: " << cloud_filtered->size() << std::endl;

           
            visu_pc = cloud_filtered;  // Se asigna al visualizador la nube de puntos filtrada, para que lo muestre el hilo del visualizador

            std::string nombre = "/home/ros2_ws/src/nubes/scenes/toma_" + std::to_string(numero_toma) + ".pcd";
            pcl::io::savePCDFileASCII(nombre, *cloud_filtered);

            std::cout << "Toma " << numero_toma << " guardada" << std::endl;
            numero_toma++;
            hacer_toma = false;
        }
        
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double q_z = msg->pose.pose.orientation.z;
        double q_w = msg->pose.pose.orientation.w;
        double angle = 2 * atan2(q_z, q_w);
        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(),
        2000,
        "Odometry -> x: " << msg->pose.pose.position.x <<
        " | y: " << msg->pose.pose.position.y <<
        " | ang: " << angle);

        // Normalizamos el ángulo con trigonometría porque en ROS2 los ángulos van de -pi a pi (-180 a 180)
        double distancia_giro = abs(atan2(sin(angle - angulo_ultima_toma), cos(angle - angulo_ultima_toma)));
        if(distancia_giro >= 0.174){ // 0.174 en radianes son 10 grados
            hacer_toma = true;
            angulo_ultima_toma = angle;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub;
    bool hacer_toma = true;
    double angulo_ultima_toma = 0.0;
    int numero_toma = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Hilo para el visualizador
    std::thread t(simpleVis);
    
    // Ejecutar el nodo
    rclcpp::spin(std::make_shared<PclSubNode>());
    
    rclcpp::shutdown();
    t.join();
    return 0;
}

// docker compose -f docker-compose_nvidia.yml up -d

// xhost +local:docker

// docker exec -it var_container_gpu bash

// colcon build --packages-select get_pointclouds  Cada vez que se haga un cambio se compila el paquete

// source install/setup.bash Cada vez que se hace un cambio para refrescar la terminal (recarga rutas)

// ros2 launch turtlebot_gazebo_multiple create_multi_robot.launch.py

// ros2 run teleop_twist_keyboard teleop_twist_keyboard

// ros2 run get_pointclouds get_pointclouds_node

//PAra ejecutar el detector de keypoints: (una vez sacados los puntos en src/nubes/scenes)
// ros2 run get_pointclouds iss

// Ver si la cámara está publicando: ros2 topic list, Ver nodos vivos: ros2 node list

/* Cada vez que se añada una librería nueva, como la de odometría, se tiene que:
    - Añadir como find_package(nav_msgs REQUIRED) en el CMakeLists.txt
    - Enlazar la dependencia (conecta ejecutable con las librerías) en ament_target_dependencies (añadimos ahí nav_msgs)
    - Actualizamos el package.xml, añadiendo el DNI del paquete así: <depend>nav_msgs</depend>*/ 
