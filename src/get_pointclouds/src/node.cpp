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
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>); // Creamos una nube de puntos vacía
        
        // Convertimos el mensaje de ROS a formato que entiende la librería pcl, y lo guarda en cloud antes de suscribirse
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>); //Creamos otra nube de puntos vacía

        std::cout << "Puntos capturados: " << cloud->size() << std::endl;

        pcl::VoxelGrid<pcl::PointXYZRGB> vGrid; // Creamos un objeto VoxelGrid, para el tipo de punto XYZRGB
        vGrid.setInputCloud(cloud); // Le metemos la nube de puntos
        vGrid.setLeafSize(0.05f, 0.05f, 0.05f); // Divide en cubos de 5x5x5 cm
        vGrid.filter(*cloud_filtered);  // Aplicamos el filtro a cloud y lo guardamos en cloud_filtered

        std::cout << "Puntos tras VG: " << cloud_filtered->size() << std::endl;

        visu_pc = cloud_filtered;  // Se asigna al visualizador la nube de puntos filtrada, para que lo muestre el hilo del visualizador
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
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