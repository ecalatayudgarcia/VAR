#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
    // 1. Cargar la nube de puntos desde el disco
    
    int toma = 0;

    while(true){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::string filename1 = "/home/ros2_ws/src/nubes/scenes/toma_" + std::to_string(toma) + ".pcd"; 
        std::string filename2 = "/home/ros2_ws/src/nubes/scenes/toma_" + std::to_string(toma + 1) + ".pcd";
        
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename1, *cloud1) == -1 ||
            pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename2, *cloud2) == -1) {
            std::cout << "No se ha detectado la toma: " << filename1 << std::endl;
            break;
        }


        // 2. Configurar y ejecutar el detector ISS
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        
        double model_resolution = 0.05; // voxelgrid de 0.05
        
        pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
        
        iss_detector.setSearchMethod (tree);
        iss_detector.setSalientRadius (6 * model_resolution);
        iss_detector.setNonMaxRadius (4 * model_resolution);
        iss_detector.setThreshold21 (0.975);
        iss_detector.setThreshold32 (0.975);
        iss_detector.setMinNeighbors (5);
        iss_detector.setNumberOfThreads (4);
        
        iss_detector.setInputCloud (cloud1);
        iss_detector.compute (*keypoints1);

        iss_detector.setInputCloud (cloud2);
        iss_detector.compute (*keypoints2);

        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Comparacion de Tomas " + std::to_string(toma) + " y " + std::to_string(toma+1)));
        
        // Creamos dos "Viewports" (pantallas virtuales). v1 será la izquierda y v2 la derecha.
        int v1(0);
        int v2(1);
        
        // Parametros: Xmin, Ymin, Xmax, Ymax (de 0.0 a 1.0)
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); // Mitad izquierda
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2); // Mitad derecha
        
        // Fondo oscuro para ambas mitades
        viewer->setBackgroundColor(0.05, 0.05, 0.05, v1);
        viewer->setBackgroundColor(0.05, 0.05, 0.05, v2);
        
        // Pintamos la nube 1 (Blanca) en la ventana izquierda (v1)
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud1_color(cloud1, 255, 255, 255);
        viewer->addPointCloud(cloud1, cloud1_color, "nube_1", v1);
        
        // Pintamos la nube 2 (Naranja) en la ventana derecha (v2)
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud2_color(cloud2, 255, 120, 65);
        viewer->addPointCloud(cloud2, cloud2_color, "nube_2", v2);
        
        // Pintamos los keypoints 1 (Rojos) en la izquierda (v1)
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> kp1_color(keypoints1, 255, 0, 0);
        viewer->addPointCloud(keypoints1, kp1_color, "keypoints1", v1);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoints1");

        // Pintamos los keypoints 2 (Verdes) en la derecha (v2)
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> kp2_color(keypoints2, 0, 255, 0);
        viewer->addPointCloud(keypoints2, kp2_color, "keypoints2", v2);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoints2");

        // Mantener la ventana abierta
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
        }
        toma++;

    }
    return 0;
}