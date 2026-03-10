#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh.h>

int main() {
    // 1. Cargar la nube de puntos desde el disco
    
    int toma = 0;
    std::string filename1 = "/home/ros2_ws/src/nubes/scenes/toma_" + std::to_string(toma) + ".pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename1, *cloud1) == -1) {
        std::cout << "No se ha detectado la toma: " << filename1 << std::endl;
        return 1;
    }
    pcl::PointCloud<pcl::Normal>::Ptr cloud1_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne; //Objeto de filtro normal
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1_n(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree1_n);
    ne.setInputCloud(cloud1);
    ne.setRadiusSearch(0.15); //Radio en el que buscar vecinos
    ne.compute(*cloud1_normals);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals1_clean(new pcl::PointCloud<pcl::Normal>);

    // Recorremos las normales calculadas
    for (std::size_t i = 0; i < cloud1_normals->size(); ++i) {
        // Si la normal en X, Y o Z NO es NaN, el punto es válido
        if (!std::isnan(cloud1_normals->points[i].normal_x) && 
            !std::isnan(cloud1_normals->points[i].normal_y) && 
            !std::isnan(cloud1_normals->points[i].normal_z)) {
            
            cloud1_clean->push_back(cloud1->points[i]);
            normals1_clean->push_back(cloud1_normals->points[i]);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1_iss (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    double model_resolution = 0.05; // voxelgrid de 0.05
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss1_detector;
    iss1_detector.setSearchMethod (tree1_iss);
    iss1_detector.setSalientRadius (6 * model_resolution);
    iss1_detector.setNonMaxRadius (4 * model_resolution);
    iss1_detector.setThreshold21 (0.975);
    iss1_detector.setThreshold32 (0.975);
    iss1_detector.setMinNeighbors (5);
    iss1_detector.setNumberOfThreads (4);
    iss1_detector.setInputCloud (cloud1_clean);
    iss1_detector.compute (*keypoints1);

    while(true){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::string filename2 = "/home/ros2_ws/src/nubes/scenes/toma_" + std::to_string(toma + 1) + ".pcd";
        
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename2, *cloud2) == -1) {
            std::cout << "No se ha detectado la toma: " << filename2 << std::endl;
            break;
        }

        /////////////////////////
        /// PREPROCESAMIENTO  ///
        /////////////////////////

        //NUBE DE NORMALES:: Contiene la normal (Direccion del plano de vecinos perpendicular para cada punto)
        pcl::PointCloud<pcl::Normal>::Ptr cloud2_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne2; //Objeto de filtro normal
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2_n(new pcl::search::KdTree<pcl::PointXYZRGB>());
        ne2.setSearchMethod(tree2_n);
        ne2.setInputCloud(cloud2);
        ne2.setRadiusSearch(0.15); //Radio en el que buscar vecinos
        ne2.compute(*cloud2_normals);
            
        std::cout << "Puntos tras calcular normales: " << cloud2_normals->size() << std::endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::Normal>::Ptr normals2_clean(new pcl::PointCloud<pcl::Normal>);

        // Recorremos las normales calculadas
        for (std::size_t i = 0; i < cloud2_normals->size(); ++i) {
            // Si la normal en X, Y o Z NO es NaN, el punto es válido
            if (!std::isnan(cloud2_normals->points[i].normal_x) && 
                !std::isnan(cloud2_normals->points[i].normal_y) && 
                !std::isnan(cloud2_normals->points[i].normal_z)) {
                
                cloud2_clean->push_back(cloud2->points[i]);
                normals2_clean->push_back(cloud2_normals->points[i]);
            }
        }
        std::cout << "Puntos tras limpiar normales invalidas: " << cloud2_clean->size() << std::endl;

        //////////////////////////////
        /// ISS KEYPOINTS DETECTOR ///
        //////////////////////////////

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2_iss (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        
        double model_resolution = 0.05; // voxelgrid de 0.05
        
        pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss2_detector;
        
        iss2_detector.setSearchMethod (tree2_iss);
        iss2_detector.setSalientRadius (6 * model_resolution);
        iss2_detector.setNonMaxRadius (4 * model_resolution);
        iss2_detector.setThreshold21 (0.975);
        iss2_detector.setThreshold32 (0.975);
        iss2_detector.setMinNeighbors (5);
        iss2_detector.setNumberOfThreads (4);
        iss2_detector.setInputCloud (cloud2);
        iss2_detector.compute (*keypoints2);

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
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud1_color(cloud1_clean, 255, 255, 255);
        viewer->addPointCloud(cloud1_clean, cloud1_color, "nube_1", v1);
        
        // Pintamos la nube 2 (Naranja) en la ventana derecha (v2)
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud2_color(cloud2_clean, 255, 120, 65);
        viewer->addPointCloud(cloud2_clean, cloud2_color, "nube_2", v2);
        
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
        cloud1_clean = cloud2_clean;
        normals1_clean = normals2_clean;
        keypoints1 = keypoints2;
    }
    return 0;
}