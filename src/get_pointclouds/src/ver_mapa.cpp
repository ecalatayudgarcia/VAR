#include <iostream> 
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/keypoints/iss_3d.h> 
#include <pcl/search/kdtree.h> 
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/features/fpfh.h> 
#include <pcl/visualization/pcl_plotter.h> 
#include <pcl/registration/correspondence_estimation.h> 
#include <pcl/common/transforms.h> 
#include <pcl/registration/correspondence_rejection_sample_consensus.h> 
#include <pcl/registration/icp.h> 
#include <pcl/registration/gicp.h> 
#include <pcl/console/print.h> 

void visualizarMapaFinal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& mapa_global) { 
    std::cout << "¡PROCESAMIENTO TERMINADO! Construyendo el mapa global..." << std::endl; 
    std::cout << "Puntos totales en el mapa: " << mapa_global->size() << std::endl; 

    // Guardamos el mapa en un archivo por si lo quieres usar después (opcional, ya lo tienes guardado en este caso)
    pcl::io::savePCDFileBinary("/home/ros2_ws/src/nubes/scenes/MAPA_FINAL_Copia.pcd", *mapa_global); 
    std::cout << "Mapa guardado en disco como MAPA_FINAL_Copia.pcd" << std::endl; 

    // --- VISUALIZADOR DEL MAPA GLOBAL --- 
    pcl::visualization::PCLVisualizer::Ptr visor_mapa(new pcl::visualization::PCLVisualizer("Mapa 3D Completo")); 
    visor_mapa->setBackgroundColor(0.05, 0.05, 0.05); 

    // Pintamos el mapa con sus colores reales (si los tiene) o en blanco 
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(mapa_global); 
    visor_mapa->addPointCloud<pcl::PointXYZRGB>(mapa_global, rgb, "nube_mapa_global"); 
    visor_mapa->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "nube_mapa_global"); 

    // Añadimos un sistema de coordenadas (ejes XYZ) para ver dónde empezó el robot 
    visor_mapa->addCoordinateSystem(1.0); 

    while (!visor_mapa->wasStopped()) { 
        visor_mapa->spinOnce(100); 
    } 
} 

int main() {
    // Silenciamos los warnings de PCL para tener una terminal limpia
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // Creamos el puntero donde se alojará el mapa en la memoria RAM
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapa_cargado(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Ruta exacta de tu mapa global
    std::string filename = "/home/ros2_ws/src/nubes/scenes/MAPA_FINAL.pcd";

    std::cout << "Cargando mapa desde el disco: " << filename << std::endl;

    // Función de PCL para leer el archivo. Si devuelve -1, es que no lo ha encontrado
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *mapa_cargado) == -1) {
        std::cout << "ERROR FATAL: No se ha podido encontrar o leer el archivo: " << filename << std::endl;
        std::cout << "¿Seguro que la ruta está bien y el archivo existe?" << std::endl;
        return -1;
    }

    std::cout << "¡Éxito! Mapa cargado correctamente." << std::endl;

    // Le pasamos el mapa recién cargado a tu función de visualización
    visualizarMapaFinal(mapa_cargado);

    return 0;
}