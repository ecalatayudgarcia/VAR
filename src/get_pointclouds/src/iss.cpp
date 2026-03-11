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
    double model_resolution = 0.02; // voxelgrid de 0.05
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss1_detector;
    iss1_detector.setSearchMethod (tree1_iss);
    iss1_detector.setSalientRadius (0.30);
    iss1_detector.setNonMaxRadius (0.25);
    iss1_detector.setThreshold21 (0.8);
    iss1_detector.setThreshold32 (0.8);
    iss1_detector.setMinNeighbors (10);
    iss1_detector.setNumberOfThreads (4);
    iss1_detector.setInputCloud (cloud1_clean);
    iss1_detector.compute (*keypoints1);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptores1 (new pcl::PointCloud<pcl::FPFHSignature33> ());
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh1;
    fpfh1.setInputCloud(keypoints1);
    fpfh1.setSearchSurface(cloud1_clean);
    fpfh1.setInputNormals(normals1_clean);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1_fpfh (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    fpfh1.setSearchMethod(tree1_fpfh);
    fpfh1.setRadiusSearch(0.35);
    fpfh1.compute(*descriptores1);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapa_global(new pcl::PointCloud<pcl::PointXYZRGB>());
    *mapa_global = *cloud1_clean; 

    // La matriz global empieza siendo la Identidad (sin movimiento)
    Eigen::Matrix4f T_total = Eigen::Matrix4f::Identity();

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
        
        double model_resolution = 0.02; // voxelgrid de 0.05
        
        pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss2_detector;
        
        iss2_detector.setSearchMethod (tree2_iss);
        iss2_detector.setSalientRadius (0.30);
        iss2_detector.setNonMaxRadius (0.25);
        iss2_detector.setThreshold21 (0.8);
        iss2_detector.setThreshold32 (0.8);
        iss2_detector.setMinNeighbors (10);
        iss2_detector.setNumberOfThreads (4);
        iss2_detector.setInputCloud (cloud2_clean);
        iss2_detector.compute (*keypoints2);

        //////////
        // FPFH //
        //////////

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptores2 (new pcl::PointCloud<pcl::FPFHSignature33> ());
        pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh2;
        fpfh2.setInputCloud(keypoints2);
        fpfh2.setSearchSurface(cloud2_clean);
        fpfh2.setInputNormals(normals2_clean);

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2_fpfh (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        fpfh2.setSearchMethod(tree2_fpfh);
        fpfh2.setRadiusSearch(0.35);
        fpfh2.compute(*descriptores2);

        //////////////
        // MATCHING //
        //////////////

        pcl::CorrespondencesPtr correspondencias(new pcl::Correspondences);
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> matching;

        matching.setInputSource(descriptores2);
        matching.setInputTarget(descriptores1);

        matching.determineReciprocalCorrespondences(*correspondencias);
        std::cout << "Correspondencias: " << correspondencias->size() << std::endl;

        ////////////
        // RANSAC //
        ////////////

        // Creamos un nuevo objeto para guardar SOLO las correspondencias buenas
        pcl::CorrespondencesPtr correspondencias_inliers(new pcl::Correspondences);
        
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> ransac;
        ransac.setInputSource(keypoints2);
        
        // OJO: Usamos keypoints2 normal, NO el que tiene el offset para visualizar
        ransac.setInputTarget(keypoints1); 
        
        // Le pasamos la lista de líneas locas iniciales
        ransac.setInputCorrespondences(correspondencias);
        
        // PARÁMETROS CLAVE DEL RANSAC:
        // Tolerancia: ¿A qué distancia máxima (metros) pueden quedar dos puntos tras moverlos 
        // para considerarlos un buen "voto"? (Si tu VoxelGrid es 0.05, 0.10 o 0.15 suele ir bien).
        ransac.setInlierThreshold(0.15); 
        ransac.setMaximumIterations(10000); // Número de veces que intenta buscar la combinación perfecta

        ransac.getCorrespondences(*correspondencias_inliers);

        std::cout << "Lineas ANTES de RANSAC (Con Outliers): " << correspondencias->size() << std::endl;
        std::cout << "Lineas DESPUES de RANSAC (Solo Inliers): " << correspondencias_inliers->size() << std::endl;

        ///////////////////////////
        // MATRIZ INICIAL (SVD) //
        ///////////////////////////

        // ¡AQUÍ ESTÁ LA MAGIA! Calculamos el giro usando SOLO las líneas limpias
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> estimador_svd;
        Eigen::Matrix4f matriz_inicial_svd = Eigen::Matrix4f::Identity();
        
        // Si RANSAC no encontró al menos 3 líneas buenas, esto fallará. Le ponemos un seguro:
        if (correspondencias_inliers->size() >= 3) {
            estimador_svd.estimateRigidTransformation(*keypoints2, *keypoints1, *correspondencias_inliers, matriz_inicial_svd);
            std::cout << "Matriz de Transformacion Inicial (SVD) calculada.\n";
        } else {
            std::cout << "¡CRÍTICO! RANSAC no encontró suficientes Inliers. Usando Identidad.\n";
        }

        /////////
        // ICP //
        /////////

        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(cloud2_clean);
        icp.setInputTarget(cloud1_clean);
        icp.setMaxCorrespondenceDistance(0.15);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-2);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alineacion_temporal(new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*alineacion_temporal, matriz_inicial_svd);

        if (icp.hasConverged()) {
            std::cout << "ICP Convergió. Score: " << icp.getFitnessScore() << std::endl;

            // Esta matriz T_local ya incluye RANSAC + ICP (Es la T_i de la línea 7)
            Eigen::Matrix4f T_local = icp.getFinalTransformation();

            // LÍNEA 8: T_T = T_T * T_i (Acumular el movimiento global)
            T_total = T_total * T_local;

            // LÍNEA 9: M = M U T_T(x_i+1) (Transformar la toma nueva a coordenadas globales y sumar)
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr nube_en_global(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::transformPointCloud(*cloud2_clean,*nube_en_global, T_total);

            *mapa_global +=*nube_en_global;

        } else {
            std::cout << "ALERTA: ICP falló en esta toma." << std::endl;
        }

        toma++;
        cloud1 = cloud2;
        cloud1_clean = cloud2_clean;
        normals1_clean = normals2_clean;
        keypoints1 = keypoints2;
        descriptores1 = descriptores2;
    }

    //////////////////
    // VISUALIZADOR //
    //////////////////

    std::cout << "¡PROCESAMIENTO TERMINADO! Construyendo el mapa global..." << std::endl;
    std::cout << "Puntos totales en el mapa: " << mapa_global->size() << std::endl;

    // Guardamos el mapa en un archivo por si lo quieres usar después
    pcl::io::savePCDFileBinary("/home/ros2_ws/src/nubes/scenes/MAPA_FINAL.pcd", *mapa_global);
    std::cout << "Mapa guardado en disco como MAPA_FINAL.pcd" << std::endl;

    // --- VISUALIZADOR DEL MAPA GLOBAL ---
    pcl::visualization::PCLVisualizer::Ptr visor_mapa(new pcl::visualization::PCLVisualizer("Mapa 3D Completo"));
    visor_mapa->setBackgroundColor(0.05, 0.05, 0.05);

    // Pintamos el mapa con sus colores reales (si los tiene) o en blanco
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_mapa(mapa_global, 200, 200, 200);
    visor_mapa->addPointCloud<pcl::PointXYZRGB>(mapa_global, color_mapa, "nube_mapa_global");
    visor_mapa->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "nube_mapa_global");

    // Añadimos un sistema de coordenadas (ejes XYZ) para ver dónde empezó el robot
    visor_mapa->addCoordinateSystem(1.0);

    while (!visor_mapa->wasStopped()) {
        visor_mapa->spinOnce(100);
    }

    return 0;
}