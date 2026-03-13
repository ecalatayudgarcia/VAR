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
#include <pcl/console/print.h>

void visualizarMapaFinal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
void visualizarMatchings(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& , pcl::PointCloud<pcl::PointXYZRGB>::Ptr& , pcl::PointCloud<pcl::PointXYZRGB>::Ptr& , pcl::PointCloud<pcl::PointXYZRGB>::Ptr& , pcl::CorrespondencesPtr&);
void preprocesar(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,  pcl::PointCloud<pcl::Normal>::Ptr&);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr iss(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptoresFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointCloud<pcl::Normal>::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

int main() {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    int toma = 0;
    std::string filename1 = "/home/ros2_ws/src/nubes/scenes/toma_" + std::to_string(toma) + ".pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename1, *cloud1) == -1) {
        std::cout << "No se ha detectado la toma: " << filename1 << std::endl;
        return 1;
    }
    
    //PREPROCESAR
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals1_clean(new pcl::PointCloud<pcl::Normal>);
    preprocesar(cloud1, cloud1_clean, normals1_clean);
    
    //ISS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints1 = iss(cloud1_clean);
    
    //DESCRIPTORES (FPFH)
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptores1 = descriptoresFPFH(cloud1_clean, normals1_clean, keypoints1);

   
    ////////////////////////
    // CREAR MAPA GLOBAL //
    ///////////////////////
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

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::Normal>::Ptr normals2_clean(new pcl::PointCloud<pcl::Normal>);
        preprocesar(cloud2, cloud2_clean, normals2_clean);
        std::cout << "Puntos tras limpiar normales invalidas: " << cloud2_clean->size() << std::endl;

        //////////////////////////////
        /// ISS KEYPOINTS DETECTOR ///
        //////////////////////////////

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2 = iss(cloud2_clean);

        //////////
        // FPFH //
        //////////

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptores2 = descriptoresFPFH(cloud2_clean, normals2_clean, keypoints2);

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

        pcl::CorrespondencesPtr correspondencias_inliers(new pcl::Correspondences);
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> ransac;
        ransac.setInputSource(keypoints2);
        ransac.setInputTarget(keypoints1); 
        ransac.setInputCorrespondences(correspondencias);
        ransac.setInlierThreshold(0.15); 
        ransac.setMaximumIterations(10000); // Número de veces que intenta buscar la combinación perfecta
        ransac.getCorrespondences(*correspondencias_inliers);

        std::cout << "Lineas ANTES de RANSAC (Con Outliers): " << correspondencias->size() << std::endl;
        std::cout << "Lineas DESPUES de RANSAC (Solo Inliers): " << correspondencias_inliers->size() << std::endl;

        ///////////////////////////////////////////////////////////////////////////////////
        // VISUALIZADOR MATCHINGS COMENTAR ESTO PARA VER DIRECTAMENTE EL RESULTADO FINAL //
        ///////////////////////////////////////////////////////////////////////////////////
        //visualizarMatchings(cloud1, cloud2, keypoints1,  keypoints2, correspondencias_inliers);

        ///////////////////////////
        // MATRIZ INICIAL (SVD) //
        ///////////////////////////

        // ¡AQUÍ ESTÁ LA MAGIA! Calculamos el giro usando SOLO las líneas limpias
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB> estimador_svd;
        Eigen::Matrix4f matriz_inicial_svd = Eigen::Matrix4f::Identity();
        
        // Si RANSAC no encontró al menos 3 líneas buenas, esto fallará. Le ponemos un seguro:
        if (correspondencias_inliers->size() >= 3) {
            estimador_svd.estimateRigidTransformation(*keypoints2, *keypoints1, *correspondencias_inliers, matriz_inicial_svd);
            if (matriz_inicial_svd.hasNaN()) {
                std::cout << "¡ALERTA! SVD genero una matriz corrupta (NaN). Forzando Identidad." << std::endl;
                matriz_inicial_svd = Eigen::Matrix4f::Identity();
            } else {
                std::cout << "Matriz de Transformacion Inicial (SVD) calculada con exito.\n";
            }
        } else {
            std::cout << "¡CRÍTICO! RANSAC no encontró suficientes Inliers. Usando Identidad.\n";
        }

        /////////
        // ICP //
        /////////

        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(cloud2_clean);
        icp.setInputTarget(cloud1_clean);
        icp.setMaxCorrespondenceDistance(0.20);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-2);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alineacion_temporal(new pcl::PointCloud<pcl::PointXYZRGB>());
        icp.align(*alineacion_temporal, matriz_inicial_svd);

        if (icp.hasConverged()) {
            std::cout << "ICP Convergió. Score: " << icp.getFitnessScore() << std::endl;
            Eigen::Matrix4f T_local = icp.getFinalTransformation(); // Matriz de transformación
            Eigen::Affine3f affine(T_local);
            float tx, ty, tz, rot_x, rot_y, rot_z;
            pcl::getTranslationAndEulerAngles(affine, tx, ty, tz, rot_x, rot_y, rot_z);

            // Anulamos movimientos imposibles para el Turtlebot SOLO TIENE UN GRADO DE LIBERTAD (ROTAR EN y)
            //tx = 0.0; // No se desliza de lado
            ty = 0.0; // No vuela ni se hunde
            //tz = 0.0; // No avanza ni retrocede
            rot_x = 0.0; // No mira hacia arriba/abajo (Cabeceo)
            rot_z = 0.0; // No ladea la cabeza (Alabeo)

            // Reconstruimos la matriz limpia
            Eigen::Affine3f affine_limpio = pcl::getTransformation(tx, ty, tz, rot_x, rot_y, rot_z);
            T_local = affine_limpio.matrix();
 
            T_total = T_total * T_local;
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

    //////////////////////////////////////
    // VISUALIZADOR MAPA FINAL COMPLETO //
    //////////////////////////////////////
    visualizarMapaFinal(mapa_global);

    return 0;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptoresFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints){
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptores (new pcl::PointCloud<pcl::FPFHSignature33> ());
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh1;
    fpfh1.setInputCloud(keypoints);
    fpfh1.setSearchSurface(cloud);
    fpfh1.setInputNormals(normals);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1_fpfh (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    fpfh1.setSearchMethod(tree1_fpfh);
    fpfh1.setRadiusSearch(0.15);
    fpfh1.compute(*descriptores);
    return descriptores;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr iss(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_iss (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
    iss_detector.setSearchMethod (tree_iss);
    iss_detector.setSalientRadius (0.10);
    iss_detector.setNonMaxRadius (0.05);
    iss_detector.setThreshold21 (0.8);
    iss_detector.setThreshold32 (0.8);
    iss_detector.setMinNeighbors (5);
    iss_detector.setNumberOfThreads (4);
    iss_detector.setInputCloud (cloud);
    iss_detector.compute (*keypoints);
    return keypoints;
}

void preprocesar(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_clean,  pcl::PointCloud<pcl::Normal>::Ptr& normals_clean){
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne; //Objeto de filtro normal
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1_n(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
    ne.setSearchMethod(tree1_n);
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.06); //Radio en el que buscar vecinos
    ne.compute(*cloud_normals);
    // Recorremos las normales calculadas
    for (std::size_t i = 0; i < cloud_normals->size(); ++i) {
        // Si la normal en X, Y o Z NO es NaN, el punto es válido
        if (!std::isnan(cloud_normals->points[i].normal_x) && 
            !std::isnan(cloud_normals->points[i].normal_y) && 
            !std::isnan(cloud_normals->points[i].normal_z)) {
            
            cloud_clean->push_back(cloud->points[i]);
            normals_clean->push_back(cloud_normals->points[i]);
        }
    }
}


void visualizarMapaFinal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& mapa_global){

    std::cout << "¡PROCESAMIENTO TERMINADO! Construyendo el mapa global..." << std::endl;
    std::cout << "Puntos totales en el mapa: " << mapa_global->size() << std::endl;

    // Guardamos el mapa en un archivo por si lo quieres usar después
    pcl::io::savePCDFileBinary("/home/ros2_ws/src/nubes/scenes/MAPA_FINAL.pcd", *mapa_global);
    std::cout << "Mapa guardado en disco como MAPA_FINAL.pcd" << std::endl;

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

void visualizarMatchings(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2, 
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& keypoints2,
                         pcl::CorrespondencesPtr& correspondencias_inliers){
    // 1. Movemos la Toma 2 y sus Keypoints 2 metros hacia la derecha (Eje X)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_offset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2_offset(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    Eigen::Matrix4f offset = Eigen::Matrix4f::Identity();
    offset(0, 3) = 5.0; // Sumamos 2.0 metros en el eje X
    
    pcl::transformPointCloud(*cloud2, *cloud2_offset, offset);
    pcl::transformPointCloud(*keypoints2, *keypoints2_offset, offset);

    // 2. Preparamos el visor (Una sola ventana)
    pcl::visualization::PCLVisualizer::Ptr viewer_corr(new pcl::visualization::PCLVisualizer("Correspondencias (Lado a Lado)"));
    viewer_corr->setBackgroundColor(0.05, 0.05, 0.05);

    // 3. Pintamos las nubes (Toma 1 a la izquierda, Toma 2 desplazada a la derecha)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> c1_color(cloud1, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> c2_color(cloud2_offset, 255, 120, 65);
    viewer_corr->addPointCloud(cloud1, c1_color, "nube_1");
    viewer_corr->addPointCloud(cloud2_offset, c2_color, "nube_2_offset");

    // 4. Pintamos los Keypoints
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> kp1_color(keypoints1, 255, 0, 0);   // Rojos
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> kp2_color(keypoints2_offset, 0, 255, 0);   // Verdes
    viewer_corr->addPointCloud(keypoints1, kp1_color, "keypoints1");
    viewer_corr->addPointCloud(keypoints2_offset, kp2_color, "keypoints2_offset");
    viewer_corr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoints1");
    viewer_corr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoints2_offset");

    // 5. Dibujamos las líneas usando la nube 2 desplazada
    viewer_corr->addCorrespondences<pcl::PointXYZRGB>(keypoints1, keypoints2_offset, *correspondencias_inliers, "lineas_correspondencia");
    viewer_corr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 1.0, "lineas_correspondencia");
    viewer_corr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "lineas_correspondencia");

    std::cout << "Mostrando correspondencias Lado a Lado. Cierra para continuar..." << std::endl;

    while (!viewer_corr->wasStopped()) {
        viewer_corr->spinOnce(100);
    }
}