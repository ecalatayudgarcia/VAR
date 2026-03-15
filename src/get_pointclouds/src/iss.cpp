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

void visualizarMapaFinal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
void visualizarMatchings(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& , pcl::PointCloud<pcl::PointXYZRGB>::Ptr& , pcl::PointCloud<pcl::PointXYZRGB>::Ptr& , pcl::PointCloud<pcl::PointXYZRGB>::Ptr& , pcl::CorrespondencesPtr&);
void preprocesar(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&,  pcl::PointCloud<pcl::Normal>::Ptr&);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr iss(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptoresFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointCloud<pcl::Normal>::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

int main() {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    int toma = 0;
    std::string filename1 = "/home/ros2_ws/src/nubes/scenes/toma_" + std::to_string(toma) + ".pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>); // Creamos una nube vacía que contendrá la primera toma
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename1, *cloud1) == -1) { // Si intentamos guardar la toma y devuelve -1 es que no se ha detectado la toma
        std::cout << "No se ha detectado la toma: " << filename1 << std::endl;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_sin_suelo(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& pt : cloud1->points)
        if (pt.z > 0.10f)
            cloud1_sin_suelo->push_back(pt);
    
    //PREPROCESAR
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_clean(new pcl::PointCloud<pcl::PointXYZRGB>); // Creamos una nube de puntos vacía donde meteremos la nube limpia de normales NaN 
    pcl::PointCloud<pcl::Normal>::Ptr normals1_clean(new pcl::PointCloud<pcl::Normal>); // Creamos una nube de de tipo Normal que contendrá las normales válidas

    preprocesar(cloud1_sin_suelo, cloud1_clean, normals1_clean);
    
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

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_sin_suelo(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& pt : cloud2->points)
            if (pt.z > 0.10f)
                cloud2_sin_suelo->push_back(pt);

        /////////////////////////
        /// PREPROCESAMIENTO  ///
        /////////////////////////

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::Normal>::Ptr normals2_clean(new pcl::PointCloud<pcl::Normal>);
        preprocesar(cloud2_sin_suelo, cloud2_clean, normals2_clean);
        std::cout << "Puntos tras limpiar normales invalidas: " << cloud2_clean->size() << std::endl;

        //////////////////////////////
        /// ISS KEYPOINTS DETECTOR ///
        //////////////////////////////

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints2 = iss(cloud2_clean); // Nube vacía donde guardaremos los puntos clave

        //////////
        // FPFH //
        //////////

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptores2 = descriptoresFPFH(cloud2_clean, normals2_clean, keypoints2);
        if (keypoints2->size() < 5) {
            std::cout << "ALERTA: ISS detectó muy pocos keypoints (" << keypoints2->size() << "). Toma saltada." << std::endl;
            toma++;
            cloud1 = cloud2;
            cloud1_clean = cloud2_clean;
            normals1_clean = normals2_clean;
            keypoints1 = keypoints2;
            descriptores1 = descriptores2;
            continue;
        }

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
        ransac.setInlierThreshold(0.15); //umbral
        ransac.setMaximumIterations(5000); // Número de veces que intenta buscar la combinación perfecta
        ransac.getCorrespondences(*correspondencias_inliers);

        std::cout << "Lineas ANTES de RANSAC (Con Outliers): " << correspondencias->size() << std::endl;
        std::cout << "Lineas DESPUES de RANSAC (Solo Inliers): " << correspondencias_inliers->size() << std::endl;


        ///////////////////////////////////////////////////////////////////////////////////
        // VISUALIZADOR MATCHINGS COMENTAR ESTO PARA VER DIRECTAMENTE EL RESULTADO FINAL //
        ///////////////////////////////////////////////////////////////////////////////////
        if(toma == 19){
            //visualizarMatchings(cloud1, cloud2, keypoints1,  keypoints2, correspondencias_inliers);

        }   

        ///////////////////////////
        // MATRIZ INICIAL (SVD) //
        ///////////////////////////
        Eigen::Matrix4f matriz_inicial = Eigen::Matrix4f::Identity();
        
        // Solo calculamos SVD si RANSAC dejó al menos 3 líneas sanas (el mínimo matemático)
        if (correspondencias_inliers->size() >= 3) {
            matriz_inicial = ransac.getBestTransformation();
            // Calculamos cuánta distancia (traslación) nos dice el SVD que nos hemos movido
            float tx = matriz_inicial(0, 3);
            float ty = matriz_inicial(1, 3);
            float tz = matriz_inicial(2, 3);
            float distancia_svd = std::sqrt(tx*tx + ty*ty + tz*tz);

            if (distancia_svd > 0.30) {
                std::cout << "SVD alucinado (Salto de " << distancia_svd << "m). Toma 100% corrupta. SALTANDO." << std::endl;
                // Actualizamos las variables para no quedarnos atascados, pero NO añadimos nada al mapa
                matriz_inicial = Eigen::Matrix4f::Identity(); // Reseteamos
            } else {
                std::cout << "SVD calculada con exito (Distancia lógica: " << distancia_svd << "m)." << std::endl;
            }
        } else {
            std::cout << "ALERTA: Muy pocos inliers (" << correspondencias_inliers->size() << "). El ICP tendra que hacerlo solo." << std::endl;
        }

        /////////
        // ICP //
        /////////
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(cloud2_clean);
        icp.setInputTarget(cloud1_clean);
        icp.setMaxCorrespondenceDistance(0.90); 
        icp.setMaximumIterations(40);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-2);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alineacion_temporal(new pcl::PointCloud<pcl::PointXYZRGB>());
        
        // ¡LE PASAMOS LA MATRIZ DEL SVD AL ICP!
        icp.align(*alineacion_temporal, matriz_inicial);

        if (icp.hasConverged()) {
            double score = icp.getFitnessScore();
            std::cout << "ICP Convergió. Score: " << score << std::endl;
    
            Eigen::Matrix4f T_local = icp.getFinalTransformation(); 
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
            pcl::transformPointCloud(*cloud2_clean, *nube_en_global, T_total);

            *mapa_global += *nube_en_global;
            cloud1 = cloud2;
            cloud1_clean = cloud2_clean;
            normals1_clean = normals2_clean;
            keypoints1 = keypoints2;
            descriptores1 = descriptores2;
        } else {
            std::cout << "ALERTA: ICP falló en esta toma." << std::endl;
        }

        toma++; 
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
    fpfh1.setRadiusSearch(0.3);
    fpfh1.compute(*descriptores);
    return descriptores;
}

 /* El algoritmo ISS se basa en usar una matriz de dispersión donde aquellos puntos que tienen más vecinos tienen menos peso que los que tienen menos vecinos, de esta manera
    si la cámara escanea la mesa, la pata más cerca de esta tendrá más puntos que la que está más lejos, por lo tanto, se le da menos peso a la que está más cerca para que 
    no se piense que la pata más cerca es más voluminosa o que tiene una forma distinta, así el algoritmo se centra en la forma geométrica real*/

    /*λ1​: Es el eje donde los puntos se esparcen más (el largo).
      λ2​: Es la dispersión en el eje intermedio (el ancho).
      λ3​: Es la dirección donde hay menos puntos (el grosor).*/

pcl::PointCloud<pcl::PointXYZRGB>::Ptr iss(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>()); // Nube vacía donde guardaremos los puntos clave
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_iss (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector; // Invocamos el algoritmo ISS para usar sus métodos
    iss_detector.setSearchMethod (tree_iss);
    iss_detector.setSalientRadius (0.25); // Dibuja una esfera virtual de x centímetros de radio a su alrededor y ve como están distribuidos geométricamente los puntos que caen en esa esfera
    iss_detector.setNonMaxRadius (0.12); // Supresión de no máximos: es para tener un keypoint limpio por cada esquina, borra el resto en un radio de x centímetros
    iss_detector.setThreshold21 (0.5); // Si este ratio sale cerca de 1 el keypoint es malo, ya que largo y ancho son iguales, y hay peligro de simetría circular o cilíndrica
    iss_detector.setThreshold32 (0.5); // Si sale cerca de 1 el ancho y grosor son iguales (forma de tubo cuadrado), si sale muy bajo, significa que no tiene nada de grosor comparado con su ancho, peligro de pared plana
    iss_detector.setMinNeighbors (8); // Mínimo número de vecinos que tiene que tener, esto permite eliminar ruido
    iss_detector.setNumberOfThreads (4); // Usar 4 núcleos a la vez para calcular
    iss_detector.setInputCloud (cloud); // Le metemos la nube de puntos limpia de normales inválidas
    iss_detector.compute (*keypoints); // Metemos el cálculo en keypoints1
    return keypoints;
}

void preprocesar(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_clean,  pcl::PointCloud<pcl::Normal>::Ptr& normals_clean){
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne; // Objeto de filtro normal para calcular las normales
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree1_n(new pcl::search::KdTree<pcl::PointXYZRGB>()); // Para optimizar la búsqueda de vecinos de un punto
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>()); // Creamos una nube vacía de tipo Normal
    ne.setSearchMethod(tree1_n); // Decimos que como método de búsqueda utilice el KdTree
    ne.setInputCloud(cloud); // Le metemos la nube de puntos
    ne.setRadiusSearch(0.1); // Radio en el que buscar vecinos
    ne.compute(*cloud_normals); // Calcula y lo guarda en cloud1_normals
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
    viewer_corr->addCorrespondences<pcl::PointXYZRGB>(keypoints2_offset, keypoints1, *correspondencias_inliers, "lineas_correspondencia");
    viewer_corr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 1.0, "lineas_correspondencia");
    viewer_corr->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "lineas_correspondencia");

    std::cout << "Mostrando correspondencias Lado a Lado. Cierra para continuar..." << std::endl;

    while (!viewer_corr->wasStopped()) {
        viewer_corr->spinOnce(100);
    }
}