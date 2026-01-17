#define PCL_SILENCE_MALLOC_WARNING
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/impl/shot.hpp>

class PCL_Dummy { //Diese Dummy-Klasse umgeht Probleme beim Befuellen und Zeigen der Punktwolke
public:
    PCL_MAKE_ALIGNED_OPERATOR_NEW

    PCL_Dummy() : 
        cloud(new pcl::PointCloud<pcl::PointXYZ>),
        viewer(new pcl::visualization::PCLVisualizer("3D Viewer")) 
    {
        // Einmaliges Setup des Viewers
        viewer->setBackgroundColor(0.0, 0.0, 0.0);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
    }

    void addPoint(float x, float y, float z) {
        cloud->push_back(pcl::PointXYZ(x, y, z));
    }

    void show() {
        // Punktwolke initial registrieren (ID: "my_cloud")
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "my_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "my_cloud");

        // Hauptschleife: Hält das Fenster offen
        while (true) {
            // Falls sich die Daten ändern, muss die Anzeige aktualisiert werden
            viewer->updatePointCloud(cloud, "my_cloud");
            
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPtr() {
        return cloud; //"cloud" ist ein smarter Pointer, der auf die aktuelle Wolke zeigt
    }

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};

    // Typ-Definitionen zur besseren Lesbarkeit
typedef pcl::PointXYZ PointT;
typedef pcl::SHOT352 DescriptorT;

// Berechnung des SHOT-Deskriptors
void computeSHOT(pcl::PointCloud<PointT>::Ptr cloud, 
                 pcl::PointCloud<DescriptorT>::Ptr descriptors) {
    // 1. Normalen schätzen (zwingend erforderlich für SHOT)
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(10); // Nutze 10 Nachbarpunkte für die Normale
    ne.compute(*normals);

    // 2. SHOT Deskriptoren berechnen
    pcl::SHOTEstimation<PointT, pcl::Normal, DescriptorT> shot;
    shot.setInputCloud(cloud);
    shot.setInputNormals(normals);
    shot.setSearchMethod(tree);
    shot.setRadiusSearch(0.4); // Radius in Metern (je nach Skalierung anpassen)
    shot.compute(*descriptors);
}

//Unterschied ("negative Ähnlichkeit") zwischen zwei Punktwolken berechnen
double calc_diff_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
    
    pcl::PointCloud<DescriptorT>::Ptr desc1(new pcl::PointCloud<DescriptorT>);
    pcl::PointCloud<DescriptorT>::Ptr desc2(new pcl::PointCloud<DescriptorT>);

    computeSHOT(cloud1, desc1);
    computeSHOT(cloud2, desc2);

    // 3. Abstände (Unterschiede) berechnen
    pcl::registration::CorrespondenceEstimation<DescriptorT, DescriptorT> est;
    est.setInputSource(desc1);
    est.setInputTarget(desc2);

    pcl::Correspondences all_corrs;
    est.determineCorrespondences(all_corrs);

    // Durchschnittliche quadratische Distanz im Merkmalsraum ausgeben
    double sum_dist = 0;
    for (const auto& corr : all_corrs) {
        sum_dist += corr.distance;
    }
    return sum_dist;
}

int main() {

    float pi = std::acos(-1.0);
    float radius = .5;
    float hoehe = .5;
    
    // ********************************************
    // * Fuelle Punktwolke mit Punkten, so dass  *
    // * eine Kugel entsteht!                     *
    // ********************************************
    PCL_Dummy cl_kugel;
    for(float phi=0; phi<=pi/2; phi+=.1){
        radius = std::cos(phi)*.5;
        hoehe = std::sin(phi)*.5;
        for(float alpha=0; alpha<=2*pi; alpha+=.04*.5/radius){
            cl_kugel.addPoint(std::sin(alpha)*radius+.5,std::cos(alpha)*radius+.5,.5+hoehe);
            cl_kugel.addPoint(std::sin(alpha)*radius+.5,std::cos(alpha)*radius+.5,.5-hoehe);
        }
    }
    
    // ********************************************
    // * Fuelle Punktwolke mit Punkten, so dass  *
    // * ein Kegel entsteht!                     *
    // ********************************************
     PCL_Dummy cl_kegel;
    for(float coord=0; coord <=1.11803; coord+=.05) {
        hoehe = coord/1.11803;
        radius = (1-hoehe)/2;
        for(float alpha=0; alpha<=2*pi; alpha+=.04*.5/radius)
            cl_kegel.addPoint(std::sin(alpha)*radius+.5,std::cos(alpha)*radius+.5,hoehe);
    }
    
    // ********************************************
    // * Fuelle Punktwolke mit Punkten, so dass  *
    // * ein Zylinder entsteht!                     *
    // ********************************************
    PCL_Dummy cl_zylinder;
    radius=.5;
    for(float hoehe=0; hoehe <=1; hoehe+=.05) {
        for(float alpha=0; alpha<=2*pi; alpha+=.04*.5/radius)
            cl_zylinder.addPoint(std::sin(alpha)*radius+.5,std::cos(alpha)*radius+.5,hoehe);
    }
    
    std::cout << "Kugel -> Kegel:" << endl;
    std::cout << calc_diff_clouds(cl_kugel.getPtr(),cl_kegel.getPtr()) << endl;
    std::cout << "Kugel -> Zylinder:" << endl;
    std::cout << calc_diff_clouds(cl_kugel.getPtr(),cl_zylinder.getPtr()) << endl;
    std::cout << "Kegel -> Zylinder:" << endl;
    std::cout << calc_diff_clouds(cl_kegel.getPtr(),cl_zylinder.getPtr()) << endl;
    
    //cl_kugel.show();
    //cl_kegel.show();
    //cl_zylinder.show();
    int n;
    std::cin >> n;
    return 0;
}