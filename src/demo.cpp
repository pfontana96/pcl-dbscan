#include <DBSCANFilter.h>

#include <iostream>
#include <cstdlib>
#include <thread>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cout << "Usage: $" << argv[0] << " <PCD INPUT>" << std::endl;
        return -1;
    }

    pcl_dbscan::DBSCANFilter dbscan(0.2, 4);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if( pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ..");
        return -1;
    }
    std::cout   << "Loaded "
                << cloud->width * cloud->height
                << " data points from "
                << argv[1]
                << std::endl;

    pcl::PointCloud<PointXYZId> cloud_filtered;
    pcl::PointCloud<PointXYZId> cloud_filtered_ptr(cloud_filtered);
    
    dbscan.setInputCloud(cloud);
    dbscan.filter(cloud_filtered);

    std::cout << "Filtered cloud points: " << cloud_filtered.points.size() << std::endl;

    // Visualization
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    // viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    std::chrono::milliseconds period(100);

    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(period);
    }

    return 0;
}
