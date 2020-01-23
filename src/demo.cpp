#include <DBSCANFilter.h>

#include <iostream>
#include <cstdlib>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    pcl_dbscan::DBSCANFilter dbscan(0.2, 4);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if( pcl::io::loadPCDFile<pcl::PointXYZ> ("../input/example.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file ../input/example.pcd");
        return (-1);
    }
    std::cout   << "Loaded "
                << cloud->width * cloud->height
                << " data points from ../input/example.pcd"
                << std::endl;

    return 0;
}
