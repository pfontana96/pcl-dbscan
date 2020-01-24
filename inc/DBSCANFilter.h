#ifndef PCL_DBSCAN_H
#define PCL_DBSCAN_H

#include <cstdlib>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/kdtree/kdtree_flann.h>

struct PointXYZId {
        PCL_ADD_POINT4D;
        int id;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(  PointXYZId,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (int, id, id))

namespace pcl_dbscan{

    class DBSCANFilter{
        public:
                        
            DBSCANFilter(float eps, int min);
            void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
            void filter(pcl::PointCloud<PointXYZId> &cloud_out);

        private:
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_ptr_;
            pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
            std::vector<bool> visited_;
            std::vector<PointXYZId> output_;

            int min_;
            float eps_;
            int cluster_;

            void expandCluster(PointXYZId p, std::vector<int> neighbors_ids);
    };
} //namespace pcl_dbscan

#endif
