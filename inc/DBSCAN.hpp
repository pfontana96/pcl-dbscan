#ifndef PCL_DBSCAN_H
#define PCL_DBSCAN_H

#include <cstdlib>
#include <iostream>

namespace pcl_dbscan{
    class DBSCANFilter{
        public:
            

            DBSCANFilter(float eps, int min);
            ~DBSCANFilter();

        private:

            int min_;
            float eps_;
    };
} //namespace pcl_dbscan

#endif
