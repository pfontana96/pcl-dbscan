#include <DBSCANFilter.h>
namespace pcl_dbscan{

    DBSCANFilter::DBSCANFilter(float eps, int min)
    {
        this->eps_ = eps;
        this->min_ = min;
        std::cout << "exited ctor.." << std::endl;
    }

} // namespace pcl_dbscan