#include <DBSCANFilter.h>

namespace pcl_dbscan{

    DBSCANFilter::DBSCANFilter(float eps, int min)
    {
        this->eps_ = eps;
        this->min_ = min;
    }

    void DBSCANFilter::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
    {
        cloud_in_ptr_ = cloud_in;
        cluster_ = -1;
        visited_.resize(cloud_in->size(), false);
        output_.reserve(cloud_in->size());
        tree_.setInputCloud(cloud_in);
    }

    void DBSCANFilter::filter(pcl::PointCloud<PointXYZId> &cloud_out)
    {
        std::vector<bool>::iterator it_vis = visited_.begin();
        for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_in_ptr_->begin();
            it != cloud_in_ptr_->end(); ++it, ++it_vis)
        {
            PointXYZId point{it->x, it->y, it->z, -2};

            std::vector<int> indices;
            std::vector<float> distances;
            *it_vis = true;
            if( tree_.radiusSearch(*it, eps_, indices, distances) < min_)
            {
                point.id = -1; // Noise Point
                output_.push_back(point);
            }else{
                cluster_++;
                expandCluster(point, indices);
            }
        }

        cloud_out.points = output_;
    
    }

    void DBSCANFilter::expandCluster(PointXYZId p, std::vector<int> neighbors_ids)
    {
        p.id = cluster_;
        output_.push_back(p);
        for( std::vector<int>::iterator it = neighbors_ids.begin(); 
            it != neighbors_ids.end(); ++it)
        {
            PointXYZId p_1 {cloud_in_ptr_->points[*it].x,
                            cloud_in_ptr_->points[*it].y,
                            cloud_in_ptr_->points[*it].z,
                            -2};
            if(!visited_[*it])
            {
                visited_[*it] = true;

                std::vector<int> indices;
                std::vector<float> distances;
                if( tree_.radiusSearch(*it, eps_, indices, distances) >= min_)
                {
                    neighbors_ids.insert(std::end(neighbors_ids), std::begin(indices), std::end(indices));
                }
            }
            if(p_1.id == -2)
            {
                p_1.id = cluster_;
                output_.push_back(p_1);
            }
        }
    }

} // namespace pcl_dbscan