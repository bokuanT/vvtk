#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

extern "C" {
    void calculate_normals(const float* points, size_t num_points, float* normals) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Convert the input points to PCL point cloud
        for (size_t i = 0; i < num_points; ++i) {
            pcl::PointXYZ point(points[i * 3], points[i * 3 + 1], points[i * 3 + 2]);
            cloud->push_back(point);
        }
        
        // Create the normal estimation object
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        
        // Create a KD-tree for search
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setSearchMethod(tree);
        
        // Output dataset
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        
        // Set the radius for normal estimation
        ne.setRadiusSearch(0.03); // Adjust the radius as per your point cloud density
        
        // Compute the normals
        ne.compute(*cloud_normals);
        
        // Convert the PCL normals to the output format
        for (size_t i = 0; i < cloud_normals->size(); ++i) {
            normals[i * 3] = cloud_normals->at(i).normal_x;
            normals[i * 3 + 1] = cloud_normals->at(i).normal_y;
            normals[i * 3 + 2] = cloud_normals->at(i).normal_z;
        }
    }
}
