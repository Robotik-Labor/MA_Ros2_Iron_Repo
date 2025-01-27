#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/ppf.h>
#include <pcl/filters/voxel_grid.h>  // Include the VoxelGrid filter header
#include <iostream>

int main()
{
    // Create a PointCloud of type pcl::PointXYZ (3D points)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Load a sample PCD file (replace this with your own file)
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/buhrmann/ws_moveit/install/point_cloud_circle_detector/share/point_cloud_circle_detector/pcd/plantpod_lesssamples.pcd", *cloud) == -1) 
    {
        PCL_ERROR("Couldn't read the file\n");
        return -1;
    }

    // Downsampling to lower resolution using VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.02f, 0.02f, 0.02f);  // Set the voxel grid leaf size (downsampling resolution)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    vg.filter(*cloud_filtered);

    // Save the filtered point cloud to a PCD file
    if (pcl::io::savePCDFileASCII("/home/buhrmann/Desktop/Pointcloud_Tests/filtered_cloud.pcd", *cloud_filtered) == -1) {
        PCL_ERROR("Couldn't write the filtered PCD file\n");
        return -1;
    }

    std::cout << "Filtered point cloud saved to /home/buhrmann/Desktop/Pointcloud_Tests/filtered_cloud.pcd" << std::endl;

    // Create a PointCloud for normals (pcl::PointNormal includes both position and normal)
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

    // Estimate normals using pcl::NormalEstimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud(cloud_filtered);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.005);  // Adjust this based on your point cloud density
    ne.compute(*cloud_with_normals);  // Output will be cloud_with_normals

    // Create the PPF feature estimation object
    pcl::PPFEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PPFSignature> ppf_estimator;

    // Set the input point cloud and normals
    ppf_estimator.setInputCloud(cloud_filtered);
    ppf_estimator.setInputNormals(cloud_with_normals);  // Providing the normal information

    // Compute the PPF features
    pcl::PointCloud<pcl::PPFSignature>::Ptr ppf_features(new pcl::PointCloud<pcl::PPFSignature>());
    ppf_estimator.compute(*ppf_features);

    // Print the computed features
    std::cout << "Number of PPF features: " << ppf_features->points.size() << std::endl;

    // Save the computed PPF features to a PCD file (ASCII format)
    if (pcl::io::savePCDFileASCII("/home/buhrmann/Desktop/Pointcloud_Tests/ppf_features.pcd", *ppf_features) == -1) {
        PCL_ERROR("Couldn't write the PCD file\n");
        return -1;
    }

    std::cout << "PPF features saved to /home/buhrmann/Desktop/Pointcloud_Tests/ppf_features.pcd" << std::endl;

    return 0;
}

