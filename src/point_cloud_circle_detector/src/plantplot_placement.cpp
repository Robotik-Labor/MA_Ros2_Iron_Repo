#define PCL_NO_PRECOMPILE

#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfh_tools.h>
#include <pcl/filters/voxel_grid.h>  // Include Voxel Grid filter
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS2 Publisher for PointCloud2
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_segmentation_publisher_;  // New publisher for pre-segmentation point cloud
rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr plantpot_coord_publisher_; 
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;



// Define a constant to toggle the orientation transformation
const bool apply_orientation_override = false;  // Set to false to disable


class PointCloudCustomObjectMatcher : public rclcpp::Node
{
public:
    PointCloudCustomObjectMatcher()
        : Node("point_cloud_custom_object_matcher")
    {	
    
        
        // Publisher to publish PointCloud2 data with Reliable QoS
        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/detected_object_point_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        // Publisher for the pre-segmentation (filtered) point cloud
        pre_segmentation_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/pre_segmentation_point_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
            
        // Publisher for the Placed Plantpot Coordianates
        plantpot_coord_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/UR5/plantpot_coords", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

        // Create subscription to PointCloud2
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/UR5/plantpod_reduced_pointcloud", 1,
             std::bind(&PointCloudCustomObjectMatcher::listener_callback, this, std::placeholders::_1));
	
		// Initialize in constructor or init function
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create the PlanningSceneInterface for the specific robot namespace
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>("/UR5");

        RCLCPP_INFO(this->get_logger(), "PointCloud Custom Object Matcher Node has been started.");


    }


private:

 geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();
    return q_msg;
  }
  
   // Helper function to create a mesh collision object
 moveit_msgs::msg::CollisionObject createMeshObject(
      const std::string& id, 
      const std::string& mesh_path, 
      const geometry_msgs::msg::Pose& pose)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = id;
    collision_object.header.frame_id = "world";
    
    // Load the mesh from the STL file
    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path);
    if (mesh) {
      // Convert the shapes::Mesh to shape_msgs::Mesh
      shapes::ShapeMsg mesh_msg;
      shapes::constructMsgFromShape(mesh, mesh_msg);
      
      // Add the mesh to the collision object
      collision_object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));
      collision_object.mesh_poses.push_back(pose);
      collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
      
      delete mesh;  // Clean up the mesh object
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from %s", mesh_path.c_str());
    }
    
    return collision_object;
  }

Eigen::Matrix4f getCameraToWorldTransform()
{
    try
    {
        // Look up transform from camera frame to world frame
        geometry_msgs::msg::TransformStamped transform_stamped = 
            tf_buffer_->lookupTransform("world", "UR5_camera_color_optical_frame", rclcpp::Time(0), std::chrono::seconds(1));
        
        // Convert to Eigen::Matrix4f
        Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform_stamped.transform);
        Eigen::Matrix4f camera_to_world = transform_eigen.matrix().cast<float>();
        
        RCLCPP_INFO(this->get_logger(), "Got transform from camera to world");
        return camera_to_world;
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not transform from %s to %s: %s",
                    "UR5_camera_color_optical_frame", "world", ex.what());
        
        // Return identity matrix in case of failure
        return Eigen::Matrix4f::Identity();
    }
}

void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
	    RCLCPP_INFO(this->get_logger(), "PointCloud2 message received");

	    // Declare time variables for timing different steps
	    auto start_time = std::chrono::high_resolution_clock::now();
	    auto end_time = std::chrono::high_resolution_clock::now();
	    std::chrono::duration<double> duration;

	    // Step 1: Convert PointCloud2 message to PCL PointCloud
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::fromROSMsg(*msg, *cloud);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "PointCloud2 to PCL conversion took %f ms", duration.count());
  
	        	   
	    // Step 2: Convert PointCloudXYZRGB to PointCloudXYZ
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	    for (const auto& point : cloud->points)
	    {
		pcl::PointXYZ point_xyz;
		point_xyz.x = point.x;
		point_xyz.y = point.y;
		point_xyz.z = point.z;
		cloud_filtered_xyz->points.push_back(point_xyz);
	    }
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Conversion to PointCloudXYZ took %f ms", duration.count());
	    
	    // Step 5: Apply voxel grid filter for downsampling
	    start_time = std::chrono::high_resolution_clock::now(); // Start timing
	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
	    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	    voxel_grid.setInputCloud(cloud_filtered_xyz);
	    voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);
	    voxel_grid.filter(*cloud_filtered_voxel);
	    end_time = std::chrono::high_resolution_clock::now(); // End timing
	    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	    RCLCPP_INFO(this->get_logger(), "Voxel grid filtering took %f ms", duration.count());
	    
   	    	    
	    Eigen::Matrix4f camera_to_world = getCameraToWorldTransform();
	    // Transform filtered cloud from camera frame to world frame
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_world(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*cloud_filtered_voxel, *cloud_filtered_world, camera_to_world);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZ>());

		// Iterate over the points in the original point cloud
		for (size_t i = 0; i < cloud_filtered_world->points.size(); ++i) {
		    float z = cloud_filtered_world->points[i].z;
		    
		    // Check if the Z value is between 3cm (0.03m) and 16cm (0.16m)
		    if (z >= 0.03f && z <= 0.16f) {
			// If the point meets the condition, add it to the filtered cloud
			cloud_filtered_z->points.push_back(cloud_filtered_world->points[i]);
		    }
		}
		
		pcl::PointXYZ min_radius_point;  // To store the point with the smallest radius
		float min_radius = std::numeric_limits<float>::max();  // Initialize with a large value

		// Iterate over the points in the cloud_filtered_z
		for (size_t i = 0; i < cloud_filtered_z->points.size(); ++i) {
		    float x = cloud_filtered_z->points[i].x;
		    float y = cloud_filtered_z->points[i].y;

		    // Calculate the radius (distance in the XY plane)
		    float radius = std::sqrt(x * x + y * y);

		    // Check if this radius is smaller than the current smallest radius
		    if (radius < min_radius) {
			min_radius = radius;
			min_radius_point = cloud_filtered_z->points[i];  // Store the point with the smallest radius
		    }
		}
	    
	    RCLCPP_INFO(this->get_logger(), "Point with smallest radius: (%f, %f, %f) with radius: %f",
			    min_radius_point.x, min_radius_point.y, min_radius_point.z, min_radius);
	    //float x = min_radius_point.x;
	    //float y = min_radius_point.y;
	    float angle;
	    angle = std::atan2(min_radius_point.y, min_radius_point.x);
	    
	    pcl::PointXYZ moved_point;
	    
	    
	    float new_radius = min_radius + 0.07f;  // Add 7cm to the radius
            moved_point.x = new_radius * std::cos(angle);
            moved_point.y = new_radius * std::sin(angle);
	    moved_point.z = -0.01399;
	    
	    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
	    geometry_msgs::msg::Pose object_pose;
	    object_pose.position.x = moved_point.x;
	    object_pose.position.y = moved_point.y;
	    object_pose.position.z = moved_point.z;
	      
	    double roll = 0.0;   // rotation around x-axis (90 degrees)
	    double pitch = 0.0;     // rotation around y-axis
	    double yaw = 0.0;    // rotation around z-axis (90 degrees)
	    object_pose.orientation = rpy_to_quaternion(roll, pitch, yaw);
	      
	    std::string stl_path = "package://point_cloud_circle_detector/pcd/14_cm_Topf.stl";
	      
	    moveit_msgs::msg::CollisionObject plantpot = createMeshObject("Plantpot", stl_path, object_pose);
	    collision_objects.push_back(plantpot);
	      
	    RCLCPP_INFO(this->get_logger(), "Added Sensor_Station with orientation: Roll=%f, Pitch=%f, Yaw=%f", roll, pitch, yaw);
	    
	    planning_scene_interface_->addCollisionObjects(collision_objects);
	    
	    sensor_msgs::msg::PointCloud2 output_pre_segmentation;
	    pcl::toROSMsg(*cloud_filtered_world, output_pre_segmentation);
	    output_pre_segmentation.header.stamp = this->get_clock()->now();
	    output_pre_segmentation.header.frame_id = "world"; // Ensure this matches your RViz fixed frame
	    pre_segmentation_publisher_->publish(output_pre_segmentation);
	    
	    auto point_msg = geometry_msgs::msg::Point();
	    point_msg.x = moved_point.x;
	    point_msg.y = moved_point.y;
	    point_msg.z = moved_point.z;
	    plantpot_coord_publisher_->publish(point_msg);
	    
	}
	



private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
};

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the node
    rclcpp::spin(std::make_shared<PointCloudCustomObjectMatcher>());

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}

