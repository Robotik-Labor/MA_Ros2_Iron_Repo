// occupancy_map_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shape_operations.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <std_srvs/srv/empty.hpp>  // For the clear service
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/convex_hull.h>

class OccupancyMapNode : public rclcpp::Node
{
public:
  OccupancyMapNode() : Node("occupancy_map_node")
  {
    // Parameters
    this->declare_parameter("pointcloud_topic", "/UR5/UR5_camera/depth/color/points/green_filtered");
    this->declare_parameter("frame_id", "world");
    this->declare_parameter("resolution", 0.005);  // Octomap resolution in meters
    this->declare_parameter("update_rate", 1.0);  // How often to publish updates in Hz
    this->declare_parameter("transform_timeout", 0.1);  // TF transform timeout in seconds
    
    std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    resolution_ = this->get_parameter("resolution").as_double();
    transform_timeout_ = this->get_parameter("transform_timeout").as_double();
    
    // Initialize octomap
    octomap_ = std::make_shared<octomap::OcTree>(resolution_);
    
    // Create MoveIt2 Planning Scene Interface
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>("/UR5");
        
    // Set up TF2 listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create publisher for planning scene
    //planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
    
    // Subscribe to PointCloud2 messages
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10, 
      std::bind(&OccupancyMapNode::pointCloudCallback, this, std::placeholders::_1));
    
    // Set up a timer for periodic occupancy map updates
    double update_rate = this->get_parameter("update_rate").as_double();
    update_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0/update_rate),
      std::bind(&OccupancyMapNode::updatePlanningScene, this));
    
    // Set up service for clearing the octomap
    clear_service_ = this->create_service<std_srvs::srv::Empty>(
      "~/clear_octomap", 
      std::bind(&OccupancyMapNode::clearOctomapCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Occupancy Map Node initialized, listening to pointcloud topic: %s", 
                pointcloud_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Clear service available at: %s", 
                this->get_name() + std::string("/clear_octomap"));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
        RCLCPP_INFO(this->get_logger(), "Received pointcloud with %u points in frame %s", 
                msg->width * msg->height, msg->header.frame_id.c_str());
         
         // Convert incoming ROS2 message to PCL
	pcl::PCLPointCloud2 pcl_pc2_raw;
	pcl_conversions::toPCL(*msg, pcl_pc2_raw);
	pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2_raw, *raw_cloud);

	// Filter: remove all points with z < 0.02 meters (2 cm)
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (const auto& point : raw_cloud->points) {
	    if (point.z >= 0.02) {
		filtered_cloud->points.push_back(point);
	    }
	}
	filtered_cloud->width = filtered_cloud->points.size();
	filtered_cloud->height = 1;
	filtered_cloud->is_dense = true;

	// Convert back to sensor_msgs::msg::PointCloud2
	sensor_msgs::msg::PointCloud2::SharedPtr filtered_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
	pcl::toROSMsg(*filtered_cloud, *filtered_msg);
	filtered_msg->header = msg->header;
		 
         
         
                
    
        // Transform pointcloud to octomap frame
    sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud;
    
        try {
      // Check if we need to transform
      if (filtered_msg->header.frame_id != frame_id_) {
        RCLCPP_INFO(this->get_logger(), "Transforming pointcloud from %s to %s", 
                    filtered_msg->header.frame_id.c_str(), frame_id_.c_str());
        
        // Wait for transform to be available
        if (!tf_buffer_->canTransform(frame_id_, filtered_msg->header.frame_id, tf2::TimePointZero, 
                                    tf2::durationFromSec(transform_timeout_))) {
          RCLCPP_WARN(this->get_logger(), 
                    "Cannot transform pointcloud from %s to %s, skipping", 
                    filtered_msg->header.frame_id.c_str(), frame_id_.c_str());
          return;
        }
        
        // Transform the pointcloud
        transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        tf2::doTransform(*filtered_msg, *transformed_cloud, 
                        tf_buffer_->lookupTransform(frame_id_, filtered_msg->header.frame_id, tf2::TimePointZero));
        filtered_msg->header.frame_id = frame_id_;
        RCLCPP_INFO(this->get_logger(), "Pointcloud transformed successfully");
      } else {
        // No transformation needed
        transformed_cloud = filtered_msg;
      }  
    
    
    // Convert to PCL pointcloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*transformed_cloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr range_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Iterate through the original cloud
    for (const auto& point : cloud->points) {
	    if (point.y <= 1.0 && point.z >= 0.12) {
		range_filtered_cloud->points.push_back(point);
	    }
	}
    
    RCLCPP_INFO(this->get_logger(),"After range filtering: %zu points", range_filtered_cloud->points.size());
    
    // Optional: Downsample pointcloud for efficiency
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(range_filtered_cloud);
    voxel_filter.setLeafSize(resolution_, resolution_, resolution_);
    voxel_filter.filter(*cloud_filtered);
    
    RCLCPP_INFO(this->get_logger(),"After voxel grid filtering: %zu points", cloud_filtered->points.size());

	// Add Statistical Outlier Removal
	pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_filtered);
	sor.setMeanK(50);  // Number of neighbors to analyze
	sor.setStddevMulThresh(1.0);  // Standard deviation threshold
	sor.filter(*outlier_filtered_cloud);
   
   RCLCPP_INFO(this->get_logger(),"After outlier removal: %zu points", outlier_filtered_cloud->points.size());
	// Create a convex hull
	pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> hull;
	hull.setInputCloud(outlier_filtered_cloud);
	hull.setDimension(3);  // 3D convex hull
	hull.reconstruct(*hull_cloud);
   RCLCPP_INFO(this->get_logger(),"After Convex Hull: %zu points", hull_cloud->points.size());
    
    
    // Update octomap with pointcloud data
    updateOctomap(outlier_filtered_cloud, filtered_msg->header.frame_id);
  } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform pointcloud: %s", ex.what());
      } catch (std::exception &ex) {
      RCLCPP_ERROR(this->get_logger(), "Error processing pointcloud: %s", ex.what());
    }
  }

  void updateOctomap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& frame_id)
  {
    // Lock to prevent concurrent access to octomap
    std::lock_guard<std::mutex> lock(octomap_mutex_);
    
    // Insert points into octomap
    for (const auto& point : cloud->points)
    {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
      {
        octomap_->updateNode(octomap::point3d(point.x, point.y, point.z), true);
      }
    }
    
    // Update inner occupancy and prune the tree
    octomap_->updateInnerOccupancy();
    octomap_->prune();
    
    // Save the frame_id for later use
    if (!frame_id.empty())
    {
      last_frame_id_ = frame_id;
    }
    
    RCLCPP_INFO(this->get_logger(), "Octomap updated, now has %lu nodes", octomap_->size());
  }

  void updatePlanningScene()
  {
    std::lock_guard<std::mutex> lock(octomap_mutex_);
    
    if (octomap_->size() == 0)
    {
      RCLCPP_INFO(this->get_logger(), "Octomap is empty, skipping planning scene update");
      return;
    }
    
    // Create planning scene message
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    
    // Create a collision object for the octomap
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = !last_frame_id_.empty() ? last_frame_id_ : frame_id_;
    collision_object.header.stamp = this->now();
    collision_object.id = "occupancy_map";
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    
    // Convert octomap to collision object
    octomap_msgs::msg::Octomap octomap_msg;
    octomap_msg.header.frame_id = collision_object.header.frame_id;
    octomap_msg.header.stamp = collision_object.header.stamp;
    octomap_msgs::binaryMapToMsg(*octomap_, octomap_msg);
    
    // Add octomap to world object
    planning_scene.world.octomap.header = octomap_msg.header;
    planning_scene.world.octomap.octomap = octomap_msg;
    
    // Publish the planning scene
    planning_scene_interface_->applyPlanningScene(planning_scene);
    
    RCLCPP_INFO(this->get_logger(), "Published updated occupancy map to planning scene");
  }

  // Service callback to clear the octomap
  void clearOctomapCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    clearOctomap();
    RCLCPP_INFO(this->get_logger(), "Octomap cleared via service call");
  }

  // Method to clear the octomap and update the planning scene
  void clearOctomap()
  {
    std::lock_guard<std::mutex> lock(octomap_mutex_);
    
    // Create a new empty octomap with the same resolution
    octomap_ = std::make_shared<octomap::OcTree>(resolution_);
    
    // Create a planning scene message to remove the occupancy map
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    
    // Create a collision object with REMOVE operation
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = !last_frame_id_.empty() ? last_frame_id_ : frame_id_;
    collision_object.header.stamp = this->now();
    collision_object.id = "occupancy_map";
    collision_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    
    // Add the collision object to the planning scene
    planning_scene.world.collision_objects.push_back(collision_object);
    
    // Publish the planning scene to remove the occupancy map
    planning_scene_interface_->applyPlanningScene(planning_scene);
    
    RCLCPP_INFO(this->get_logger(), "Cleared octomap and removed from planning scene");
  }

  // Subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  //rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_service_;
  
  // MoveIt planning scene interface
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  
  
    // TF2 components
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Octomap data
  std::shared_ptr<octomap::OcTree> octomap_;
  std::mutex octomap_mutex_;
  
  // Parameters
  std::string frame_id_;
  std::string last_frame_id_;
  double resolution_;
  double transform_timeout_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OccupancyMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
