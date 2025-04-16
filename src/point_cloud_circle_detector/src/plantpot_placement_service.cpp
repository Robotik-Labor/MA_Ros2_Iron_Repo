#define PCL_NO_PRECOMPILE
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <common_services_package/srv/get_point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

using namespace std::chrono_literals;

class PointCloudServiceProcessor : public rclcpp::Node
{
public:
    PointCloudServiceProcessor() : Node("point_cloud_service_processor")
    {
        // Publishers
        pre_segmentation_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/UR5/pre_segmentation_point_cloud", 10);
        plantpot_coord_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/UR5/plantpot_coords", 10);

        // TF2 Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Planning Scene
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>("/UR5");

        // Client for service
        cloud_service_client_ = this->create_client<common_services_package::srv::GetPointCloud2>(
            "/UR5/service/pointcloud_data");

	cloud_message_publisher_ = create_publisher<std_msgs::msg::String>("/UR5/cloud_messages", 10);

        // Timer to initiate service call
        timer_ = create_wall_timer(1s, std::bind(&PointCloudServiceProcessor::call_service, this));

        RCLCPP_INFO(this->get_logger(), "Node ready, waiting for service response...");
    }

private:
    void call_service()
    {
        if (!cloud_service_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Service not available yet...");
            return;
        }

        auto request = std::make_shared<common_services_package::srv::GetPointCloud2::Request>();

        cloud_service_client_->async_send_request(request,
            std::bind(&PointCloudServiceProcessor::on_service_response, this, std::placeholders::_1));
        timer_->cancel(); // stop calling again
    }

void on_service_response(rclcpp::Client<common_services_package::srv::GetPointCloud2>::SharedFuture future)
{
    auto response = future.get();
    const auto &pointcloud_msg = response->cloud;

    // Process the point cloud
    process_point_cloud(pointcloud_msg);
}

    Eigen::Matrix4f getCameraToWorldTransform()
    {
        try
        {
            auto transform_stamped = tf_buffer_->lookupTransform("world", "UR5_camera_color_optical_frame", tf2::TimePointZero, 1s);
            auto tf_eigen = tf2::transformToEigen(transform_stamped.transform);
            return tf_eigen.matrix().cast<float>();
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", e.what());
            return Eigen::Matrix4f::Identity();
        }
    }

    geometry_msgs::msg::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q.x(); q_msg.y = q.y(); q_msg.z = q.z(); q_msg.w = q.w();
        return q_msg;
    }

    moveit_msgs::msg::CollisionObject createMeshObject(const std::string &id, const std::string &path, const geometry_msgs::msg::Pose &pose)
    {
        moveit_msgs::msg::CollisionObject obj;
        obj.id = id;
        obj.header.frame_id = "world";

        shapes::Mesh *mesh = shapes::createMeshFromResource(path);
        if (!mesh)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mesh from %s", path.c_str());
            return obj;
        }

        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(mesh, mesh_msg);
        obj.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));
        obj.mesh_poses.push_back(pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;

        delete mesh;
        return obj;
    }

    void process_point_cloud(const sensor_msgs::msg::PointCloud2& pointcloud_msg)
    {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
 	pcl::fromROSMsg(pointcloud_msg, *cloud);

        // Downsample
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(0.01f, 0.01f, 0.01f);
        voxel.filter(*filtered);

        // Transform to world frame
        Eigen::Matrix4f tf = getCameraToWorldTransform();
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*filtered, *transformed, tf);

        // Filter by Z range
        pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        for (auto &pt : transformed->points)
        {
            if (pt.z >= 0.03 && pt.z <= 0.16)
                z_filtered->points.push_back(pt);
        }

        // Find point closest to camera (min radius)
        pcl::PointXYZ min_point;
        float min_radius = std::numeric_limits<float>::max();
        for (auto &pt : z_filtered->points)
        {
            float r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (r < min_radius)
            {
                min_radius = r;
                min_point = pt;
            }
        }

        float angle = std::atan2(min_point.y, min_point.x);
        float new_radius = min_radius + 0.07;
        pcl::PointXYZ moved;
        moved.x = new_radius * std::cos(angle);
        moved.y = new_radius * std::sin(angle);
        moved.z = -0.01399;

        geometry_msgs::msg::Pose pose;
        pose.position.x = moved.x;
        pose.position.y = moved.y;
        pose.position.z = moved.z;
        pose.orientation = rpy_to_quaternion(0, 0, 0);

        auto obj = createMeshObject("Plantpot", "package://point_cloud_circle_detector/pcd/14_cm_Topf.stl", pose);
        planning_scene_interface_->addCollisionObjects({obj});

        // Publish point
        geometry_msgs::msg::Point pt;
        pt.x = moved.x;
        pt.y = moved.y;
        pt.z = moved.z;
        plantpot_coord_publisher_->publish(pt);

        // Publish debug cloud
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*transformed, out_msg);
        out_msg.header.frame_id = "world";
        pre_segmentation_publisher_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(), "Processing complete. Published target position and point cloud.");
	
	std_msgs::msg::String msg;
  	msg.data = "3D_Model_of_Plantpot_placed";
  	cloud_message_publisher_->publish(msg);        
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        rclcpp::shutdown();
    }

    rclcpp::Client<common_services_package::srv::GetPointCloud2>::SharedPtr cloud_service_client_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_segmentation_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr plantpot_coord_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cloud_message_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
   

};

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the node
    rclcpp::spin(std::make_shared<PointCloudServiceProcessor>());

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}

