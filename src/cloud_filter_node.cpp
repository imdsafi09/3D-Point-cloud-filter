#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <cmath>

class CloudFilterNode : public rclcpp::Node
{
public:
  CloudFilterNode()
  : Node("cloud_filter_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    target_frame_ = "os_sensor";

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster/points", rclcpp::SensorDataQoS(),
      std::bind(&CloudFilterNode::cloud_callback, this, std::placeholders::_1));

    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/ouster/scan", rclcpp::SensorDataQoS(),
      std::bind(&CloudFilterNode::scan_callback, this, std::placeholders::_1));

    filtered_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);
    original_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/original_points", 10);
    scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/republished_scan", 10);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    last_scan_msg_ = msg;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 cloud_in_sensor_frame;

    try
    {
      tf_buffer_.transform(*msg, cloud_in_sensor_frame, target_frame_, tf2::durationFromSec(0.1));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                  msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    // Publish original transformed point cloud
    cloud_in_sensor_frame.header.frame_id = target_frame_;
    original_publisher_->publish(cloud_in_sensor_frame);

    // Republish scan with synchronized timestamp
    if (last_scan_msg_)
    {
      auto synced_scan = *last_scan_msg_;
      synced_scan.header.stamp = msg->header.stamp;
      scan_publisher_->publish(synced_scan);
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(cloud_in_sensor_frame, *input);

    // Filter parameters
    constexpr float x_min = 0.0f;
    constexpr float x_max = 8.0f;
    constexpr float z_min = -1.0f;
    constexpr float z_max = 2.0f;
    constexpr float tan_h_fov = std::tan(M_PI / 4);  // ±45°
    constexpr float tan_v_fov = std::tan(M_PI / 6);  // ±30°

    pcl::PointCloud<pcl::PointXYZI>::Ptr fov_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    fov_filtered->reserve(input->size());

    for (const auto& point : input->points)
    {
      if (point.x < x_min || point.x > x_max) continue;

      float inv_x = 1.0f / point.x;
      if (std::abs(point.y * inv_x) > tan_h_fov) continue;
      if (std::abs(point.z * inv_x) > tan_v_fov) continue;
      if (point.z < z_min || point.z > z_max) continue;

      fov_filtered->emplace_back(point);
    }

    // Apply voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(fov_filtered);
    voxel_filter.setLeafSize(0.03f, 0.03f, 0.03f);  // Adjust as needed

    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_filter.filter(*voxel_filtered);

    // Convert and publish
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*voxel_filtered, out_msg);
    out_msg.header.stamp = this->get_clock()->now();
    out_msg.header.frame_id = target_frame_;
    filtered_publisher_->publish(out_msg);

    RCLCPP_INFO(this->get_logger(), "Filtered %zu -> %zu points", input->points.size(), voxel_filtered->points.size());
  }

  std::string target_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr original_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;

  sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudFilterNode>());
  rclcpp::shutdown();
  return 0;
}

