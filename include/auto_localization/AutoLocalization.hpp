#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sqlite3.h>
#include <string>
#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdexcept>
#include <sstream>

class AutoLocalization : public rclcpp::Node
{
public:
    AutoLocalization();
    ~AutoLocalization();

private:
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void loadLastKnownPose();
    void savePoseToDb(const geometry_msgs::msg::PoseWithCovarianceStamped &pose);
    void openDatabase();
    void publishInitialPose();
    
    // Subscribers and Publishers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_poseSub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialPosePub;
    
    std::string m_dbFilePath;
    sqlite3 *m_db;  // SQLite3 database connection
    bool m_poseInitialized;
    geometry_msgs::msg::PoseWithCovarianceStamped m_lastKnownPose;
};