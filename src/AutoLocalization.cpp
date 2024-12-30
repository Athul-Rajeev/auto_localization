#include "auto_localization/AutoLocalization.hpp"

AutoLocalization::AutoLocalization()
: Node("auto_localization"),
  m_poseInitialized(false),
  m_db(nullptr)
{
    // Declare ROS parameters with default values
    this->declare_parameter<std::string>("database_file", "data/pose.db");
    this->declare_parameter<std::string>("pose_topic", "/amcl_pose");

    // Get the parameters
    std::string relativeDbFilePath;
    std::string poseTopic;
    this->get_parameter("database_file", relativeDbFilePath);
    this->get_parameter("pose_topic", poseTopic);

    // Resolve the database file path
    std::string packageShareDirectory = ament_index_cpp::get_package_share_directory("auto_localization");
    m_dbFilePath = packageShareDirectory + "/" + relativeDbFilePath;

    RCLCPP_INFO(this->get_logger(), "Database file path: %s", m_dbFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Pose topic: %s", poseTopic.c_str());

    // Open the database connection
    openDatabase();

    // Load the last known pose from the database
    loadLastKnownPose();

    // Create subscriber for the pose topic
    m_poseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        poseTopic, 10,
        std::bind(&AutoLocalization::poseCallback, this, std::placeholders::_1)
    );

    // Create publisher for the initial pose
    m_initialPosePub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10
    );

    // If we have a stored pose, publish it once for initial localization
    if (m_poseInitialized) {
        for (size_t i = 0; i < 36; i++) {
            m_lastKnownPose.pose.covariance[i] = 0.0;
        }
        m_lastKnownPose.pose.covariance[0] = 0.5;  // x
        m_lastKnownPose.pose.covariance[7] = 0.5;  // y
        m_lastKnownPose.pose.covariance[35] = 0.5; // yaw

        rclcpp::sleep_for(std::chrono::seconds(1));
        publishInitialPose();
    }

    RCLCPP_INFO(this->get_logger(), "AutoLocalization node started.");
}

AutoLocalization::~AutoLocalization()
{
    if (m_db)
    {
        sqlite3_close(m_db);
        RCLCPP_INFO(this->get_logger(), "Database connection closed.");
    }
}

void AutoLocalization::openDatabase()
{
    // Open the SQLite3 database
    RCLCPP_INFO(this->get_logger(), "Opening database");
    if (sqlite3_open(m_dbFilePath.c_str(), &m_db) != SQLITE_OK)
    {
        throw std::runtime_error("Failed to open the SQLite3 database.");
    }
}

void AutoLocalization::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // Store the latest pose in the database
    m_lastKnownPose = *msg;
    savePoseToDb(m_lastKnownPose);
}

void AutoLocalization::publishInitialPose()
{
    RCLCPP_INFO(this->get_logger(), "Publishing initial pose for localization: [%.2f, %.2f]",
                m_lastKnownPose.pose.pose.position.x,
                m_lastKnownPose.pose.pose.position.y);

    // Update timestamp and frame_id
    m_lastKnownPose.header.stamp = this->now();
    m_lastKnownPose.header.frame_id = "map";

    // Publish the initial pose
    m_initialPosePub->publish(m_lastKnownPose);
}

void AutoLocalization::loadLastKnownPose()
{
    const char *query = R"(
        SELECT x, y, z, qx, qy, qz, qw FROM robot_pose WHERE rowid = 1;
    )";

    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, query, -1, &stmt, nullptr) != SQLITE_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to prepare SELECT statement.");
        return;
    }

    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
        m_lastKnownPose.pose.pose.position.x = sqlite3_column_double(stmt, 0);
        m_lastKnownPose.pose.pose.position.y = sqlite3_column_double(stmt, 1);
        m_lastKnownPose.pose.pose.position.z = sqlite3_column_double(stmt, 2);
        m_lastKnownPose.pose.pose.orientation.x = sqlite3_column_double(stmt, 3);
        m_lastKnownPose.pose.pose.orientation.y = sqlite3_column_double(stmt, 4);
        m_lastKnownPose.pose.pose.orientation.z = sqlite3_column_double(stmt, 5);
        m_lastKnownPose.pose.pose.orientation.w = sqlite3_column_double(stmt, 6);

        m_lastKnownPose.header.frame_id = "map";
        m_poseInitialized = true;
        
        RCLCPP_INFO(this->get_logger(), "Loaded last known pose from DB: [%.2f, %.2f]",
                    m_lastKnownPose.pose.pose.position.x,
                    m_lastKnownPose.pose.pose.position.y);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "No pose found in database. Starting fresh.");
    }

    sqlite3_finalize(stmt);
}

void AutoLocalization::savePoseToDb(const geometry_msgs::msg::PoseWithCovarianceStamped &pose)
{
    const char *replaceQuery = R"(
        INSERT OR REPLACE INTO robot_pose (rowid, x, y, z, qx, qy, qz, qw, timestamp)
        VALUES (1, ?, ?, ?, ?, ?, ?, ?, datetime('now'));
    )";

    sqlite3_stmt *stmt = nullptr;
    if (sqlite3_prepare_v2(m_db, replaceQuery, -1, &stmt, nullptr) != SQLITE_OK)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to prepare INSERT statement.");
        return;
    }

    sqlite3_bind_double(stmt, 1, pose.pose.pose.position.x);
    sqlite3_bind_double(stmt, 2, pose.pose.pose.position.y);
    sqlite3_bind_double(stmt, 3, pose.pose.pose.position.z);
    sqlite3_bind_double(stmt, 4, pose.pose.pose.orientation.x);
    sqlite3_bind_double(stmt, 5, pose.pose.pose.orientation.y);
    sqlite3_bind_double(stmt, 6, pose.pose.pose.orientation.z);
    sqlite3_bind_double(stmt, 7, pose.pose.pose.orientation.w);

    if (sqlite3_step(stmt) != SQLITE_DONE)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute INSERT statement.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Saved pose to DB: [%.2f, %.2f]",
                    pose.pose.pose.position.x,
                    pose.pose.pose.position.y);
    }

    sqlite3_finalize(stmt);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoLocalization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}