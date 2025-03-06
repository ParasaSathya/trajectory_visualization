#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <jsoncpp/json/json.h>

class TrajectoryReader {
private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    
    std::vector<geometry_msgs::PoseStamped> trajectory_points_;
    
    // Parameters
    std::string trajectory_file_;
    std::string source_frame_;
    std::string target_frame_;
    double marker_lifetime_;
    double publish_rate_;
    
    // TF2 for transformations
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    bool loadTrajectoryFromFile() {
        trajectory_points_.clear();
        
        // Check if file exists
        std::ifstream file(trajectory_file_);
        if (!file.is_open()) {
            ROS_ERROR("Could not open trajectory file: %s", trajectory_file_.c_str());
            return false;
        }
        file.close();
        
        // Determine file format from extension
        std::string extension = trajectory_file_.substr(trajectory_file_.find_last_of(".") + 1);
        
        try {
            if (extension == "json") {
                return loadFromJson();
            } else if (extension == "yaml" || extension == "yml") {
                return loadFromYaml();
            } else if (extension == "csv") {
                return loadFromCsv();
            } else {
                ROS_ERROR("Unsupported file format: %s", extension.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Error loading trajectory: %s", e.what());
            return false;
        }
    }
    
    bool loadFromJson() {
        std::ifstream file(trajectory_file_);
        Json::Value root;
        Json::CharReaderBuilder reader;
        std::string errors;
        
        bool parsingSuccessful = Json::parseFromStream(reader, file, &root, &errors);
        if (!parsingSuccessful) {
            ROS_ERROR("Failed to parse JSON: %s", errors.c_str());
            return false;
        }
        
        // Get frame_id from file or use default
        if (root.isMember("frame_id")) {
            source_frame_ = root["frame_id"].asString();
        }
        
        if (!root.isMember("trajectory") || !root["trajectory"].isArray()) {
            ROS_ERROR("Invalid JSON format: missing trajectory array");
            return false;
        }
        
        const Json::Value& trajectory = root["trajectory"];
        for (unsigned int i = 0; i < trajectory.size(); i++) {
            const Json::Value& point = trajectory[i];
            
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = source_frame_;
            
            if (point.isMember("timestamp")) {
                pose.header.stamp.fromSec(point["timestamp"].asDouble());
            } else {
                pose.header.stamp = ros::Time::now();
            }
            
            pose.pose.position.x = point["x"].asDouble();
            pose.pose.position.y = point["y"].asDouble();
            pose.pose.position.z = point["z"].asDouble();
            pose.pose.orientation.x = point["qx"].asDouble();
            pose.pose.orientation.y = point["qy"].asDouble();
            pose.pose.orientation.z = point["qz"].asDouble();
            pose.pose.orientation.w = point["qw"].asDouble();
            
            trajectory_points_.push_back(pose);
        }
        
        ROS_INFO("Loaded %zu trajectory points from JSON file: %s", 
                 trajectory_points_.size(), trajectory_file_.c_str());
        return true;
    }
    
    bool loadFromYaml() {
        YAML::Node config;
        try {
            config = YAML::LoadFile(trajectory_file_);
        } catch (const YAML::Exception& e) {
            ROS_ERROR("Failed to parse YAML: %s", e.what());
            return false;
        }
        
        // Get frame_id from file or use default
        if (config["frame_id"]) {
            source_frame_ = config["frame_id"].as<std::string>();
        }
        
        if (!config["trajectory"] || !config["trajectory"].IsSequence()) {
            ROS_ERROR("Invalid YAML format: missing trajectory sequence");
            return false;
        }
        
        const YAML::Node& trajectory = config["trajectory"];
        for (const auto& point : trajectory) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = source_frame_;
            
            if (point["timestamp"]) {
                pose.header.stamp.fromSec(point["timestamp"].as<double>());
            } else {
                pose.header.stamp = ros::Time::now();
            }
            
            pose.pose.position.x = point["x"].as<double>();
            pose.pose.position.y = point["y"].as<double>();
            pose.pose.position.z = point["z"].as<double>();
            pose.pose.orientation.x = point["qx"].as<double>();
            pose.pose.orientation.y = point["qy"].as<double>();
            pose.pose.orientation.z = point["qz"].as<double>();
            pose.pose.orientation.w = point["qw"].as<double>();
            
            trajectory_points_.push_back(pose);
        }
        
        ROS_INFO("Loaded %zu trajectory points from YAML file: %s", 
                 trajectory_points_.size(), trajectory_file_.c_str());
        return true;
    }
    
    bool loadFromCsv() {
        std::ifstream file(trajectory_file_);
        std::string line;
        
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<double> values;
            
            while (std::getline(ss, cell, ',')) {
                values.push_back(std::stod(cell));
            }
            
            if (values.size() != 8) {
                ROS_WARN("Invalid CSV line format, expected 8 values");
                continue;
            }
            
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = source_frame_;
            pose.header.stamp.fromSec(values[0]);
            pose.pose.position.x = values[1];
            pose.pose.position.y = values[2];
            pose.pose.position.z = values[3];
            pose.pose.orientation.x = values[4];
            pose.pose.orientation.y = values[5];
            pose.pose.orientation.z = values[6];
            pose.pose.orientation.w = values[7];
            
            trajectory_points_.push_back(pose);
        }
        
        ROS_INFO("Loaded %zu trajectory points from CSV file: %s", 
                 trajectory_points_.size(), trajectory_file_.c_str());
        return true;
    }
    
    bool transformTrajectory() {
        if (source_frame_ == target_frame_) {
            ROS_INFO("Source and target frames are the same, no transformation needed");
            return true;
        }
        
        std::vector<geometry_msgs::PoseStamped> transformed_points;
        
        for (const auto& pose : trajectory_points_) {
            try {
                geometry_msgs::PoseStamped transformed_pose;
                
                // Wait for transform to be available
                if (!tf_buffer_.canTransform(target_frame_, source_frame_, ros::Time(0), ros::Duration(1.0))) {
                    ROS_WARN("Transform from %s to %s not available", 
                             source_frame_.c_str(), target_frame_.c_str());
                    return false;
                }
                
                tf_buffer_.transform(pose, transformed_pose, target_frame_);
                transformed_points.push_back(transformed_pose);
                
            } catch (tf2::TransformException &ex) {
                ROS_WARN("Transform failed: %s", ex.what());
                return false;
            }
        }
        
        trajectory_points_ = transformed_points;
        ROS_INFO("Transformed %zu trajectory points from %s to %s", 
                 trajectory_points_.size(), source_frame_.c_str(), target_frame_.c_str());
        return true;
    }
    
    visualization_msgs::MarkerArray createMarkerArray() {
        visualization_msgs::MarkerArray marker_array;
        
        if (trajectory_points_.empty()) {
            return marker_array;
        }
        
        // Create line strip marker
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = target_frame_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "trajectory_line";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.lifetime = ros::Duration(marker_lifetime_);
        
        // Set line properties
        line_strip.scale.x = 0.05; // Line width
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;
        line_strip.pose.orientation.w = 1.0;
        
        // Create points marker
        visualization_msgs::Marker points;
        points.header = line_strip.header;
        points.ns = "trajectory_points";
        points.id = 1;
        points.type = visualization_msgs::Marker::POINTS;
        points.action = visualization_msgs::Marker::ADD;
        points.lifetime = ros::Duration(marker_lifetime_);
        
        // Set points properties
        points.scale.x = 0.1; // Point size
        points.scale.y = 0.1;
        points.color.r = 1.0;
        points.color.g = 0.0;
        points.color.b = 0.0;
        points.color.a = 1.0;
        points.pose.orientation.w = 1.0;
        
        // Add points to markers
        for (const auto& pose : trajectory_points_) {
            geometry_msgs::Point p;
            p.x = pose.pose.position.x;
            p.y = pose.pose.position.y;
            p.z = pose.pose.position.z;
            
            line_strip.points.push_back(p);
            points.points.push_back(p);
        }
        
        marker_array.markers.push_back(line_strip);
        marker_array.markers.push_back(points);
        
        return marker_array;
    }
    
public:
    TrajectoryReader() : nh_("~"), tf_listener_(tf_buffer_) {
        // Load parameters
        nh_.param<std::string>("trajectory_file", trajectory_file_, "/tmp/trajectory.json");
        nh_.param<std::string>("source_frame", source_frame_, "odom");
        nh_.param<std::string>("target_frame", target_frame_, "odom");
        nh_.param<double>("marker_lifetime", marker_lifetime_, 0.0); // 0 = forever
        nh_.param<double>("publish_rate", publish_rate_, 10.0);
        
        // Initialize publisher
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1, true);
        
        ROS_INFO("Trajectory Reader and Publisher Node initialized");
        
        // Load trajectory from file
        if (!loadTrajectoryFromFile()) {
            ROS_ERROR("Failed to load trajectory from file");
            return;
        }
        
        // Transform trajectory if needed
        transformTrajectory();
    }
    
    void run() {
        ros::Rate rate(publish_rate_);
        
        while (ros::ok()) {
            if (!trajectory_points_.empty()) {
                marker_pub_.publish(createMarkerArray());
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_reader");
    TrajectoryReader node;
    node.run();
    return 0;
}