#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <jsoncpp/json/json.h>
#include "trajectory_visualization/SaveTrajectory.h"

class TrajectorySaver {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher marker_pub_;
    ros::ServiceServer save_service_;
    
    std::deque<geometry_msgs::PoseStamped> trajectory_points_;
    ros::Time last_update_time_;
    
    // Parameters
    std::string odom_topic_;
    std::string base_frame_;
    std::string odom_frame_;
    double marker_lifetime_;
    double publish_rate_;
    double point_distance_threshold_;
    
    visualization_msgs::MarkerArray createMarkerArray() {
        visualization_msgs::MarkerArray marker_array;
        
        // Create line strip marker
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = odom_frame_;
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
    
    bool saveTrajectoryCallback(trajectory_visualization::SaveTrajectory::Request &req,
                               trajectory_visualization::SaveTrajectory::Response &res) {
        bool success = saveTrajectoryToFile(req.filename, req.duration);
        
        if (success) {
            res.success = true;
            res.message = "Trajectory saved successfully to " + req.filename;
        } else {
            res.success = false;
            res.message = "Failed to save trajectory to " + req.filename;
        }
        
        return true;
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        
        // Only add point if it's far enough from the last point
        if (trajectory_points_.empty() || 
            calculateDistance(trajectory_points_.back(), pose) > point_distance_threshold_) {
            trajectory_points_.push_back(pose);
            ROS_DEBUG("Added new trajectory point, total points: %zu", trajectory_points_.size());
        }
    }
    
    double calculateDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
        double dx = p1.pose.position.x - p2.pose.position.x;
        double dy = p1.pose.position.y - p2.pose.position.y;
        double dz = p1.pose.position.z - p2.pose.position.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    bool saveTrajectoryToFile(const std::string& filename, double duration) {
        // Filter trajectory points based on duration
        std::vector<geometry_msgs::PoseStamped> filtered_points;
        ros::Time cutoff_time = ros::Time::now() - ros::Duration(duration);
        
        for (const auto& point : trajectory_points_) {
            if (duration <= 0 || point.header.stamp >= cutoff_time) {
                filtered_points.push_back(point);
            }
        }
        
        if (filtered_points.empty()) {
            ROS_WARN("No trajectory points to save");
            return false;
        }
        
        // Determine file format from extension
        std::string extension = filename.substr(filename.find_last_of(".") + 1);
        
        try {
            if (extension == "json") {
                return saveToJson(filename, filtered_points);
            } else if (extension == "yaml" || extension == "yml") {
                return saveToYaml(filename, filtered_points);
            } else if (extension == "csv") {
                return saveToCsv(filename, filtered_points);
            } else {
                ROS_ERROR("Unsupported file format: %s", extension.c_str());
                return false;
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Error saving trajectory: %s", e.what());
            return false;
        }
    }
    
    bool saveToJson(const std::string& filename, const std::vector<geometry_msgs::PoseStamped>& points) {
        Json::Value root;
        root["frame_id"] = odom_frame_;
        root["timestamp"] = ros::Time::now().toSec();
        
        Json::Value trajectory(Json::arrayValue);
        for (const auto& point : points) {
            Json::Value p;
            p["timestamp"] = point.header.stamp.toSec();
            p["x"] = point.pose.position.x;
            p["y"] = point.pose.position.y;
            p["z"] = point.pose.position.z;
            p["qx"] = point.pose.orientation.x;
            p["qy"] = point.pose.orientation.y;
            p["qz"] = point.pose.orientation.z;
            p["qw"] = point.pose.orientation.w;
            trajectory.append(p);
        }
        
        root["trajectory"] = trajectory;
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Could not open file for writing: %s", filename.c_str());
            return false;
        }
        
        Json::StreamWriterBuilder writer;
        writer["indentation"] = "  ";
        std::string document = Json::writeString(writer, root);
        file << document;
        file.close();
        
        ROS_INFO("Saved %zu trajectory points to JSON file: %s", points.size(), filename.c_str());
        return true;
    }
    
    bool saveToYaml(const std::string& filename, const std::vector<geometry_msgs::PoseStamped>& points) {
        YAML::Node root;
        root["frame_id"] = odom_frame_;
        root["timestamp"] = ros::Time::now().toSec();
        
        YAML::Node trajectory;
        for (size_t i = 0; i < points.size(); ++i) {
            const auto& point = points[i];
            YAML::Node p;
            p["timestamp"] = point.header.stamp.toSec();
            p["x"] = point.pose.position.x;
            p["y"] = point.pose.position.y;
            p["z"] = point.pose.position.z;
            p["qx"] = point.pose.orientation.x;
            p["qy"] = point.pose.orientation.y;
            p["qz"] = point.pose.orientation.z;
            p["qw"] = point.pose.orientation.w;
            trajectory[i] = p;
        }
        
        root["trajectory"] = trajectory;
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Could not open file for writing: %s", filename.c_str());
            return false;
        }
        
        file << YAML::Dump(root);
        file.close();
        
        ROS_INFO("Saved %zu trajectory points to YAML file: %s", points.size(), filename.c_str());
        return true;
    }
    
    bool saveToCsv(const std::string& filename, const std::vector<geometry_msgs::PoseStamped>& points) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Could not open file for writing: %s", filename.c_str());
            return false;
        }
        
        // Write header
        file << "timestamp,x,y,z,qx,qy,qz,qw" << std::endl;
        
        // Write data
        for (const auto& point : points) {
            file << point.header.stamp.toSec() << ","
                 << point.pose.position.x << ","
                 << point.pose.position.y << ","
                 << point.pose.position.z << ","
                 << point.pose.orientation.x << ","
                 << point.pose.orientation.y << ","
                 << point.pose.orientation.z << ","
                 << point.pose.orientation.w << std::endl;
        }
        
        file.close();
        
        ROS_INFO("Saved %zu trajectory points to CSV file: %s", points.size(), filename.c_str());
        return true;
    }
    
public:
    TrajectorySaver() : nh_("~") {
        // Load parameters
        nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<double>("marker_lifetime", marker_lifetime_, 0.0); // 0 = forever
        nh_.param<double>("publish_rate", publish_rate_, 10.0);
        nh_.param<double>("point_distance_threshold", point_distance_threshold_, 0.05);
        
        // Initialize subscribers, and services
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &TrajectorySaver::odomCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 1);
        save_service_ = nh_.advertiseService("save_trajectory", 
                                            &TrajectorySaver::saveTrajectoryCallback, this);
        
        ROS_INFO("Trajectory Saver Node initialized");
        ROS_INFO("Listening to odometry on: %s", odom_topic_.c_str());
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
    ros::init(argc, argv, "trajectory_saver");
    TrajectorySaver node;
    node.run();
    return 0;
}
