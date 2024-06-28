#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <tuple>

class CameraImage {
public:
    CameraImage() :
        r_cx(0.122),
        r_cy(-0.033),
        r_cz(0.082),
        r_c_roll(M_PI / 2),
        r_c_pitch(0.0),
        r_c_yaw(M_PI / 2),
        k(1),
        camera{r_cx, r_cy, r_cz, r_c_roll, r_c_pitch, r_c_yaw},
        tf_br()
    {
        image_pub = nh.advertise<nav_msgs::Odometry>("/aruco_position", 10);
        image_sub = nh.subscribe("/kobuki/sensors/realsense/color/image_color", 10, &CameraImage::imageCallback, this);
        beacon_list.push_back(1);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher image_pub;
    ros::Subscriber image_sub;

    double r_cx, r_cy, r_cz, r_c_roll, r_c_pitch, r_c_yaw;
    int k;
    std::vector<cv::Mat> world;
    std::vector<cv::Mat> R_pose;
    std::array<double, 6> camera;
    tf::TransformBroadcaster tf_br;
    std::vector<int> beacon_list;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat cv_image = cv_ptr->image;
        // Continue with the rest of the image processing...
    }
};

void drawCube(cv::Mat& frame, const cv::Vec3d& rvecs, const cv::Vec3d& tvecs, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, double marker_length) {
    // Define the color of the cube's edges
    cv::Scalar color(0, 0, 255); // BGR color format

    // Project the 3D points of the cube onto the image plane
    std::vector<cv::Point3f> cube_points = {
        {-marker_length / 2, marker_length / 2, 0},
        {marker_length / 2, marker_length / 2, 0},
        {marker_length / 2, -marker_length / 2, 0},
        {-marker_length / 2, -marker_length / 2, 0},
        {-marker_length / 2, marker_length / 2, marker_length},
        {marker_length / 2, marker_length / 2, marker_length},
        {marker_length / 2, -marker_length / 2, marker_length},
        {-marker_length / 2, -marker_length / 2, marker_length}
    };
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(cube_points, rvecs, tvecs, camera_matrix, dist_coeffs, image_points);

    // Draw the edges of the cube on the frame
    std::vector<std::vector<cv::Point>> faces = {
        {image_points[0], image_points[1], image_points[2], image_points[3]},
        {image_points[4], image_points[5], image_points[6], image_points[7]}
    };
    for (const auto& face : faces) {
        cv::polylines(frame, face, true, color, 2, cv::LINE_AA);
    }
    for (int i = 0; i < 4; ++i) {
        cv::line(frame, image_points[i], image_points[i + 4], color, 2, cv::LINE_AA);
    }
}

// Function to perform the transformation
std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d> transform_r_c(double x, double y, double z, double roll, double pitch, double yaw) {
    Eigen::Matrix4d Transf = Eigen::Matrix4d::Identity();

    // Convert Euler angles to rotation matrix
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, std::cos(roll), -std::sin(roll),
          0, std::sin(roll), std::cos(roll);

    Eigen::Matrix3d Rz;
    Rz << std::cos(yaw), -std::sin(yaw), 0,
          std::sin(yaw), std::cos(yaw), 0,
          0, 0, 1;

    Eigen::Matrix3d R = Rz * Rx;

    // Create transformation matrix
    Eigen::Vector3d Trans;
    Trans << x, y, z;

    // Set the transformation matrix
    Transf.block<3, 3>(0, 0) = R;
    Transf.block<3, 1>(0, 3) = Trans;

    return std::make_tuple(Trans, R, Transf);
}

// Function to perform the transformation
std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix4d> transform_w_a(double x, double y, double z, double roll, double pitch, double yaw) {
    Eigen::Matrix4d Transf = Eigen::Matrix4d::Identity();

    // Convert Euler angles to rotation matrix
    Eigen::Matrix3d Rx;
    Rx << 1, 0, 0,
          0, std::cos(roll), -std::sin(roll),
          0, std::sin(roll), std::cos(roll);

    Eigen::Matrix3d Rz;
    Rz << std::cos(yaw), -std::sin(yaw), 0,
          std::sin(yaw), std::cos(yaw), 0,
          0, 0, 1;

    Eigen::Matrix3d R = Rz * Rx;

    // Create transformation matrix
    Eigen::Vector3d Trans;
    Trans << x, y, z;

    // Set the transformation matrix
    Transf.block<3, 3>(0, 0) = R;
    Transf.block<3, 1>(0, 3) = Trans;

    return std::make_tuple(Trans, R, Transf);
}

void imageCallback(const sensor_msgs::ImageConstPtr& Image_msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(Image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Set marker ID and length
    double marker_length = 0.16;

    // Load the ArUco dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

    // Camera matrix and distortion coefficients
    Eigen::Matrix3d camera_matrix;
    camera_matrix << 1396.8086675255468, 0.0, 960.0,
                     0.0, 1396.8086675255468, 540.0,
                     0.0, 0.0, 1.0;

    Eigen::VectorXd dist_coeffs(5);
    dist_coeffs << 0.0, 0.0, 0.0, 0.0, 0.0;

    // Create a video capture object for the camera
    cv::Mat frame = cv_ptr->image.clone();

    // Display the frame on the screen
    cv::namedWindow("Camera", cv::WINDOW_NORMAL);
    cv::resizeWindow("Camera", 800, 600);

    std::vector<int> beacon_list = {1};

    if (k == 1) {
        // Aruco one position in world frame (should be known a priori)
        auto [_, _, Tranformation] = transform_w_a(0, 1, 0, -M_PI / 2, 0, M_PI);
        world.push_back(Tranformation);
        k = 2;
    }

    // Detect ArUco markers in the frame
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    cv::aruco::detectMarkers(frame, dictionary, marker_corners, marker_ids);

    std::vector<Eigen::Matrix4d> world_list;

    if (!marker_ids.empty()) {
        for (size_t i = 0; i < marker_ids.size(); ++i) {
            int id = marker_ids[i];

            if (std::find(beacon_list.begin(), beacon_list.end(), id) == beacon_list.end()) {
                if (!R_pose.isZero()) {
                    cv::Vec3d rvecs, tvecs;
                    cv::aruco::estimatePoseSingleMarkers(marker_corners[i], marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);

                    Eigen::Matrix4d Transf;
                    Eigen::Vector3d Trans(tvecs(0), tvecs(1), tvecs(2));
                    cv::Mat rot_mat;
                    cv::Rodrigues(rvecs, rot_mat);
                    Eigen::Matrix3d Rot;
                    cv::cv2eigen(rot_mat, Rot);

                    Transf.block<3, 3>(0, 0) = Rot;
                    Transf.block<3, 1>(0, 3) = Trans;

                    auto [_, _, Transf_r_c] = transform_r_c(camera(0), camera(1), camera(2), camera(3), camera(4), camera(5));
                    Eigen::Matrix4d Transf_r = Transf_r_c * Transf;

                    Eigen::Matrix3d rot_r = Transf_r.block<3, 3>(0, 0);
                    Eigen::Vector3d Trans_r = Transf_r.block<3, 1>(0, 3);
                    Eigen::Matrix4d Transf_a;
                    Transf_a.block<3, 3>(0, 0) = rot_r.transpose();
                    Transf_a.block<3, 1>(0, 3) = -rot_r.transpose() * Trans_r;

                    cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids, cv::Scalar(0, 255, 0));
                    cv::aruco::drawAxis(frame, camera_matrix, dist_coeffs, rvecs, tvecs, 0.05);

                    Eigen::Matrix4d C_pose = Transf_a;

                    if (id == 1) {
                        if (k == 2) {
                            R_pose = Tranformation * C_pose;
                            k = 3;
                        }
                        Eigen::Matrix4d W_pose = Tranformation * C_pose;
                        world_list.push_back(W_pose);
                        R_pose = W_pose;
                    } else {
                        auto it = std::find(beacon_list.begin(), beacon_list.end(), id);
                        size_t idx = std::distance(beacon_list.begin(), it);
                        Eigen::Matrix4d world = C_pose;
                        Eigen::Matrix4d Aruco_world = world[idx];
                        Eigen::Matrix4d W_pose = Aruco_world * world;
                        world_list.push_back(W_pose);
                    }
                }
            }
        }

        if (!world_list.empty()) {
            Eigen::Matrix4d average_transform = world_list[0];
            for (size_t i = 1; i < world_list.size(); ++i) {
                average_transform += world_list[i];
            }
            average_transform /= world_list.size();

            R_pose = average_transform;

            double x = average_transform(0, 3);
            double y = average_transform(1, 3);
            double z = average_transform(2, 3);

            Eigen::Matrix3d Rot_avg = average_transform.block<3, 3>(0, 0);
            Eigen::Vector3d euler_angles = Rot_avg.eulerAngles(0, 1, 2);

            Eigen::Quaterniond q(Rot_avg);

            x_translation.push_back(x);
            y_translation.push_back(y);
            z_translation.push_back(z);

            // Publishing the message
            nav_msgs::Odometry odom_msg;
            odom_msg.header.frame_id = "world";
            odom_msg.child_frame_id = "kobuki/base_footprint";

            geometry_msgs::Point point_msg;
            point_msg.x = x;
            point_msg.y = y;
            point_msg.z = z;

            odom_msg.pose.pose.position = point_msg;

            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();

            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;

            image_pub.publish(odom_msg);

            std::string position = "Position: (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
            std::cout << "Position: (" << x << ", " << y << ", " << z << ")" << std::endl;
            cv::putText(frame, position, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

            tf_br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()), tf::Vector3(x, y, z)), ros::Time::now(), "world", "kobuki/base_footprint"));
        }
    }

    // Display the frame on the screen
    cv::imshow("Camera", frame);
    cv::waitKey(3);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_sub");
    CameraImage camera_image;
    ros::spin();
    return 0;
} 