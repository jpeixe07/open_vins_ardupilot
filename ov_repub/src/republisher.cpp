#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class OdomRepublisher : public rclcpp::Node
{
public:
    OdomRepublisher(const std::string & input_topic, const std::string & output_topic, bool is_correct)
        : Node("republisher"), is_correct_(is_correct)
    {
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_topic, 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic,
            10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->callback(msg); });
        // publish at 20 Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&OdomRepublisher::timer_callback, this));
    
        latest_msg_ = nullptr;
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto odom_msg = *msg;

        if (!is_correct_)
        {

            // convert geometry_msgs::Quaternion to tf2::Quaternion
            tf2::Quaternion q(
                odom_msg.pose.pose.orientation.x,
                odom_msg.pose.pose.orientation.y,
                odom_msg.pose.pose.orientation.z,
                odom_msg.pose.pose.orientation.w);
            
            std::cout << "q: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
            
            // 90 degree quaternion rotation
            tf2::Quaternion q90;
            q90.setRPY(0, 0, -M_PI/2);
            q = q90 * q;
            
            q.normalize();

            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();

            double tmp_x = odom_msg.pose.pose.position.x;
            double tmp_y = odom_msg.pose.pose.position.y;
            odom_msg.pose.pose.position.y = -tmp_x;
            odom_msg.pose.pose.position.x = tmp_y;

            // tf2::Quaternion q(
            //     odom_msg.pose.pose.orientation.x,
            //     odom_msg.pose.pose.orientation.y,
            //     odom_msg.pose.pose.orientation.z,
            //     odom_msg.pose.pose.orientation.w);

            // tf2::Matrix3x3 rotation_matrix(q);

            // Eigen::Matrix4f eigen_rotation_matrix;
            // eigen_rotation_matrix << rotation_matrix[0].x(), rotation_matrix[0].y(), rotation_matrix[0].z(), 0,
            //                          rotation_matrix[1].x(), rotation_matrix[1].y(), rotation_matrix[1].z(), 0,
            //                          rotation_matrix[2].x(), rotation_matrix[2].y(), rotation_matrix[2].z(), 0,
            //                          0, 0, 0, 1;

            // Eigen::Matrix4f ninety_degree_matrix;
            // ninety_degree_matrix << 0, -1, 0, 0,
            //                         -1, 0, 0, 0,
            //                         0, 0, 1, 0,
            //                         0, 0, 0, 1;

            // Eigen::Matrix4f new_rotation_matrix = ninety_degree_matrix * eigen_rotation_matrix;

            // // Convert Eigen Matrix3f to tf2::Matrix3x3
            // tf2::Matrix3x3 new_rotation_matrix_tf2;
            // new_rotation_matrix_tf2.setValue(new_rotation_matrix(0, 0), new_rotation_matrix(0, 1), new_rotation_matrix(0, 2),
            //                                  new_rotation_matrix(1, 0), new_rotation_matrix(1, 1), new_rotation_matrix(1, 2),
            //                                  new_rotation_matrix(2, 0), new_rotation_matrix(2, 1), new_rotation_matrix(2, 2));

            // tf2::Quaternion new_q;
            
            // new_rotation_matrix_tf2.getRotation(new_q);
            

            // odom_msg.pose.pose.orientation.x = -new_q.x();
            // odom_msg.pose.pose.orientation.y = -new_q.y();
            // odom_msg.pose.pose.orientation.z = -new_q.z();
            // odom_msg.pose.pose.orientation.w = new_q.w();
        }

        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        latest_msg_ = std::make_shared<nav_msgs::msg::Odometry>(odom_msg);
    }


    void timer_callback()
    {
        if (latest_msg_ != nullptr)
        {
            pub_->publish(*latest_msg_);
            std::cout << "I'm inside" << std::endl;
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_correct_;

    nav_msgs::msg::Odometry::SharedPtr latest_msg_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomRepublisher>("/ov_msckf/odomimu", "/mavros/odometry/out", false);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
