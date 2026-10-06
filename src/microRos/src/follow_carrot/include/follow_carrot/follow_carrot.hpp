#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp" 
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <cmath>

// IMPORTANTE: Definir la estructura Point aquí
struct Point {
    double x;
    double y;
};

class FollowCarrot : public rclcpp::Node
{
public:
    FollowCarrot();
    // No necesitamos destructor explícito si no usamos punteros raw
    // ~FollowCarrot(); 

private:
    // --- Callbacks ---
    // CORREGIDO: Usar Pose2D, no Odometry
    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void timer_callback();

    // --- Funciones Auxiliares ---
    void stop_robot();

    // --- Variables de ROS ---
    // CORREGIDO: El suscriptor debe ser de tipo Pose2D
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Variables de Estado del Robot ---
    double current_x_;
    double current_y_;
    double current_theta_;
    bool pose_received_; 
    bool stop;
    double vel_ant;

    // --- Variables de la Trayectoria ---
    std::vector<Point> trajectory_;
    size_t current_waypoint_index_;

    // --- Parámetros de Control ---
    double dist_tolerance_;   
    double kp_angular_;       
    double max_linear_vel_;   
    double max_angular_vel_;
};
