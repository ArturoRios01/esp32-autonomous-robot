#include "follow_carrot/follow_carrot.hpp"

using std::placeholders::_1;

FollowCarrot::FollowCarrot() : Node("follow_carrot_node")
{
    // 1. Inicializar variables
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_theta_ = 0.0;
    pose_received_ = false;
    current_waypoint_index_ = 0;
    stop=false;

    // 2. Parámetros ajustables
    dist_tolerance_ = 0.10; // 10 cm de margen
    kp_angular_ = 2.0;      // Ganancia de giro
    max_linear_vel_ = 0.30; // Velocidad de avance (m/s)
    max_angular_vel_ = 0.7; // Velocidad de giro máx
    vel_ant=0;

    // 3. Definir la trayectoria (rectangulo de 0.7x0.5 metro)
    trajectory_ = {
        {0.7, 0.0},
        {0.7, 0.5},
        {0.0, 0.5},
        {0.0, 0.0},
    };

    // 4. Configurar Suscriptor (Pose2D)
    // Escuchamos en "/robot_pose" que es lo que configuramos en el ESP32
    sub_pose_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "/robot_pose", 10, std::bind(&FollowCarrot::pose_callback, this, _1));

    // Configurar Publicador (Velocidad)
    pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    // 5. Timer de control (20Hz = 50ms)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&FollowCarrot::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Nodo Follow Carrot (Pose2D) iniciado.");
}

void FollowCarrot::pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_theta_ = msg->theta;
    
    pose_received_ = true;
}

void FollowCarrot::timer_callback()
{
    // Si no hemos recibido posición del ESP32 o acabamos la ruta, no hacemos nada
    if (!pose_received_ || current_waypoint_index_ >= trajectory_.size()) {
        return;
    }

    // 1. Obtener el siguiente punto objetivo
    Point target = trajectory_[current_waypoint_index_];

    // 2. Calcular distancia al objetivo
    double dx = target.x - current_x_;
    double dy = target.y - current_y_;
    double distance = std::sqrt(dx*dx + dy*dy);

    // 3. Comprobar si hemos llegado
    if (distance < dist_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Punto %ld alcanzado!", current_waypoint_index_);
        current_waypoint_index_++;
        
        if (current_waypoint_index_ >= trajectory_.size()) {
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "Ruta completada.");
        }
        return;
    }

    // 4. Algoritmo de Control
    // Ángulo hacia el objetivo
    double desired_yaw = std::atan2(dy, dx);
    double yaw_error = desired_yaw - current_theta_;

    // Normalizar error angular entre -PI y PI
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    geometry_msgs::msg::Twist cmd_msg;

    // A. Giro Proporcional
    cmd_msg.angular.z = std::min(kp_angular_ * yaw_error,max_angular_vel_);

    // Saturación giro
    if (cmd_msg.angular.z > max_angular_vel_) cmd_msg.angular.z = max_angular_vel_;
    if (cmd_msg.angular.z < -max_angular_vel_) cmd_msg.angular.z = -max_angular_vel_;

    // B. Avance
    // Solo avanzamos si el error angular es pequeño (acmd_msg.linear.x = std::min(vel_ant+0.1,max_linear_vel_);prox 20 grados)
    // Pégalo justo después de calcular y normalizar el yaw_error
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
    "Deseado: %.2f | Actual: %.2f | Error: %.2f", 
    desired_yaw, current_theta_, yaw_error);

    if (std::abs(yaw_error) > 0.1) {
        cmd_msg.linear.x = 0.0; // Girar en el sitio
        vel_ant=0;
    } else {
        if(distance>1.0){
           cmd_msg.linear.x = std::min(vel_ant+0.1,max_linear_vel_); 
           vel_ant=std::min(max_linear_vel_,vel_ant+0.1);
        }
        else{
            cmd_msg.linear.x=std::max(0.1,vel_ant+0.1);
        }
        
    }
    if(!stop){
        pub_cmd_->publish(cmd_msg);
    }
   
}

void FollowCarrot::stop_robot()
{
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = 0.0;
    cmd_msg.angular.z = 0.0;
    stop=true;
    pub_cmd_->publish(cmd_msg);
    
}

int main ( int argc, char * argv[] )
{
    rclcpp::init ( argc, argv );
    
    auto node=std::make_shared<FollowCarrot>();

    rclcpp::Rate rate(20);

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}