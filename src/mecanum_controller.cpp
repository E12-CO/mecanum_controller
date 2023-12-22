// Mecanum controller and interface for ROS2 
// Designed for ABU Robocon 2024 by TinLethax at Robot C

#include <chrono>
#include <cmath>
#include <string>
#include <iostream>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h> 

struct termios tty;

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

// tf2 lib
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

// Geometry lib
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

int serial_port = 0;

class mecanum_rbc : public rclcpp::Node{
	
	public:
	
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubRobotOdom;
	
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_sub;
	
	rclcpp::TimerBase::SharedPtr timer_;
	
	std::unique_ptr<tf2_ros::TransformBroadcaster> br;
	
    std::string robot_frame_id;
    std::string odom_frame_id;
	std::string serial_port_;
	int slen = 0;
	
	int serial_port = 0;
	
	float denum = 0.0;
	
	float x_vel,y_vel,az_vel = 0.0;
	float robot_width, robot_length, robot_total_len = 0.0;// In unit of meter
	float gear_ratio = 0.0;
	float max_x_vel, max_y_vel, max_az_vel = 0.0;
	float wheel_radius = 0.0;// In meter unit 
	float invert_LF, invert_LB, invert_RF, invert_RB = 0.0;
	int invert_LF_int, invert_LB_int, invert_RF_int, invert_RB_int = 0;
	int forward_LF_int, forward_LB_int, forward_RF_int, forward_RB_int = 0;
	float forward_LF, forward_LB, forward_RF, forward_RB = 0.0;
	float cur_x_pos, cur_y_pos, cur_az_ang = 0.0;
	
	// initalize ROS2 node
	mecanum_rbc() : Node("MecanumControllerInterface"){
		
		RCLCPP_INFO(this->get_logger(), "Robot club KMITL : Starting mecanum controller...");

		declare_parameter("serial_port", "/dev/ttyUSB0");
		get_parameter("serial_port", serial_port_);
		declare_parameter("robot_frame_id", "base_link");
		get_parameter("robot_frame_id", robot_frame_id);
		declare_parameter("odom_frame_id", "odom");
		get_parameter("odom_frame_id", odom_frame_id);
		declare_parameter("robot_width", 0.3);
		get_parameter("robot_width", robot_width);	
		declare_parameter("robot_length", 0.2);
		get_parameter("robot_length", robot_length);
		declare_parameter("wheel_radius", 0.06); 
		get_parameter("wheel_radius", wheel_radius);
		declare_parameter("gear_ratio", 1.0);
		get_parameter("gear_ratio", gear_ratio);
		declare_parameter("maximum_x_velocity", max_x_vel);
		get_parameter("maximum_x_velocity", max_x_vel);
		declare_parameter("maximum_y_velocity", max_y_vel);
		get_parameter("maximum_y_velocity", max_y_vel);
		declare_parameter("maximum_az_velocity", max_az_vel);
		get_parameter("maximum_az_velocity", max_az_vel);
		
		robot_total_len = robot_width + robot_length;
		
		slen = serial_port_.length();
		char* serial_port_file = new char[slen+1];
		strcpy(serial_port_file, serial_port_.c_str());
		serial_port = open(serial_port_file, O_RDWR);
		
		if(serial_port < 0){
			RCLCPP_INFO(this->get_logger(), "Error openning Serial");
			return;
		}
		
		if(tcgetattr(serial_port, &tty) != 0) {
			printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
			return;
		}
		
		tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
		tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
		tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
		tty.c_cflag |= CS8; // 8 bits per byte (most common)
		tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
		tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

		tty.c_lflag &= ~ICANON;
		tty.c_lflag &= ~ECHO; // Disable echo
		tty.c_lflag &= ~ECHOE; // Disable erasure
		tty.c_lflag &= ~ECHONL; // Disable new-line echo
		tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
		tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
		tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

		tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
		tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
		// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
		// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

		tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
		tty.c_cc[VMIN] = 0;

		// Set in/out baud rate to be 115200
		cfsetispeed(&tty, B115200);
		cfsetospeed(&tty, B115200);
		
		if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return;
		}
		
		motion_sub = create_subscription<geometry_msgs::msg::Twist>(
					"/cmd_vel",
					10,
					std::bind(&mecanum_rbc::motion_callback, this, std::placeholders::_1) 
					);
		
		pubRobotOdom = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
		
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(20),
			std::bind(&mecanum_rbc::feedback_tf, this)
			);
	
		tcflush(serial_port,TCIOFLUSH);// FLush buffer before start	
		RCLCPP_INFO(this->get_logger(), "mecanum controller started!");
		
	}
	
	// Callback handling 
	// 1. prcess cmd_vel data to mecanum velocity
	// 2. send data to MCU via serial
	// 3. read back encoder and calculate motion vectors
	// 4. publish odom and do transfer
	void motion_callback(const geometry_msgs::msg::Twist::SharedPtr twist_data){		
		//1. grab twist_data->linear.x , twist_data->linear.y and twist_data->angular.z linear unit is m/s and angular unit is rad/s
		x_vel 	= twist_data->linear.x;
		y_vel	= twist_data->linear.y;
		az_vel	= twist_data->angular.z;
		
		//2. inverse kinematic
		// Also convert rad/s to rpm
		invert_LF 	= ((x_vel - y_vel - (robot_total_len*az_vel)) / wheel_radius) * 9.5493;
		invert_LB 	= ((x_vel + y_vel - (robot_total_len*az_vel)) / wheel_radius) * 9.5493;
		invert_RF	= ((x_vel + y_vel + (robot_total_len*az_vel)) / wheel_radius) * 9.5493;
		invert_RB	= ((x_vel - y_vel + (robot_total_len*az_vel)) / wheel_radius) * 9.5493;
		
		// Convert float to int
		invert_LF_int 	= round(invert_LF);
		invert_LB_int 	= round(invert_LB);
		invert_RF_int	= round(invert_RF);
		invert_RB_int 	= round(invert_RB);
		
		// Send to Serial
		std::string send_packet;
		
		send_packet += "Rb";
		send_packet += std::to_string(invert_LF_int);
		send_packet += " ";
		send_packet += std::to_string(invert_LB_int);
		send_packet += " ";
		send_packet += std::to_string(invert_RF_int);
		send_packet += " ";
		send_packet += std::to_string(invert_RB_int);
		send_packet += "\n";
		
		int char_size = send_packet.length();
		
		char* send_char_array = new char[char_size + 1];
		
		strcpy(send_char_array, send_packet.c_str());
		write(serial_port, send_char_array, char_size + 1);
	}
	
	char rx_buf[30];
	void feedback_tf(){
		int rx_bytes = 0;
		int idx = 0;
		ioctl(serial_port, FIONREAD, &rx_bytes);
		// Receive from Serial 
		if(rx_bytes > 0){
			read(serial_port, rx_buf,rx_bytes);
			// Detect Magic word "RB"
			if(rx_buf[0] != 'R')
				return;
			if(rx_buf[1] != 'b')
				return;
			
			std::string lf, lb, rf, rb;
			
			idx+=2;// Offset away from magic word
			while(rx_buf[idx] != ' '){
				lf += (rx_buf[idx++]);
				if(idx > rx_bytes)
					return;
			}
			lf += '\0';
			idx++;
			while(rx_buf[idx] != ' '){
				lb += (rx_buf[idx++]);
				if(idx > rx_bytes)
					return;
			}
			lb += '\0';
			idx++;
			while(rx_buf[idx] != ' '){
				rf += (rx_buf[idx++]);
				if(idx > rx_bytes)
					return;
			}
			rf += '\0';
			idx++;
			while(rx_buf[idx] != '\n'){
				rb += (rx_buf[idx++]);
				if(idx > rx_bytes)
					return;
			}
			rb += '\0';
			
			forward_LF_int 	= std::stoi(lf);
			forward_LB_int	= std::stoi(lb);
			forward_RF_int	= std::stoi(rf);
			forward_RB_int	= std::stoi(rb);
			
			// RPM to rad/s
			forward_LF	= forward_LF_int / 9.5493;
			forward_LB	= forward_LB_int / 9.5493;
			forward_RF	= forward_RF_int / 9.5439;
			forward_RB	= forward_RB_int / 9.5439;
		
			// divided with gear ratio
			forward_LF = forward_LF / gear_ratio;
			forward_LB = forward_LB / gear_ratio;
			forward_RF = forward_RF / gear_ratio;
			forward_RB = forward_RB / gear_ratio;
		
			//4. read back encoder velocity and do forward kinematic
			x_vel	 	= (forward_LF + forward_LB + forward_RF + forward_RB) * (wheel_radius/4);
			y_vel 		= (-forward_LF + forward_LB + forward_RF - forward_RB) * (wheel_radius/4);
			az_vel 		= (-forward_LF - forward_LB + forward_RF + forward_RB) * (wheel_radius/(4*robot_total_len));		

			//5. calculate position with integration of x_vel y_vel and az_vel
			
			cur_az_ang 	+= az_vel * 0.023; 
			cur_x_pos 	+= ((x_vel * cos(cur_az_ang)) - (y_vel * sin(cur_az_ang))) * 0.023;
			cur_y_pos 	+= ((x_vel * sin(cur_az_ang)) + (y_vel * cos(cur_az_ang))) * 0.023;
			
			nav_msgs::msg::Odometry robotOdom;
			br = std::make_unique<tf2_ros::TransformBroadcaster>(this);
			geometry_msgs::msg::TransformStamped transform;

			tf2::Quaternion xyz_angular;
			xyz_angular.setRPY(0, 0, cur_az_ang);
			//xyz_angular = xyz_angular.normalize();
			
			// Publish message on Odom topic 
			robotOdom.header.frame_id 			= odom_frame_id;
			robotOdom.child_frame_id			= robot_frame_id;
			robotOdom.header.stamp 				= this->get_clock()->now();
			robotOdom.twist.twist.linear.x 		= x_vel;
			robotOdom.twist.twist.angular.z 	= az_vel;
			robotOdom.pose.pose.orientation.x 	= xyz_angular.getX();
			robotOdom.pose.pose.orientation.y 	= xyz_angular.getY();
			robotOdom.pose.pose.orientation.z 	= xyz_angular.getZ();
			robotOdom.pose.pose.orientation.w 	= xyz_angular.getW();
			robotOdom.pose.pose.position.x 		= cur_x_pos;
			robotOdom.pose.pose.position.y 		= cur_y_pos;
			pubRobotOdom->publish(robotOdom);
			
			// Do the Odom transform
			transform.header.stamp 				= robotOdom.header.stamp;
			transform.header.frame_id 			= odom_frame_id;
			transform.child_frame_id 			= robot_frame_id;
			transform.transform.translation.x 	= cur_x_pos;
			transform.transform.translation.y 	= cur_y_pos;
			transform.transform.rotation.x 		= round(xyz_angular.getX() * 100) / 100;
			transform.transform.rotation.y 		= round(xyz_angular.getY() * 100) / 100;
			transform.transform.rotation.z 		= round(xyz_angular.getZ() * 100) / 100;
			transform.transform.rotation.w 		= round(xyz_angular.getW() * 100) / 100;
			br->sendTransform(transform);
				
		}
	}
	
};

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	auto mecanumrbc {std::make_shared<mecanum_rbc>()};
	rclcpp::spin(mecanumrbc);
	close(serial_port);
	rclcpp::shutdown();
}

