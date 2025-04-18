#include <r270_spec.h>

#if defined(ROS1)
#include <ros1/kanavi_node.h>

// Entry point for this module
/**
 * @brief [TODO] Describe the function main
 * @return int [description]
 * @param argc [description]
 * @param **argv [description]
 */
int main(int argc, char **argv)
{
RCLCPP_INFO(node_->get_logger(), "ROS1 build test");
	ros::init(argc, argv, "r270");

/**
 * @brief [TODO] Describe the function node
 * @return kanavi_node [description]
 * @param "r270" [description]
 * @param argc [description]
 * @param argv [description]
 */
	kanavi_node node("r270", argc, argv);

	node.run();

	return 0;
}

#elif defined (ROS2)

#include <ros2/kanavi_node.h>

// Entry point for this module
/**
 * @brief [TODO] Describe the function main
 * @return int [description]
 * @param argc [description]
 * @param **argv [description]
 */
int main(int argc, char **argv)
{
	// init ROS2
	rclcpp::init(argc, argv);

	// generate node
	auto node = std::make_shared<kanavi_node>("r270", argc, argv);

	// start node
	rclcpp::spin(node);

	// exit node
	rclcpp::shutdown();

	return 0;
}

#endif