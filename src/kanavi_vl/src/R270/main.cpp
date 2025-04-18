#include <r270_spec.h>

#if defined(ROS1)
#include <ros1/kanavi_node.h>

// Entry point for this module
int main(int argc, char **argv)
{
	ros::init(argc, argv, "r270");

	kanavi_node node("r270", argc, argv);

	node.run();

	return 0;
}

#elif defined (ROS2)

#include <ros2/kanavi_node.h>

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