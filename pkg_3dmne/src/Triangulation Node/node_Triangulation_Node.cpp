#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "Triangulation_Node");

    // Start the Triangulation Node Nodelet
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(ros::this_node::getName(), "ns_3DMNE/ns_3DMNE", remap, nargv);

	ros::spin();
	return 0;
}