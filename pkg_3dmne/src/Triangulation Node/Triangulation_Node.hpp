// Basic Libraries
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <string>

// ROS Messages
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PolygonMesh.h>
#include <geometry_msgs/PoseStamped.h>

// ROS Services
#include <std_srvs/Empty.h>

// Filters
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

// Features
#include <pcl/features/normal_3d.h>

// Segmentation
#include <pcl/segmentation/segment_differences.h>

// Surface
#include <pcl/surface/gp3.h>

// IO
#include <pcl/io/ply_io.h>

namespace ns_3DMNE{
  class Class_Triangulation_Node : public nodelet::Nodelet{
    public:
        Class_Triangulation_Node (); // Constructor
        virtual ~Class_Triangulation_Node (); // Destructor
        virtual void onInit ();

    protected:
        void callback_pose (const geometry_msgs::PoseStamped::Ptr& msg_pose);
        void callback_fused_pc(const sensor_msgs::PointCloud2::ConstPtr& msg);
        bool distance_poses (geometry_msgs::PoseStamped::Ptr pose, geometry_msgs::PoseStamped::Ptr last_pose);
        void pose2trans_rot(geometry_msgs::PoseStamped::Ptr pose);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropbox_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, int box);
        void voxel_downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr subtract_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_buffer);
	pcl::PolygonMesh triangulation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);
	void save(pcl::PolygonMesh input);

        bool Save_reconstructed_map (std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    private:
        // Initialize ROS
        ros::NodeHandle n;
	ros::NodeHandle n_priv;
	
        // ROS Subscribers
        ros::Subscriber sub_pose;
        ros::Subscriber sub_fused_pc;

	// ROS Services
        ros::ServiceServer Save_Reconstructed_Map;

        // Vectors
	Eigen::Vector4f P_min = Eigen::Vector4f::Zero ();
        Eigen::Vector4f P_max = Eigen::Vector4f::Zero ();
        Eigen::Vector3f translation;
        Eigen::Vector3f rotation;

        // Pose messages
	geometry_msgs::PoseStamped::Ptr pose_msg;
        geometry_msgs::PoseStamped::Ptr last_pose;
        geometry_msgs::PoseStamped::Ptr d;

        // Clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input {new pcl::PointCloud<pcl::PointXYZRGB>()};
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cropbox {new pcl::PointCloud<pcl::PointXYZRGB>()};
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel {new pcl::PointCloud<pcl::PointXYZRGB>()};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subt {new pcl::PointCloud<pcl::PointXYZRGB>()};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_cloud {new pcl::PointCloud<pcl::PointXYZRGB>()};

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZRGB>()};
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cache {new pcl::PointCloud<pcl::PointXYZRGB>()};
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_buffer {new pcl::PointCloud<pcl::PointXYZRGB>()};
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_buffer_process {new pcl::PointCloud<pcl::PointXYZRGB>()};
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_outsm {new pcl::PointCloud<pcl::PointXYZRGBNormal>()};
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_mls {new pcl::PointCloud<pcl::PointXYZRGBNormal>};
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_mls_buffer {new pcl::PointCloud<pcl::PointXYZRGBNormal>};
        pcl::PolygonMesh triangles;
        pcl::PolygonMesh triangles_buffer;
        pcl::PolygonMesh cloud_out_tri;

        // Other variables
        std::string File_Path;
	int session_counter = 1;
	int last_size = 0;
	float PI = 3.141596;
        float a=0.01;
        int cont=0;
        float b=0.005;
        bool first_loop = true;
        bool SaveMap_mode = false;
	clock_t start, end = clock();
  };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ns_3DMNE::Class_Triangulation_Node,nodelet::Nodelet);
