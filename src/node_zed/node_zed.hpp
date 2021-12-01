// ROS Initializer
//#include "init.cpp"

//#include "sl_tools.h"
//#include "sl_tools.cpp"

// ZED Camera Library
#include <sl/Camera.hpp>

// ROS Library
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_srvs/Empty.h>

// Messages
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
//#include <mesh_msgs/MeshGeometryStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ROS Services
#include <node_zed/start_remote_stream.h>
#include <node_zed/stop_remote_stream.h>

#include <mutex>
#include <condition_variable>
#include <stdio.h>

/*
#include "pcl/point_types.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <pcl/point_cloud.h>


#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl/filters/passthrough.h>

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZRGB>()};
pcl::PCLPointCloud2 cloud;
*/


using namespace sl;
using namespace std;

namespace node_zed_nodelet{

// Coordinate frames

// Initialize state variables
ERROR_CODE openning_status;
ERROR_CODE err_pos_track; // POSITIONAL_TRACKING_STATE::OFF
ERROR_CODE err_spa_map;
POSITIONAL_TRACKING_STATE odom_state;
POSITIONAL_TRACKING_STATE pose_state;

// Transformation validation variables
bool Sensor2Base_Transf_Valid = true;
bool Sensor2Camera_Transf_Valid = true;
bool Camera2Base_Transf_Valid = true;

// Initialize varibles for ROS messages
geometry_msgs::PoseStamped pose_msg;
nav_msgs::Odometry odom_msg;
std_msgs::Int8 odom_confidence;
nav_msgs::PathPtr Pose_Path_msg;
nav_msgs::PathPtr Odom_Path_msg;
sensor_msgs::Imu imu_msg;
sensor_msgs::PointCloud2Ptr fused_pc_msg;
sensor_msgs::PointCloud2Ptr pc_msg;
//mesh_msgs::MeshGeometryStampedPtr mesh_msg;
    // State variables
std_msgs::String pose_state_msg;
std_msgs::String odom_state_msg;

// Declaration TFs
tf2::Transform Map2Odom_Transf;         // Coordinates of the odometry frame in map frame
tf2::Transform Odom2Base_Transf;        // Coordinates of the base in odometry frame
tf2::Transform Map2Base_Transf;         // Coordinates of the base in map frame
tf2::Transform Map2Camera_Transf;       // Coordinates of the camera in base frame
tf2::Transform Sensor2Base_Transf;      // Coordinates of the base frame in sensor frame
tf2::Transform Camera2Sensor_Transf;    // Coordinates of the camera frame in sensor frame
tf2::Transform Base2Camera_Transf;      // Coordinates of the base frame in camera frame
tf2::Transform imu_tf;

// TF Buffer (TF Listener)
tf2_ros::Buffer tfBuffer;

Transform Init_Pose;

// Thread Sync
std::mutex CamDataMutex;
std::mutex PcMutex;
std::condition_variable PcDataReadyCondVar;
bool PcDataReady;

// Positional Tracking variables
std::vector<geometry_msgs::PoseStamped> Odom_Path;
std::vector<geometry_msgs::PoseStamped> Pose_Path;

// Variables declaration
//CameraInformation cam_info;
Mat svo_left_image;
Mat svo_right_image;
Mat Point_Cloud;
FusedPointCloud fused_map;
Mesh mesh_map;
Pose zed_pose;
Pose zed_odom;
SensorsData sensors_data;
bool open_from_svo = true;
bool end_svo = false;
bool start_moving = false;
bool mesh = false;
bool stop = false;
int cont = 0;
int timer = 0;
int svo_position = 0;
float baseline = 0.12;
float z_camera_initial = 1.366;
uint max_depth_perception = 20;
uint map_max_depth = 7;
double map_resolution = 0.03;
tf2::Vector3 Left_Camera_Origin (0.0, baseline/2, 0.0);

string name = "/zed2/Zed_ROS_Class/";

bool Streaming = false;

class Zed_ROS_Class : public nodelet::Nodelet{
    public:
        Zed_ROS_Class(); // Constructor
        virtual ~Zed_ROS_Class(); // Destructor
        virtual void onInit();
        
    protected:
        void readParameters();
        void main_function();
        void initTransforms();
        //void readParameters();
        //Transform set_initial_pose(float x0, float y0, float z0, float r0, float p0, float yaw0);
        Transform set_initial_pose(vector<float> pInitialBasePose);
        nav_msgs::Odometry construct_Odom(Pose zed_odom);
        nav_msgs::PathPtr construct_Odom_Path();
        geometry_msgs::PoseStamped construct_Pose();
        nav_msgs::PathPtr construct_Pose_Path();
        sensor_msgs::Imu generate_imu_msg (SensorsData sensors_data);
        sensor_msgs::PointCloud2Ptr generate_pointcloud_msg (Mat Point_Cloud);
        sensor_msgs::PointCloud2Ptr generate_fused_pc_msg (FusedPointCloud pc_input);
        //mesh_msgs::MeshGeometryStampedPtr generate_mesh_msg (Mesh mesh_map);
        geometry_msgs::Transform generate_imu_tf_msg(Translation tr, Orientation rot);
        bool start_moving_func (sl::SensorsData sensors_data, bool start_moving);
        bool getSensor2Base_Transf();
        bool getCamera2Sensor_Transf();
        bool getBase2Camera_Transf();
        tf2::Transform getCamera2Odom_Transf();
        tf2::Transform generate_imu_tf (SensorsData sensors_data);
        void publish_odom_frame (tf2::Transform Odom_Transf);
        void publish_base_frame (tf2::Transform Odom_Transf);
        void publish_camera_frame (tf2::Transform Camera_Transf);
        void publish_gps_frame (tf2::Transform Camera_Transf);
        void publish_leftsensor_frame (tf2::Transform Camera_Transf);
        void publish_imu_frame (tf2::Transform Imu_Transf);
        std::vector<std::string> split_string(const std::string& s, char seperator);
        //void main_function();

        bool stop_serv(std_srvs::Empty::Request &req ,std_srvs::Empty::Response &resp);
        bool start_remote_stream(node_zed::start_remote_stream::Request& req, node_zed::start_remote_stream::Response& res);
        bool stop_remote_stream(node_zed::stop_remote_stream::Request& req, node_zed::stop_remote_stream::Response& res);

    private:
        // Initialize ROS
        ros::NodeHandle nh;
        ros::NodeHandle nh_priv;

            // ROS Publishers
        ros::Publisher pose_pub = nh_priv.advertise<geometry_msgs::PoseStamped>("zed/pose", 1);
        ros::Publisher pose_with_cov_pub = nh_priv.advertise<geometry_msgs::PoseWithCovarianceStamped>("zed/pose_with_covariance", 1);
        ros::Publisher odom_pub = nh_priv.advertise<nav_msgs::Odometry>("zed/odom", 1);
        ros::Publisher odom_conf_pub = nh_priv.advertise<std_msgs::Int8>("zed/odom_confidence", 1);
        ros::Publisher pose_path = nh_priv.advertise<nav_msgs::Path>("zed/pose_path", 1);
        ros::Publisher odom_path = nh_priv.advertise<nav_msgs::Path>("zed/odom_path", 1);
        ros::Publisher imu_pub = nh_priv.advertise<sensor_msgs::Imu>("zed/imu", 1);
        ros::Publisher imu_tf_pub = nh_priv.advertise<geometry_msgs::Transform>("zed/imu_tf", 1);
        ros::Publisher pc_pub = nh_priv.advertise<sensor_msgs::PointCloud2>("zed/point_cloud", 1);
        ros::Publisher fused_pc_pub = nh_priv.advertise<sensor_msgs::PointCloud2>("zed/point_cloud_fused", 1);
        //ros::Publisher mesh_pub = nh_priv.advertise<mesh_msgs::MeshGeometryStamped>("zed/mesh", 1);
                // State publishers
        ros::Publisher pose_state_pub = nh_priv.advertise<std_msgs::String>("zed/Pose_State",1);
        ros::Publisher odom_state_pub = nh_priv.advertise<std_msgs::String>("zed/Odom_State",1);   

        ros::ServiceServer stop_node;
        ros::ServiceServer Start_Remote_Stream;
        ros::ServiceServer Stop_Remote_Stream;

        // ZED Object
        Camera zed;
	InitParameters init_params;
        PositionalTrackingParameters tracking_parameters;
	SpatialMappingParameters mapping_parameters;
	// Launch File Parameters
        string pCameraName;
        string pCameraModel;
        sl::MODEL pZedCameraModel;
        RESOLUTION pCameraResol;
        sl::DEPTH_MODE pDepthMode;
        double pCameraMaxDepth;
        double pCameraMinDepth;
        int pCameraFrameRate;
        int pGPUid;
        int pZEDid;
        int pDepthStabilization;
        string pRemoteStreamAddress;
        string pSvoFilepath;
        vector<float> pInitialBasePose;
        string pAreaMemPath;
        bool pAreaMemory;
        bool pImuFusion;
        bool pMappingEnabled;
        float pMappingResolution;
        float pMaxMapDepth;
        // Coordinate Frames
        string pMapFrameId;
        string pOdometryFrameId;
        string pBaseFrameId;
        string pCameraFrameId;
        string pImuFrameId;
        string pLeftCameraFrameId;
        string pLeftCameraOpticalFrameId;
        string pRightCameraFrameId;
        string pRightCameraOpticalFrameId;
        string pDepthFrameId;
        string pDepthOptFrameId;
        string pPointCloudFrameId;
        // TF Broadcasting
        bool pPublish_Tf_Frames = true;
        bool pPublishMapTf;
        // Dynamic Parameters
        int pCamBrightness = 4;
        int pCamContrast = 4;
        int pCamHue = 0;
        int pCamSaturation = 4;
        int pCamSharpness = 3;
        int pCamGamma = 1;
        bool pCamAutoExposure = true;
        int pCamGain = 100;
        int pCamExposure = 100;
        bool pCamAutoWB = true;
        int pCamWB = 4200;
        int pCamDepthConfidence = 100;
        int pCamDepthTextureConf = 100;
        double pVideoDepthFreq = 15.;
        double pPointCloudFreq = 15.;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener (tf2_ros::Buffer tfBuffer);
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(node_zed_nodelet::Zed_ROS_Class, nodelet::Nodelet);
