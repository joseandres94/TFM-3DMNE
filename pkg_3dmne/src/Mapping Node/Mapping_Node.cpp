#include "Mapping_Node.hpp"

namespace ns_3DMNE{

Class_Mapping_Node::Class_Mapping_Node() : Nodelet() {}

Class_Mapping_Node::~Class_Mapping_Node(){}

void Class_Mapping_Node::onInit(){
    NODELET_INFO_STREAM("Initializing nodelet...");

	ros::NodeHandle nh = getMTNodeHandle();
	ros::NodeHandle nh_priv = getMTPrivateNodeHandle();

	readParameters();
	initTransforms();

    Start_Remote_Stream = nh_priv.advertiseService("Start_remote_stream", &Class_Mapping_Node::Start_remote_stream, this);
	Stop_Remote_Stream = nh_priv.advertiseService("Stop_remote_stream", &Class_Mapping_Node::Stop_remote_stream, this);
	Start_3D_Mapping = nh_priv.advertiseService("Start_3d_mapping", &Class_Mapping_Node::Start_3d_mapping, this);
	Stop_3D_Mapping = nh_priv.advertiseService("Stop_3d_mapping", &Class_Mapping_Node::Stop_3d_mapping, this);
	Save_3D_Map = nh_priv.advertiseService("Save_3d_map", &Class_Mapping_Node::Save_3d_map, this);
	Reset_Odometry = nh_priv.advertiseService("Reset_odometry", &Class_Mapping_Node::Reset_odometry, this);
	Reset_Session = nh_priv.advertiseService("Reset_session", &Class_Mapping_Node::Reset_session, this);
	Stop_Node_Service = nh_priv.advertiseService("Stop_node_serv", &Class_Mapping_Node::Stop_node_serv, this);

	// Initialize parameters
		// Initial Parameters
	init_params.coordinate_units = UNIT::METER;
	init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
	NODELET_INFO_STREAM(" * Camera coordinate system\t-> " << sl::toString(init_params.coordinate_system));
	init_params.depth_mode = static_cast<sl::DEPTH_MODE>(pDepthMode); // Set the depth mode to QUALITY
	init_params.depth_minimum_distance = pCameraMinDepth;
	init_params.depth_maximum_distance = pCameraMaxDepth;      // Set the maximum depth perception distance to 20m

		// Positional Tracking Parameters
	Initial_Pose = set_pose(pInitialBasePose);
	tracking_parameters.initial_world_transform = Initial_Pose;
	tracking_parameters.enable_pose_smoothing = true;
	tracking_parameters.enable_area_memory = pAreaMemory;
	tracking_parameters.enable_imu_fusion = pImuFusion;
	//tracking_parameters.area_file_path = (pRoute+pAreaMemPath).c_str();

		// Spatial Mapping Parameters
	if (pMappingEnabled){
		// Setting Resolution
		float low_Res = mapping_parameters.allowed_resolution.first;
		float high_Res = mapping_parameters.allowed_resolution.second;
		if(pMappingResolution < low_Res){
			NODELET_WARN_STREAM("Mapping resolution value (" << pMappingResolution << "m) is lower than allowed. Fixed automatically to " << low_Res << "m");
			pMappingResolution = low_Res;
		}
		if(pMappingResolution > high_Res){
			NODELET_WARN_STREAM("Mapping resolution value (" << pMappingResolution << "m) is higher than allowed. Fixed automatically to " << high_Res << "m");
			pMappingResolution = high_Res;
		}
		mapping_parameters.resolution_meter = pMappingResolution;

		// Setting Range
		float low_Range = mapping_parameters.allowed_range.first;
		float high_Range = mapping_parameters.allowed_range.second;
		if(pMaxMapDepth < low_Range){
			NODELET_WARN_STREAM("Mapping resolution value (" << pMaxMapDepth << "m) is lower than allowed. Fixed automatically to " << low_Range << "m");
			pMaxMapDepth = low_Range;
		}
		if(pMaxMapDepth > high_Range){
			NODELET_WARN_STREAM("Mapping resolution value (" << pMaxMapDepth << "m) is higher than allowed. Fixed automatically to " << high_Range << "m");
			pMaxMapDepth = high_Range;
		}
		mapping_parameters.range_meter = pMaxMapDepth;
		
			// Fused or meshed map
		if (pMeshEnabled){
			// Set mapping with mesh output
			mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
			mapping_parameters.save_texture = true ;  // Scene texture will be recorded
		}
		else{
			// or select point cloud output
			mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
		}
	}

	// Initialization transformation listener
    tfBuffer.reset(new tf2_ros::Buffer);
    tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));

	if (!pSvoFilepath.empty() || !pRemoteStreamAddress.empty()){
		if (!pSvoFilepath.empty()){
			init_params.input.setFromSVOFile(pSvoFilepath.c_str());
			init_params.svo_real_time_mode = false;
		}else if (!pRemoteStreamAddress.empty()){
			std::vector<std::string> configStream = split_string(pRemoteStreamAddress, ':');
            sl::String ip = sl::String(configStream.at(0).c_str());
			if (configStream.size() == 2){
				init_params.input.setFromStream(ip, atoi(configStream.at(1).c_str()));
			}else{
				init_params.input.setFromStream(ip);
			}
		}
	}
	
	// Open the camera
	NODELET_INFO_STREAM(" *** Opening " << sl::toString(pZedCameraModel) << "...");
	openning_status = ERROR_CODE::CAMERA_NOT_DETECTED;
	while (openning_status != ERROR_CODE::SUCCESS){
		openning_status = zed.open(init_params);
		NODELET_INFO_STREAM("ZED connection \t\t\t-> " << sl::toString(openning_status));
	}

    main_function();
}

void Class_Mapping_Node::readParameters(){
	// Camera Parameters
	NODELET_INFO_STREAM(" * Reading Parameters *");
	nh_priv.getParam(name + "general/camera_name", pCameraName);
    NODELET_INFO_STREAM(" * Camera Name\t\t\t-> " << pCameraName.c_str());

	nh_priv.getParam(name + "general/camera_model", pCameraModel);
    if (pCameraModel == "zed"){
        pZedCameraModel = sl::MODEL::ZED;
        NODELET_INFO_STREAM(" * Camera Model by param\t-> " <<  pCameraModel);
    }else if (pCameraModel == "zedm"){
        pZedCameraModel = sl::MODEL::ZED_M;
        NODELET_INFO_STREAM(" * Camera Model by param\t-> " <<  pCameraModel);
    }else if (pCameraModel == "zed2"){
        pZedCameraModel = sl::MODEL::ZED2;
        NODELET_INFO_STREAM(" * Camera Model by param\t-> " <<  pCameraModel);
    }else{
        NODELET_INFO_STREAM(" * Camera model not valid: " <<  pCameraModel);
    }

	int resol;
	nh_priv.getParam("general/resolution", resol);
	CameraResol = static_cast<sl::RESOLUTION>(resol);
	printf("Resolution: %s.\n", sl::toString(CameraResol).c_str());

	nh_priv.getParam("general/grab_frame_rate", CameraFrameRate);
	printf("Camera Name: %s.\n", CameraName.c_str());

	nh_priv.getParam("general/gpu_id", GPUid);
	printf("GPU id: %i.\n", GPUid);

	nh_priv.getParam("general/zed_id", ZEDid);
	printf("Camera Name: %i.\n", ZEDid);

	// Depth Parameters
	int depth_mode;
    nh_priv.getParam(name + "depth/quality", depth_mode);
    pDepthMode = static_cast<sl::DEPTH_MODE>(depth_mode);
	NODELET_INFO_STREAM(" * Depth quality\t\t-> " << sl::toString(pDepthMode).c_str());

	nh_priv.getParam("depth/depth_stabilization", pDepthStabilization);
	printf(" * Depth Stabilization -> ")
    NODELET_INFO_STREAM(" * Depth Stabilization\t\t-> " << (mDepthStabilization ? "ENABLED" : "DISABLED"));

	nh_priv.getParam(name + "depth/min_depth", pCameraMinDepth);
    NODELET_INFO_STREAM(" * Minimum depth\t\t-> " <<  pCameraMinDepth << " m");

	nh_priv.getParam(name + "depth/max_depth", pCameraMaxDepth);
    NODELET_INFO_STREAM(" * Maximum depth\t\t-> " << pCameraMaxDepth << " m");

	// Tracking
	nh_priv.getParam(name + "pos_tracking/initial_base_pose", pInitialBasePose);
	nh_priv.getParam(name + "pos_tracking/area_memory_db_path", pAreaMemPath);
    NODELET_INFO_STREAM(" * Odometry DB path\t\t-> " << pAreaMemPath.c_str());
    nh_priv.param<bool>(name + "pos_tracking/area_memory", pAreaMemory, false);
    NODELET_INFO_STREAM(" * Spatial Memory\t\t-> " << (pAreaMemory ? "ENABLED" : "DISABLED"));
    nh_priv.param<bool>(name + "pos_tracking/imu_fusion", pImuFusion, true);
    NODELET_INFO_STREAM(" * IMU Fusion\t\t\t-> " << (pImuFusion ? "ENABLED" : "DISABLED"));

	// Mapping
	nh_priv.param<bool>(name + "mapping/mapping_enabled", pMappingEnabled, false);
    NODELET_INFO_STREAM(" * Mapping\t\t\t-> " << (pMappingEnabled ? "ENABLED" : "DISABLED"));
	if (pMappingEnabled){
		nh_priv.getParam(name + "mapping/resolution", pMappingResolution);
        	NODELET_INFO_STREAM(" * Mapping resolution\t\t-> " << pMappingResolution << " m" );

		nh_priv.getParam(name + "mapping/max_mapping_range", pMaxMapDepth);
		NODELET_INFO_STREAM(" * Mapping max range\t\t-> " << pMaxMapDepth << " m" << ((pMaxMapDepth < 0.0)?" [AUTO]":""));

		nh_priv.param<bool>(name + "mapping/mesh", pMeshEnabled, false);
    		NODELET_INFO_STREAM(" * Mesh\t\t\t\t-> " << (pMeshEnabled ? "ENABLED" : "DISABLED"));

		nh_priv.getParam(name + "general/route_map", pRoute);
                NODELET_INFO_STREAM(" * Route to save map\t\t-> " << pRoute);
	}

	// SVO File
	nh_priv.param<std::string>(name + "svo_file", pSvoFilepath, std::string());
    NODELET_INFO_STREAM(" * SVO input file: \t\t-> " << pSvoFilepath.c_str());

    // Remote Streaming
	nh_priv.param<std::string>(name + "stream", pRemoteStreamAddress, std::string());

	// Coordinate frames
	nh_priv.param<std::string>(name + "pos_tracking/map_frame", pMapFrameId, "map");
	nh_priv.param<std::string>(name + "pos_tracking/odometry_frame", pOdometryFrameId, "odom");
	nh_priv.param<std::string>(name + "general/base_frame", pBaseFrameId, "base_link");
	pCameraFrameId = pCameraName + "_camera_center";
    pImuFrameId = pCameraName + "_imu_link";
    pLeftCameraFrameId = pCameraName + "_left_camera_frame";
    pLeftCameraOpticalFrameId = pCameraName + "_left_camera_optical_frame";
    pRightCameraFrameId = pCameraName + "_right_camera_frame";
    pRightCameraOpticalFrameId = pCameraName + "_right_camera_optical_frame";

    pPointCloudFrameId = pCameraFrameId;
	pDepthFrameId = pLeftCameraFrameId;
    pDepthOptFrameId = pLeftCameraOpticalFrameId;

    NODELET_INFO_STREAM(" * Map_frame\t\t\t-> " << pMapFrameId);
    NODELET_INFO_STREAM(" * Odometry_frame\t\t-> " << pOdometryFrameId);
    NODELET_INFO_STREAM(" * Base_frame\t\t\t-> " << pBaseFrameId);
    NODELET_INFO_STREAM(" * Camera_frame\t\t\t-> " << pCameraFrameId);
    NODELET_INFO_STREAM(" * Imu_link\t\t\t-> " << pImuFrameId);
    NODELET_INFO_STREAM(" * Left_camera_frame\t\t-> " << pLeftCameraFrameId);
    NODELET_INFO_STREAM(" * Left_camera_optical_frame\t-> " << pLeftCameraOpticalFrameId);
    NODELET_INFO_STREAM(" * Right_camera_frame\t\t-> " << pRightCameraFrameId);
    NODELET_INFO_STREAM(" * Right_camera_optical_frame\t-> " << pRightCameraOpticalFrameId);
    NODELET_INFO_STREAM(" * Depth_frame\t\t\t-> " << pDepthFrameId);
    NODELET_INFO_STREAM(" * Depth_optical_frame\t\t-> " << pDepthOptFrameId);

	// TF broadcasting
	nh_priv.param<bool>(name + "pos_tracking/publish_tf", pPublish_Tf_Frames, true);
	NODELET_INFO_STREAM(" * Broadcast odometry TF\t-> " << (pPublish_Tf_Frames ? "ENABLED" : "DISABLED"));
	nh_priv.param<bool>(name + "pos_tracking/publish_map_tf", pPublishMapTf, true);
	NODELET_INFO_STREAM(" * Broadcast map pose TF\t-> " << (pPublish_Tf_Frames ? (pPublishMapTf ? "ENABLED" : "DISABLED") : "DISABLED"));

	// Dynamic parameters
	nh_priv.getParam(name + "depth_confidence", pCamDepthConfidence);
    NODELET_INFO_STREAM(" * [DYN] Depth confidence\t-> " << pCamDepthConfidence);

    nh_priv.getParam(name + "depth_texture_conf", pCamDepthTextureConf);
    NODELET_INFO_STREAM(" * [DYN] Depth texture conf.\t-> " << pCamDepthTextureConf);
    
    nh_priv.getParam(name + "pub_frame_rate", pVideoDepthFreq);
    NODELET_INFO_STREAM(" * [DYN] pub_frame_rate\t\t-> " << pVideoDepthFreq << " Hz");
    
    nh_priv.getParam(name + "point_cloud_freq", pPointCloudFreq);
    NODELET_INFO_STREAM(" * [DYN] point_cloud_freq\t-> " << pPointCloudFreq << " Hz");
    
    nh_priv.getParam(name + "brightness", pCamBrightness);
    NODELET_INFO_STREAM(" * [DYN] brightness\t\t-> " << pCamBrightness);
    
    nh_priv.getParam(name + "contrast", pCamContrast);
    NODELET_INFO_STREAM(" * [DYN] contrast\t\t-> " << pCamContrast);
    
    nh_priv.getParam(name + "hue", pCamHue);
    NODELET_INFO_STREAM(" * [DYN] hue\t\t\t-> " << pCamHue);
    
    nh_priv.getParam(name + "saturation", pCamSaturation);
    NODELET_INFO_STREAM(" * [DYN] saturation\t\t-> " << pCamSaturation);
    
    nh_priv.getParam(name + "sharpness", pCamSharpness);
    NODELET_INFO_STREAM(" * [DYN] sharpness\t\t-> " << pCamSharpness);
    
    nh_priv.getParam(name + "gamma", pCamGamma);
    NODELET_INFO_STREAM(" * [DYN] gamma\t\t\t-> " << pCamGamma);
	
    nh_priv.getParam(name + "auto_exposure_gain", pCamAutoExposure);
    NODELET_INFO_STREAM(" * [DYN] auto_exposure_gain\t-> " << (pCamAutoExposure ? "ENABLED" : "DISABLED"));
    
    nh_priv.getParam(name + "gain", pCamGain);
    nh_priv.getParam(name + "exposure", pCamExposure);
    if(!pCamAutoExposure) {
        NODELET_INFO_STREAM("  * [DYN] gain\t\t-> " << pCamGain);
        NODELET_INFO_STREAM("  * [DYN] exposure\t\t-> " << pCamExposure);
    }
    nh_priv.getParam(name + "auto_whitebalance", pCamAutoWB);
    NODELET_INFO_STREAM(" * [DYN] auto_whitebalance\t-> " << (pCamAutoWB ? "ENABLED" : "DISABLED"));
    nh_priv.getParam(name + "whitebalance_temperature", pCamWB);
    if(!pCamAutoWB) {
        NODELET_INFO_STREAM("  * [DYN] whitebalance_temperature\t\t-> " << pCamWB);
	}
}

void Class_Mapping_Node::initTransforms(){
	// Initialization TFs
	Map2Odom_Transf.setIdentity();
	Map2Base_Transf.setIdentity();
	Odom2Base_Transf.setIdentity();
	Base2Sensor_Transf.setIdentity();	
	Camera2Sensor_Transf.setIdentity();
	Camera2Sensor_Transf.setOrigin(Left_Camera_Origin);
	Base2Camera_Transf.setIdentity();
}

Transform Class_Mapping_Node::set_pose(vector<float> Pose){
	Pose.resize(6);
	tf2::Vector3 trasl(Pose[0], Pose[1], Pose[2]);
	tf2::Quaternion quat;
	quat.setRPY(Pose[3], Pose[4], Pose[5]);
	
	tf2::Transform pose_tf;
	pose_tf.setOrigin(trasl);
	pose_tf.setRotation(quat);

	//pose_tf = pose_tf * Sensor2Base_Transf.inverse();
	pose_tf = pose_tf * Base2Sensor_Transf.inverse();

	sl::float3 trasl_vector;
	trasl_vector[0] = pose_tf.getOrigin().x();
	trasl_vector[1] = pose_tf.getOrigin().y();
	trasl_vector[2] = pose_tf.getOrigin().z();

	sl::float4 quat_vector;
	quat_vector[0] = pose_tf.getRotation().x();
	quat_vector[1] = pose_tf.getRotation().y();
	quat_vector[2] = pose_tf.getRotation().z();
	quat_vector[3] = pose_tf.getRotation().w();

	Translation position (trasl_vector);
	Orientation orientation (quat_vector);
	Transform pose_sl;
	pose_sl.setTranslation(position);
	pose_sl.setOrientation(orientation);

	return pose_sl;
}

nav_msgs::Odometry Class_Mapping_Node::construct_Odom_msg(Pose zed_odom){
	nav_msgs::Odometry odom_msg;
	geometry_msgs::Transform odom2base = tf2::toMsg(Odom2Base_Transf);

	// Header
	odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = pOdometryFrameId;
	odom_msg.child_frame_id = pBaseFrameId;

	// Pose
	odom_msg.pose.pose.position.x = odom2base.translation.x;
    odom_msg.pose.pose.position.y = odom2base.translation.y;
	odom_msg.pose.pose.position.z = odom2base.translation.z;
	// Pose Covariance
	for (int c = 0; c<36; c++){
		if (!isnan(zed_odom.pose_covariance[c]) && !isinf(zed_odom.pose_covariance[c])){
			odom_msg.pose.covariance[c] = zed_odom.pose_covariance[c];
		}else{
			odom_msg.pose.covariance[c] = 0;
		}
	}
	
	// Orientation
    odom_msg.pose.pose.orientation.x = odom2base.rotation.x;
	odom_msg.pose.pose.orientation.y = odom2base.rotation.y;
	odom_msg.pose.pose.orientation.z = odom2base.rotation.z;
	odom_msg.pose.pose.orientation.w = odom2base.rotation.w;

	return odom_msg;
}

nav_msgs::PathPtr Class_Mapping_Node::construct_Odom_Path_msg(){
	geometry_msgs::Transform odom2base = tf2::toMsg(Odom2Base_Transf);
	
	geometry_msgs::PoseStamped odom_path_msg;
	odom_path_msg.header = odom_msg.header;
	odom_path_msg.pose = odom_msg.pose.pose;

	Odom_Path.push_back(odom_path_msg);

	nav_msgs::PathPtr Odom_Path_msg = boost::make_shared<nav_msgs::Path>();
	Odom_Path_msg->header.frame_id = pOdometryFrameId; //pMapFrameId;
	Odom_Path_msg->header.stamp = ros::Time::now();
	Odom_Path_msg->poses = Odom_Path;

	return Odom_Path_msg;
}

geometry_msgs::PoseStamped Class_Mapping_Node::construct_Pose_msg(tf2::Transform base_pose){
	geometry_msgs::Transform base2frame = tf2::toMsg(base_pose);

	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = pMapFrameId;
	
	geometry_msgs::Pose pose;
	pose.position.x = base2frame.translation.x;
    pose.position.y = base2frame.translation.y;
    pose.position.z = base2frame.translation.z;

    pose.orientation.x = base2frame.rotation.x;
    pose.orientation.y = base2frame.rotation.y;
    pose.orientation.z = base2frame.rotation.z;
    pose.orientation.w = base2frame.rotation.w;

	geometry_msgs::PoseStamped pose_msg;

	pose_msg.header = header;
	pose_msg.pose = pose;

	return pose_msg;
}

nav_msgs::PathPtr Class_Mapping_Node::construct_Pose_Path_msg(){
	geometry_msgs::Transform map2base = tf2::toMsg(Map2Base_Transf);

	Pose_Path.push_back(pose_msg);

	nav_msgs::PathPtr Pose_Path_msg = boost::make_shared<nav_msgs::Path>();
	Pose_Path_msg->header.frame_id = pMapFrameId;
	Pose_Path_msg->header.stamp = ros::Time::now();
	Pose_Path_msg->poses = Pose_Path;

	return Pose_Path_msg;
}

sensor_msgs::Imu Class_Mapping_Node::generate_imu_msg (SensorsData sensors_data){

	sensor_msgs::Imu imu;

	// header
	imu.header.stamp = ros::Time::now();
	//imu.header.frame_id = "zed2_imu_link";
	imu.header.frame_id = pImuFrameId;

	// orientation
	imu.orientation.x =  sensors_data.imu.pose.getOrientation().x;
	imu.orientation.y =  sensors_data.imu.pose.getOrientation().y;
	imu.orientation.z =  sensors_data.imu.pose.getOrientation().z;
	imu.orientation.w =  sensors_data.imu.pose.getOrientation().w;
	memcpy(&imu.orientation_covariance, &sensors_data.imu.pose_covariance, 9 * sizeof(float));

	// angular velocity
	imu.angular_velocity.x =  sensors_data.imu.angular_velocity.x;
	imu.angular_velocity.y =  sensors_data.imu.angular_velocity.y;
	imu.angular_velocity.z =  sensors_data.imu.angular_velocity.z;

	memcpy(&imu.angular_velocity_covariance, &sensors_data.imu.angular_velocity_covariance, 9*sizeof(float));
	
	// linear acceleration
	imu.linear_acceleration.x =  sensors_data.imu.linear_acceleration.x;
	imu.linear_acceleration.y =  sensors_data.imu.linear_acceleration.y;
	imu.linear_acceleration.z =  sensors_data.imu.linear_acceleration.z;

	memcpy(&imu.linear_acceleration_covariance, &sensors_data.imu.linear_acceleration_covariance, 9*sizeof(float));
	
	return imu;
}

sensor_msgs::PointCloud2Ptr Class_Mapping_Node::generate_pointcloud_msg (Mat Point_Cloud){

	sensor_msgs::PointCloud2Ptr pc_msg = boost::make_shared<sensor_msgs::PointCloud2>();
	
	pc_msg->header.stamp = ros::Time::now();
	pc_msg->header.frame_id = pPointCloudFrameId;

	pc_msg->is_bigendian = false;
    pc_msg->is_dense = false;

	pc_msg->height = Point_Cloud.getHeight();
	pc_msg->width = Point_Cloud.getWidth();

	size_t num_of_elements = Point_Cloud.getHeight() * Point_Cloud.getWidth();
	
	sensor_msgs::PointCloud2Modifier modifier(*pc_msg);
	modifier.setPointCloud2Fields(4,
									"x", 1, sensor_msgs::PointField::FLOAT32,
									"y", 1, sensor_msgs::PointField::FLOAT32,
									"z", 1, sensor_msgs::PointField::FLOAT32,
									"rgb", 1, sensor_msgs::PointField::FLOAT32);
	

	sl::Vector4<float>* cloud_pts = Point_Cloud.getPtr<sl::float4>();
    float * ptCloudPtr = (float*)(&pc_msg->data[0]);

    // We can do a direct memcpy since data organization is the same
    memcpy(ptCloudPtr, (float*)cloud_pts, 4 * num_of_elements * sizeof(float));

	return pc_msg;				  
}

sensor_msgs::PointCloud2Ptr Class_Mapping_Node::generate_fused_pc_msg (FusedPointCloud pc_input){

	sensor_msgs::PointCloud2Ptr fused_pc_msg = boost::make_shared<sensor_msgs::PointCloud2>();

	fused_pc_msg->header.stamp = ros::Time::now();
	fused_pc_msg->header.frame_id = pMapFrameId;

	fused_pc_msg->height = 1;
	fused_pc_msg->width = pc_input.vertices.size();

	sensor_msgs::PointCloud2Modifier modifier(*fused_pc_msg);
	modifier.setPointCloud2Fields(4,
									"x", 1, sensor_msgs::PointField::FLOAT32,
									"y", 1, sensor_msgs::PointField::FLOAT32,
									"z", 1, sensor_msgs::PointField::FLOAT32,
									"rgb", 1, sensor_msgs::PointField::FLOAT32);

	fused_pc_msg->data.resize(pc_input.getNumberOfPoints() * 4 * sizeof(float));
	float * pc_Ptr = (float *)(&fused_pc_msg->data[0]);

	for (int c = 0; c < pc_input.chunks.size(); c++){
		size_t chunkSize = pc_input.chunks[c].vertices.size();

		if (chunkSize > 0){
			float * cloud_pts  = (float *)(pc_input.chunks[c].vertices.data());
			memcpy(pc_Ptr, cloud_pts, 4 * chunkSize * sizeof(float));
			pc_Ptr += 4 * chunkSize;
		}
	}
	return fused_pc_msg;
}
/* Codigo para la publicacion del mapa en formato "Mesh". (No funciona)
mesh_msgs::MeshGeometryStampedPtr Class_Mapping_Node::generate_mesh_msg (Mesh mesh_map){
	
	int num_of_trian = mesh_map.getNumberOfTriangles();
	int num_of_verts = 0;
	for (int c = 0; c < mesh_map.chunks.size(); c++){
        num_of_verts += mesh_map.chunks[c].vertices.size();
	}

	mesh_msgs::MeshGeometryStampedPtr mesh = boost::make_shared<mesh_msgs::MeshGeometryStamped>();
	
	mesh->header.stamp = ros::Time::now();
    mesh->header.frame_id = pMapFrameId;

	mesh->mesh_geometry.vertices.resize (3 * sizeof(float) * num_of_verts);
	float * verts_Ptr = (float *)(&mesh->mesh_geometry.vertices);

	mesh->mesh_geometry.vertex_normals.resize (3 * sizeof(float) * num_of_verts);
	float * normals_Ptr = (float *)(&mesh->mesh_geometry.vertex_normals);

	mesh->mesh_geometry.faces.resize (3 * sizeof(int) * num_of_trian);
	int * faces_Ptr = (int *)(&mesh->mesh_geometry.faces);

	return mesh;
}
*/
geometry_msgs::Transform Class_Mapping_Node::generate_imu_tf_msg(Translation tr, Orientation rot){

	geometry_msgs::Transform imu_tf_msg;

	imu_tf_msg.translation.x = tr.x;
	imu_tf_msg.translation.y = tr.y;
	imu_tf_msg.translation.z = tr.z;
	imu_tf_msg.rotation.x = rot.ox;
	imu_tf_msg.rotation.y = rot.oy;
	imu_tf_msg.rotation.z = rot.oz;
	imu_tf_msg.rotation.w = rot.ow;

	return imu_tf_msg;
}

bool Class_Mapping_Node::start_moving_func (sl::SensorsData sensors_data, bool start_moving){

	if (sensors_data.imu.linear_acceleration.x >= 0.3 && start_moving == false)
		start_moving = true;

	return start_moving;
}

bool Class_Mapping_Node::getBase2Camera_Transf(){
	Base2Camera_Transf_Valid = false;

	try{
		// Save the transformation
		geometry_msgs::TransformStamped b2c = tfBuffer->lookupTransform(pCameraFrameId, pBaseFrameId, ros::Time(0));
        
		// Get the TF2 transformation
        tf2::fromMsg(b2c.transform, Base2Camera_Transf);

		double roll, pitch, yaw;
        tf2::Matrix3x3(Base2Camera_Transf.getRotation()).getRPY(roll, pitch, yaw);

	}catch(tf2::TransformException& ex){
		Base2Camera_Transf.setIdentity();
		return false;
	}

	Base2Camera_Transf_Valid = true;
	return true;
}

bool Class_Mapping_Node::getBase2Sensor_Transf(){
	Base2Sensor_Transf_Valid = false;
	
	try{
		// Save the transformation
		geometry_msgs::TransformStamped s2b = tfBuffer->lookupTransform(pDepthFrameId, pBaseFrameId, ros::Time(0));

		// Get the TF2 transformation
        tf2::fromMsg(s2b.transform, Base2Sensor_Transf);

		double roll, pitch, yaw;
        tf2::Matrix3x3(Base2Sensor_Transf.getRotation()).getRPY(roll, pitch, yaw);

	}catch(tf2::TransformException& ex){
		Base2Sensor_Transf.setIdentity();
		return false;
	}

	Base2Sensor_Transf_Valid = true;
	return true;
}

tf2::Transform Class_Mapping_Node::getCamera2Odom_Transf(){
	tf2::Transform cam2odom_transf;

	try{
		// Save the transformation
		geometry_msgs::TransformStamped c2o = tfBuffer->lookupTransform(pOdometryFrameId, pCameraFrameId, ros::Time(0));
        
		// Get the TF2 transformation
        tf2::fromMsg(c2o.transform, cam2odom_transf);

	}catch(tf2::TransformException& ex){
		cam2odom_transf.setIdentity();
	}

	return cam2odom_transf;
}

tf2::Transform Class_Mapping_Node::generate_imu_tf (SensorsData sensors_data){
	
	// quaternion to tf generation
	tf2::Quaternion imu_quat;
	imu_quat.setX(sensors_data.imu.pose.getOrientation()[0]);
	imu_quat.setY(sensors_data.imu.pose.getOrientation()[1]);
	imu_quat.setZ(sensors_data.imu.pose.getOrientation()[2]);
	imu_quat.setW(sensors_data.imu.pose.getOrientation()[3]);

	tf2::Transform cam2odom = getCamera2Odom_Transf();
	tf2::Quaternion cam2odom_quat = cam2odom.getRotation();

	tf2::Transform imu_pose;
	imu_pose.setIdentity();
	imu_pose.setRotation(imu_quat * cam2odom_quat.inverse());
	
	return imu_pose;
}

void Class_Mapping_Node::publish_pose_frame (tf2::Transform Pose_Transf){

	if (!Base2Sensor_Transf_Valid){
		Base2Sensor_Transf_Valid = getBase2Sensor_Transf();
	}

	if (!Base2Camera_Transf_Valid){
		Base2Camera_Transf_Valid = getBase2Camera_Transf();
	}

    geometry_msgs::TransformStamped transform;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pMapFrameId;
	transform.child_frame_id = pOdometryFrameId;

	transform.transform = tf2::toMsg (Pose_Transf);
	Transform_Pose_Broadcaster.sendTransform(transform);
}

void Class_Mapping_Node::publish_odom_frame (tf2::Transform Odom_Transf){

	if (!Base2Sensor_Transf_Valid){
		Base2Sensor_Transf_Valid = getBase2Sensor_Transf();
	}

	if (!Base2Camera_Transf_Valid){
		Base2Camera_Transf_Valid = getBase2Camera_Transf();
	}

    geometry_msgs::TransformStamped transform;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pOdometryFrameId;
	transform.child_frame_id = pBaseFrameId;

	transform.transform = tf2::toMsg (Odom_Transf);
	Transform_Odom_Broadcaster.sendTransform(transform);
}

void Class_Mapping_Node::publish_camera_frame (tf2::Transform Camera_Transf){

	if (!Base2Sensor_Transf_Valid){
		Base2Sensor_Transf_Valid = getBase2Sensor_Transf();
	}

	if (!Base2Camera_Transf_Valid){
		Base2Camera_Transf_Valid = getBase2Camera_Transf();
	}

    geometry_msgs::TransformStamped transform;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pBaseFrameId;
	transform.child_frame_id = pCameraFrameId;

	transform.transform = tf2::toMsg (Camera_Transf);
    Transform_Camera_Broadcaster.sendTransform(transform);
}

void Class_Mapping_Node::publish_gps_frame (tf2::Transform Camera_Transf){

    geometry_msgs::TransformStamped transform;
	tf2_ros::TransformBroadcaster br;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pBaseFrameId;
	transform.child_frame_id = "gps";
	transform.transform = tf2::toMsg (Camera_Transf);
    br.sendTransform(transform);
}

void Class_Mapping_Node::publish_leftsensor_frame (tf2::Transform Camera_Transf){

	if (!Base2Sensor_Transf_Valid){
		Base2Sensor_Transf_Valid = getBase2Sensor_Transf();
	}

	if (!Base2Camera_Transf_Valid){
		Base2Camera_Transf_Valid = getBase2Camera_Transf();
	}

    geometry_msgs::TransformStamped transform;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pCameraFrameId;
	transform.child_frame_id = pLeftCameraFrameId;

	transform.transform = tf2::toMsg (Camera_Transf);
    Transform_Left_Camera_Broadcaster.sendTransform(transform);
}

void Class_Mapping_Node::publish_imu_frame (tf2::Transform Imu_Transf){

	if (!Base2Sensor_Transf_Valid){
		Base2Sensor_Transf_Valid = getBase2Sensor_Transf();
	}

	if (!Base2Camera_Transf_Valid){
		Base2Camera_Transf_Valid = getBase2Camera_Transf();
	}

    geometry_msgs::TransformStamped transform;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pCameraFrameId;
	transform.child_frame_id = pImuFrameId;

	transform.transform = tf2::toMsg (Imu_Transf);
    Transform_Imu_Broadcaster.sendTransform(transform);
}

std::vector<std::string> Class_Mapping_Node::split_string (const std::string& s, char seperator){
	std::vector<std::string> output;
	std::string::size_type prev_pos = 0, pos = 0;

	while ((pos = s.find(seperator, pos)) != std::string::npos) {
		std::string substring(s.substr(prev_pos, pos - prev_pos));
		output.push_back(substring);
		prev_pos = ++pos;
	}

	output.push_back(s.substr(prev_pos, pos - prev_pos));
	return output;
}

void Class_Mapping_Node::Save_map(){
	
	NODELET_INFO_STREAM("  ** Saving Map **");

	if (pMeshEnabled ){
		NODELET_INFO_STREAM(" * Creating Mesh Map *");
		zed.extractWholeSpatialMap(mesh_map);
		// Process the mesh
		NODELET_INFO_STREAM("Applying filter.");
		mesh_map.filter(MeshFilterParameters::MESH_FILTER::MEDIUM); // Filter the mesh
		NODELET_INFO_STREAM("Applying texture.");
		mesh_map.applyTexture(MESH_TEXTURE_FORMAT::RGB); // Apply the texture
		NODELET_INFO_STREAM("Saving mesh.");
		sl::String path_mesh = (pRoute + "mesh" + std::to_string(session_counter) + ".obj").c_str();
		mesh_map.save(path_mesh, MESH_FILE_FORMAT::OBJ);
		NODELET_INFO_STREAM("Mesh saved.");
	}else if (!pMeshEnabled){
		NODELET_INFO_STREAM(" * Creating Point Cloud Map *");
		NODELET_INFO_STREAM("Saving fused point cloud.");
		sl::String path_pc = (pRoute + "fused_pc" + std::to_string(session_counter) + ".obj").c_str();
		fused_map.save(path_pc, MESH_FILE_FORMAT::OBJ);
		NODELET_INFO_STREAM("Fused point cloud saved.");
	}
	session_counter++;
}

// ROS Services functions
bool Class_Mapping_Node::Start_remote_stream (pkg_3dmne::start_remote_stream::Request& req, pkg_3dmne::start_remote_stream::Response& res){
	
	if (Streaming) {
        res.result = false;
        res.info = "SVO remote streaming was just active";
        return false;
    }

    sl::StreamingParameters params;

	params.codec = static_cast<sl::STREAMING_CODEC>(req.codec);
	params.port = req.port;
	params.bitrate = req.bitrate;
	params.gop_size = req.gop_size;
	params.adaptative_bitrate = req.adaptative_bitrate;
	
    if (params.gop_size < -1 || params.gop_size > 256) {	
        Streaming = false;

        res.result = false;
        res.info = "`gop_size` wrong (";
        res.info += params.gop_size;
        res.info += "). Remote streaming not started";

        NODELET_ERROR_STREAM(res.info);
        return false;
    }

    if (params.port % 2 != 0) {
		Streaming = false;

        res.result = false;
        res.info = "`port` must be an even number. Remote streaming not started";

        NODELET_ERROR_STREAM(res.info);
        return false;
    }
    ERROR_CODE err = zed.enableStreaming(params);

    if (err != sl::ERROR_CODE::SUCCESS) {
        Streaming = false;

        res.result = false;
        res.info = sl::toString(err).c_str();

        NODELET_ERROR_STREAM("Remote streaming not started (" << res.info << ")");

        return false;
    }
    Streaming = true;

    NODELET_INFO_STREAM("Remote streaming STARTED");

    res.result = true;
    res.info = "Remote streaming STARTED";
    cout<<res.info<<endl;
    return true;

}

bool Class_Mapping_Node::Stop_remote_stream (pkg_3dmne::stop_remote_stream::Request& req, pkg_3dmne::stop_remote_stream::Response& res) {
    if (Streaming) {
        zed.disableStreaming();
    }
    
    Streaming = false;
    NODELET_INFO_STREAM("SVO remote streaming STOPPED");

    res.done = true;
    return true;
}

bool Class_Mapping_Node::Start_3d_mapping (std_srvs::Empty::Request &req ,std_srvs::Empty::Response &resp){
	pMappingEnabled = true;

	mapping_parameters.range_meter = pMaxMapDepth; // Ver cuando mMaxMappingRange = -1 (AUTO)
	mapping_parameters.resolution_meter = pMappingResolution;
		// Fused or meshed map
	if (pMeshEnabled){
		// Set mapping with mesh output
		mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
		mapping_parameters.save_texture = true ;  // Scene texture will be recorded
	}
	else{
		// or select point cloud output
		mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
	}
}

bool Class_Mapping_Node::Stop_3d_mapping (std_srvs::Empty::Request &req ,std_srvs::Empty::Response &resp){
	pMappingEnabled = false;

	zed.disableSpatialMapping();
	NODELET_INFO_STREAM("Spatial Mapping disabled.");
	err_spa_map = ERROR_CODE::FAILURE;
}

bool Class_Mapping_Node::Save_3d_map (std_srvs::Empty::Request &req ,std_srvs::Empty::Response &resp){
	Save_map();
}

bool Class_Mapping_Node::Reset_odometry (std_srvs::Empty::Request &req ,std_srvs::Empty::Response &resp){
	reset_odom = true;
}

bool Class_Mapping_Node::Reset_session (std_srvs::Empty::Request &req ,std_srvs::Empty::Response &resp){
	NODELET_INFO_STREAM("* Restarting Mapping Session. *");
	Save_map();
	Stop_3d_mapping (req, resp);
	zed.disablePositionalTracking((pRoute+pAreaMemPath).c_str());
	err_pos_track = ERROR_CODE::FAILURE;

	Start_3d_mapping (req, resp);
}

bool Class_Mapping_Node::Stop_node_serv (std_srvs::Empty::Request &req ,std_srvs::Empty::Response &resp){
	stop_node = true;
}

void Class_Mapping_Node::main_function (){

	// Main Loop
	while (!stop_node){

		ERROR_CODE zed_state = zed.grab();
		if (zed_state == ERROR_CODE::SUCCESS){
			
			// Timer and frame count
			timer++;

			if(!pSvoFilepath.empty()){
				svo_position = zed.getSVOPosition();
				//NODELET_INFO_STREAM(" Svo Frame: \t-> " << svo_position);
			}

			// Publish Frames Tree (TFs)
			if (pPublish_Tf_Frames && !start_moving){
				publish_pose_frame (Map2Odom_Transf); // Publish Odom frame
				publish_odom_frame (Odom2Base_Transf); // Publish base_link frame
				publish_camera_frame (Base2Camera_Transf); // Publish camera frame
				publish_leftsensor_frame (Camera2Sensor_Transf); // Publish left camera frame
			}

			// Point Cloud
			int num_subs_pc_topic = pc_pub.getNumSubscribers();
			if (timer%7 == 0 && num_subs_pc_topic != 0){
				zed.retrieveMeasure(Point_Cloud, sl::MEASURE::XYZBGRA, sl::MEM::CPU);
				pc_msg = generate_pointcloud_msg (Point_Cloud);
				pc_pub.publish(pc_msg);
			}

			// IMU Data
			zed.getSensorsData(sensors_data, TIME_REFERENCE::IMAGE);
			if (!start_moving){
				start_moving = start_moving_func(sensors_data, start_moving);
				if (timer == 1 || start_moving){
					NODELET_INFO_STREAM("Vehicle started moving \t\t-> " << (start_moving ? "True" : "False"));
					if (!start_moving){
						NODELET_INFO_STREAM("Pose State \t\t\t-> " << pose_state);
						NODELET_INFO_STREAM("Odom State \t\t\t-> " << odom_state);
						NODELET_INFO_STREAM("Waiting for the vehicle to start moving.");
						printf("\n");
					}
				}
			}
			
			if (start_moving == true){

				// Enable Positional Tracking
				if (err_pos_track != ERROR_CODE::SUCCESS){
					//tracking_parameters.area_file_path = (pRoute+pAreaMemPath).c_str();
					err_pos_track = zed.enablePositionalTracking(tracking_parameters);
					NODELET_INFO_STREAM("Positional Tracking Status \t-> " << err_pos_track);
				}

				// Enable Spatial Mapping
				if (pMappingEnabled && err_spa_map != ERROR_CODE::SUCCESS){
					err_spa_map = zed.enableSpatialMapping(mapping_parameters);
					NODELET_INFO_STREAM("Spatial Mapping Status \t\t-> " << err_pos_track);
				}

				// Get Transformations
				if (!Base2Sensor_Transf_Valid){
					Base2Sensor_Transf_Valid = getBase2Sensor_Transf();
				}

				if (!Base2Camera_Transf_Valid){
					Base2Camera_Transf_Valid = getBase2Camera_Transf();
				}

				// Odometry
				last_odom_state = odom_state;
				odom_state = zed.getPosition(zed_odom, REFERENCE_FRAME::CAMERA);
					// Odom State
				odom_state_msg.data = sl::toString(odom_state).c_str();
				odom_state_pub.publish(odom_state_msg);
					// Odom Confidence
				//NODELET_INFO_STREAM("Map Confidence \t-> " << zed_odom.pose_confidence);
				odom_confidence.data = (int8_t) zed_odom.pose_confidence;
				odom_conf_pub.publish(odom_confidence);
				switch(odom_state){
					case POSITIONAL_TRACKING_STATE::OK:{
						if (last_odom_state != POSITIONAL_TRACKING_STATE::OK){
							NODELET_INFO_STREAM("Odom State \t\t\t-> " << odom_state);
						}

						searching_counter = 0; // Reseteo del contador de ''SEARCHING''
						
						geometry_msgs::Transform odom2sens_msg;
						odom2sens_msg.translation.x = zed_odom.getTranslation().tx;
						odom2sens_msg.translation.y = zed_odom.getTranslation().ty;
						odom2sens_msg.translation.z = zed_odom.getTranslation().tz;
						odom2sens_msg.rotation.x = zed_odom.getOrientation().ox;
						odom2sens_msg.rotation.y = zed_odom.getOrientation().oy;
						odom2sens_msg.rotation.z = zed_odom.getOrientation().oz;
						odom2sens_msg.rotation.w = zed_odom.getOrientation().ow;

						tf2::Transform odom2sens_Transf;
						tf2::fromMsg(odom2sens_msg, odom2sens_Transf);

						// Odom from sensor to base frame
						tf2::Transform deltaOdom2Base_Transf = Base2Sensor_Transf.inverse() * odom2sens_Transf * Base2Sensor_Transf;

						// Propagation of Odom transform in time
						Odom2Base_Transf = Odom2Base_Transf * deltaOdom2Base_Transf;

						// Construct and publish odom topic
						odom_msg = construct_Odom_msg(zed_odom);
						odom_pub.publish(odom_msg);

						// Construct odom_path message
						Odom_Path_msg = construct_Odom_Path_msg();
						odom_path.publish(Odom_Path_msg);
					}
						break;

					case POSITIONAL_TRACKING_STATE::SEARCHING:
						if (last_odom_state != POSITIONAL_TRACKING_STATE::SEARCHING){
							NODELET_INFO_STREAM("Odom State \t\t\t-> " << odom_state);
						}
						break;

					case POSITIONAL_TRACKING_STATE::FPS_TOO_LOW:
						if (last_odom_state != POSITIONAL_TRACKING_STATE::FPS_TOO_LOW){
							NODELET_INFO_STREAM("Odom State \t\t\t-> " << odom_state);
						}
						break;

					case POSITIONAL_TRACKING_STATE::OFF:
						if (last_odom_state != POSITIONAL_TRACKING_STATE::OFF){
							NODELET_INFO_STREAM("Odom State \t\t\t-> " << odom_state);
						}
						break;
				}

				// Pose
				last_pose_state = pose_state;
				pose_state = zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame
				//NODELET_INFO_STREAM("Pose Confidence \t-> " <<zed_pose.pose_confidence);
					// Pose State
				pose_state_msg.data = sl::toString(pose_state).c_str();
				pose_state_pub.publish(pose_state_msg);
				switch(pose_state){
					case POSITIONAL_TRACKING_STATE::OK:{
						if (last_pose_state != POSITIONAL_TRACKING_STATE::OK){
							NODELET_INFO_STREAM("Pose State \t\t\t-> " << pose_state);
						}

						searching_counter = 0; // Reseteo del contador de ''SEARCHING''

						Translation transl = zed_pose.getTranslation();
						Orientation quat = zed_pose.getOrientation();

						geometry_msgs::Transform map2sens_msg;
						map2sens_msg.translation.x = transl(0);
						map2sens_msg.translation.y = transl(1);
						map2sens_msg.translation.z = transl(2);
						map2sens_msg.rotation.x = quat(0);
						map2sens_msg.rotation.y = quat(1);
						map2sens_msg.rotation.z = quat(2);
						map2sens_msg.rotation.w = quat(3);
						
						tf2::Transform map2sens_Transf;
						tf2::fromMsg(map2sens_msg, map2sens_Transf);
						
						// Base position in map frame
						Map2Base_Transf = map2sens_Transf * Base2Sensor_Transf;
						Map2Odom_Transf = Map2Base_Transf * Odom2Base_Transf.inverse();

						if (reset_odom || init_odom){
							Odom2Base_Transf = Map2Base_Transf;
							//Map2Base_Transf.setIdentity();

							odom_msg = construct_Odom_msg(zed_odom);
							odom_pub.publish(odom_msg);

							reset_odom = false;
							init_odom = false;
						}

						// Construct and publish pose topic
						pose_msg = construct_Pose_msg(Map2Base_Transf);
						pose_pub.publish(pose_msg);

						// Construct odom_path message
						Pose_Path_msg = construct_Pose_Path_msg();
						pose_path.publish(Pose_Path_msg);
					}
						break;

					case POSITIONAL_TRACKING_STATE::SEARCHING:
						if (last_pose_state != POSITIONAL_TRACKING_STATE::SEARCHING){
							NODELET_INFO_STREAM("Pose State \t\t\t-> " << pose_state);
						}
						if (odom_state == POSITIONAL_TRACKING_STATE::SEARCHING){
							searching_counter++;
						}
						break;
					
					case POSITIONAL_TRACKING_STATE::FPS_TOO_LOW:
						if (last_pose_state != POSITIONAL_TRACKING_STATE::FPS_TOO_LOW){
							NODELET_INFO_STREAM("Pose State \t\t\t-> " << pose_state);
						}
						break;
					
					case POSITIONAL_TRACKING_STATE::OFF:
						if (last_pose_state != POSITIONAL_TRACKING_STATE::OFF){
							NODELET_INFO_STREAM("Pose State \t\t\t-> " << pose_state);
						}
						break;
				}

				if (searching_counter > 1000){
					std_srvs::Empty::Request req;
					std_srvs::Empty::Response res;

					Reset_session(req, res);

					searching_counter = 0;
				}

				// Map Construction
				if (pMappingEnabled && pose_state == POSITIONAL_TRACKING_STATE::OK && odom_state == POSITIONAL_TRACKING_STATE::OK && zed_odom.pose_confidence >= 75){
					// Request an update of the mesh every 60 frames (1 s in HD720 mode)
					if (timer % 60 == 0){
						zed.requestSpatialMapAsync();
					}
					
						// Retrieve spatial map when ready
					if (zed.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS){
						if (pMeshEnabled){
							zed.retrieveSpatialMapAsync(mesh_map); // AQUI SE CARGA EL MAPA EN FORMATO MESH
							
							// - NO FUNCIONA -
							//mesh_msg = generate_mesh_msg(mesh_map);	// AQUI INTENTA GENERAR EL MENSAJE
							//mesh_pub.publish(mesh_msg); // AQUI PUBLICA EL MENSAJE
						}
						else{
							ERROR_CODE spa_map_state = zed.retrieveSpatialMapAsync(fused_map);
							if (spa_map_state != ERROR_CODE::SUCCESS){
								NODELET_WARN_STREAM("Fused point cloud not extracted: " << sl::toString(spa_map_state).c_str());
							}
							fused_pc_msg = generate_fused_pc_msg(fused_map);
							fused_pc_pub.publish(fused_pc_msg);
						}
					}
				}

				// Construct IMU messages
				imu_msg = generate_imu_msg(sensors_data);
				imu_pub.publish(imu_msg);
				imu_tf = generate_imu_tf(sensors_data);

				// Publish Frames Tree (TFs)
				if (pPublish_Tf_Frames){
					publish_pose_frame (Map2Odom_Transf); // Publish Odom frame
					publish_odom_frame (Odom2Base_Transf); // Publish base_link frame
					publish_camera_frame (Base2Camera_Transf); // Publish camera frame
					publish_leftsensor_frame (Camera2Sensor_Transf); // Publish left camera frame
					publish_imu_frame(imu_tf);
				}
				
			}
		}
		else if (zed_state == ERROR_CODE::END_OF_SVOFILE_REACHED) {
			NODELET_INFO_STREAM(" ** SVO end has been reached **");
			stop_node = true;
		}else{
			NODELET_INFO_STREAM("Grab Error: " << zed_state);
            stop_node = true;
		}
		ros::spinOnce();
	}

	NODELET_INFO_STREAM("  ** Closing Node **");

	// Save the constructed map (Mesh or Fused Cloud)
	Save_map();

	// Disable spatial mapping, positional tracking and close the camera
	if (err_spa_map == ERROR_CODE::SUCCESS){
		zed.disableSpatialMapping();
		NODELET_INFO_STREAM("Spatial Mapping disabled.");
		zed.disablePositionalTracking((pRoute+pAreaMemPath).c_str());
		NODELET_INFO_STREAM("Positional Tracking disabled.");
	}

	zed.close();
	NODELET_INFO_STREAM(" ** Node Closed ** ");
}
}
