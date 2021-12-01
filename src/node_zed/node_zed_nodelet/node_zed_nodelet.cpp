#include "node_zed.hpp"

namespace node_zed_nodelet{

Zed_ROS_Class::Zed_ROS_Class() : Nodelet() {}

Zed_ROS_Class::~Zed_ROS_Class(){}

void Zed_ROS_Class::onInit(){
    NODELET_DEBUG("Initializing nodelet...");

	ros::NodeHandle nh = getMTNodeHandle();
	ros::NodeHandle nh_priv = getMTPrivateNodeHandle();

	readParameters();
	initTransforms();

	stop_node = nh_priv.advertiseService("stop", &Zed_ROS_Class::stop_serv, this);
    Start_Remote_Stream = nh_priv.advertiseService("start_remote_stream", &Zed_ROS_Class::start_remote_stream, this);
	Stop_Remote_Stream = nh_priv.advertiseService("stop_remote_stream", &Zed_ROS_Class::stop_remote_stream, this);

	// Initialize parameters
		// Initial Parameters
	init_params.coordinate_units = UNIT::METER;
	init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
	NODELET_INFO_STREAM(" * Camera coordinate system\t-> " << sl::toString(init_params.coordinate_system));
	init_params.depth_mode = static_cast<sl::DEPTH_MODE>(pDepthMode); // Set the depth mode to QUALITY
	init_params.depth_minimum_distance = pCameraMinDepth;
	init_params.depth_maximum_distance = pCameraMaxDepth;      // Set the maximum depth perception distance to 20m
		// Positional Tracking Parameters
	Init_Pose = set_initial_pose(pInitialBasePose);
	tracking_parameters.initial_world_transform = Init_Pose;
	tracking_parameters.enable_pose_smoothing = true;
	tracking_parameters.enable_area_memory = pAreaMemory;
	tracking_parameters.enable_imu_fusion = pImuFusion;
	tracking_parameters.area_file_path = pAreaMemPath.c_str();
	// Positional Tracking status variable
	err_pos_track = zed.enablePositionalTracking(tracking_parameters);
		// Spatial Mapping Parameters
	if (pMappingEnabled){
		mapping_parameters.range_meter = pMaxMapDepth; // Ver cuando mMaxMappingRange = -1 (AUTO)
		mapping_parameters.resolution_meter = pMappingResolution;
			// Fused or meshed map
		if (mesh){
			// Set mapping with mesh output
			mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
			mapping_parameters.save_texture = true ;  // Scene texture will be recorded
		}
		else{
			// or select point cloud output
			mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::FUSED_POINT_CLOUD;
		}
		// Spatial Mapping status variable
		err_spa_map = zed.enableSpatialMapping(mapping_parameters);
	}
	

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
	openning_status = ERROR_CODE::CAMERA_NOT_DETECTED;
	NODELET_INFO_STREAM(" *** Opening " << sl::toString(pZedCameraModel) << "...");
	while (openning_status != ERROR_CODE::SUCCESS){
		openning_status = zed.open(init_params);
		NODELET_INFO_STREAM("ZED connection -> " << sl::toString(openning_status));
	}

    main_function();
}

void Zed_ROS_Class::readParameters(){
	// Camera Parameters
	printf("Reading Parameters:\n");
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
/*
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
*/
	// Depth Parameters
	int depth_mode;
    nh_priv.getParam(name + "depth/quality", depth_mode);
    pDepthMode = static_cast<sl::DEPTH_MODE>(depth_mode);
	NODELET_INFO_STREAM(" * Depth quality\t\t-> " << sl::toString(pDepthMode).c_str());

	/*nh_priv.getParam("depth/depth_stabilization", pDepthStabilization);
	printf(" * Depth Stabilization -> ")
    NODELET_INFO_STREAM(" * Depth Stabilization\t\t-> " << (mDepthStabilization ? "ENABLED" : "DISABLED"));
*/
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

void Zed_ROS_Class::initTransforms(){
	// Initialization TFs
	Map2Odom_Transf.setIdentity();
	Map2Base_Transf.setIdentity();
	Odom2Base_Transf.setIdentity();
	Sensor2Base_Transf.setIdentity();
	Camera2Sensor_Transf.setIdentity();
	Camera2Sensor_Transf.setOrigin(Left_Camera_Origin);
	Base2Camera_Transf.setIdentity();
}

//Transform Zed_ROS_Class::set_initial_pose(float x0, float y0, float z0, float r0, float p0, float yaw0){
Transform Zed_ROS_Class::set_initial_pose(vector<float> pInitialBasePose){
	pInitialBasePose.resize(6);
	tf2::Transform init_pose_tf;
	tf2::Vector3 trasl_origin(pInitialBasePose[0], pInitialBasePose[1], pInitialBasePose[2]);
	init_pose_tf.setOrigin(trasl_origin);
	tf2::Quaternion quat_origin;
	quat_origin.setRPY(pInitialBasePose[3], pInitialBasePose[4], pInitialBasePose[5]);
	init_pose_tf.setRotation(quat_origin);

	init_pose_tf = init_pose_tf * Sensor2Base_Transf.inverse();

	sl::float3 trasl_vector;
	trasl_vector[0] = init_pose_tf.getOrigin().x();
	trasl_vector[1] = init_pose_tf.getOrigin().y();
	trasl_vector[2] = init_pose_tf.getOrigin().z();

	sl::float4 quat_vector;
	quat_vector[0] = init_pose_tf.getRotation().x();
	quat_vector[1] = init_pose_tf.getRotation().y();
	quat_vector[2] = init_pose_tf.getRotation().z();
	quat_vector[3] = init_pose_tf.getRotation().w();

	Translation init_position (trasl_vector);
	Orientation init_orientation (quat_vector);
	Transform init_pose_sl;
	init_pose_sl.setTranslation(init_position);
	init_pose_sl.setOrientation(init_orientation);

	return init_pose_sl;
}

nav_msgs::Odometry Zed_ROS_Class::construct_Odom(Pose zed_odom){
	nav_msgs::Odometry odom_msg;
	geometry_msgs::Transform odom2base = tf2::toMsg(Odom2Base_Transf);

	// Header
	odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
	odom_msg.child_frame_id = "base_link";

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

nav_msgs::PathPtr Zed_ROS_Class::construct_Odom_Path(){
	geometry_msgs::PoseStamped odom_path_msg;
	geometry_msgs::Transform odom2base = tf2::toMsg(Odom2Base_Transf);

	// Header
	odom_path_msg.header.stamp = ros::Time::now();
    odom_path_msg.header.frame_id = pOdometryFrameId;

	// Pose
	odom_path_msg.pose.position.x = odom2base.translation.x;
    odom_path_msg.pose.position.y = odom2base.translation.y;
	odom_path_msg.pose.position.z = odom2base.translation.z;
	
    odom_path_msg.pose.orientation.x = odom2base.rotation.x;
	odom_path_msg.pose.orientation.y = odom2base.rotation.y;
	odom_path_msg.pose.orientation.z = odom2base.rotation.z;
	odom_path_msg.pose.orientation.w = odom2base.rotation.w;

	Odom_Path.push_back(odom_path_msg);

	nav_msgs::PathPtr Odom_Path_msg = boost::make_shared<nav_msgs::Path>();
	Odom_Path_msg->header.frame_id = pMapFrameId;
	Odom_Path_msg->header.stamp = ros::Time::now();
	Odom_Path_msg->poses = Odom_Path;

	return Odom_Path_msg;
}

geometry_msgs::PoseStamped Zed_ROS_Class::construct_Pose(){
	tf2::Transform base_pose;
	base_pose.setIdentity();

	base_pose = Map2Base_Transf;
	//base_pose = Map2Odom_Transf;

	geometry_msgs::Transform base2frame = tf2::toMsg(base_pose);

	geometry_msgs::Pose pose;
	std_msgs::Header header;

	header.stamp = ros::Time::now();
	header.frame_id = pMapFrameId;

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

nav_msgs::PathPtr Zed_ROS_Class::construct_Pose_Path(){
	geometry_msgs::PoseStamped pose_path_msg;
	geometry_msgs::Transform map2base = tf2::toMsg(Map2Base_Transf);

	// Header
	pose_path_msg.header.stamp = ros::Time::now();
    pose_path_msg.header.frame_id = pMapFrameId;

	// Pose
	pose_path_msg.pose.position.x = map2base.translation.x;
    pose_path_msg.pose.position.y = map2base.translation.y;
	pose_path_msg.pose.position.z = map2base.translation.z;
	
    pose_path_msg.pose.orientation.x = map2base.rotation.x;
	pose_path_msg.pose.orientation.y = map2base.rotation.y;
	pose_path_msg.pose.orientation.z = map2base.rotation.z;
	pose_path_msg.pose.orientation.w = map2base.rotation.w;

	Pose_Path.push_back(pose_path_msg);

	nav_msgs::PathPtr Pose_Path_msg = boost::make_shared<nav_msgs::Path>();
	Pose_Path_msg->header.frame_id = pMapFrameId;
	Pose_Path_msg->header.stamp = ros::Time::now();
	Pose_Path_msg->poses = Pose_Path;

	return Pose_Path_msg;
}

sensor_msgs::Imu Zed_ROS_Class::generate_imu_msg (SensorsData sensors_data){

	sensor_msgs::Imu imu;

	// header
	imu.header.stamp = ros::Time::now();
	imu.header.frame_id = "zed2_imu_link";

	// orientation
	imu.orientation.x =  sensors_data.imu.pose.getOrientation().x;
	imu.orientation.y =  sensors_data.imu.pose.getOrientation().y;
	imu.orientation.z =  sensors_data.imu.pose.getOrientation().z;
	imu.orientation.w =  sensors_data.imu.pose.getOrientation().w;
	memcpy(&imu.orientation_covariance, &sensors_data.imu.pose_covariance, 9*sizeof(float));

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

sensor_msgs::PointCloud2Ptr Zed_ROS_Class::generate_pointcloud_msg (Mat Point_Cloud){

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

sensor_msgs::PointCloud2Ptr Zed_ROS_Class::generate_fused_pc_msg (FusedPointCloud pc_input){

	sensor_msgs::PointCloud2Ptr fused_pc_msg = boost::make_shared<sensor_msgs::PointCloud2>();

	fused_pc_msg->header.stamp = ros::Time::now();
	fused_pc_msg->header.frame_id = "map";

	fused_pc_msg->height = 1;
	fused_pc_msg->width = pc_input.vertices.size();

	sensor_msgs::PointCloud2Modifier modifier(*fused_pc_msg);
	modifier.setPointCloud2Fields(4,
									"x", 1, sensor_msgs::PointField::FLOAT32,
									"y", 1, sensor_msgs::PointField::FLOAT32,
									"z", 1, sensor_msgs::PointField::FLOAT32,
									"rgb", 1, sensor_msgs::PointField::FLOAT32);

	//float * pc_Ptr = (float*) malloc (pc_input.getNumberOfPoints() * 4 * sizeof(float));
	float * pc_Ptr = new float [pc_input.getNumberOfPoints() * 4 * sizeof(float)];

	pc_Ptr = (float *)(&fused_pc_msg->data[0]);

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

/*
mesh_msgs::MeshGeometryStampedPtr Zed_ROS_Class::generate_mesh_msg (Mesh mesh_map){
	int num_of_verts = 0;
	int num_of_trian = mesh_map.getNumberOfTriangles();
	for (int c = 0; c < mesh_map.chunks.size(); c++){
        num_of_verts += mesh_map.chunks[c].vertices.size();
	}

	mesh_msgs::MeshGeometryStampedPtr mesh = boost::make_shared<mesh_msgs::MeshGeometryStamped>();
	
	mesh->header.stamp = ros::Time::now();
    mesh->header.frame_id = "map";

	//float * verts_Ptr = (float*) malloc (3 * sizeof(float) * num_of_verts);
	float * verts_Ptr = new float[3 * sizeof(float) * num_of_verts];
	verts_Ptr = (float *)(&mesh->mesh_geometry.vertices);

	//float * normals_Ptr = (float*) malloc (3 * sizeof(float) * num_of_verts);
	float * normals_Ptr = new float[3 * sizeof(float) * num_of_verts];
	normals_Ptr = (float *)(&mesh->mesh_geometry.vertex_normals);

	//int * faces_Ptr = (int*) malloc (3 * sizeof(int) * num_of_trian);
	int * faces_Ptr = new int [3 * sizeof(int) * num_of_trian];
	faces_Ptr = (int *)(&mesh->mesh_geometry.faces);

	for(int c = 0; c < mesh_map.chunks.size(); c++){
		cout<<c+1<<"/"<<mesh_map.chunks.size()<<endl;
		cout<<mesh_map.chunks[c].vertices.size()<<endl;

		float * mesh_verts = (float *)(mesh_map.chunks[c].vertices.data());
        memcpy(verts_Ptr, mesh_verts, 3 * sizeof(float) * mesh_map.chunks[c].vertices.size());
		verts_Ptr += 3 * mesh_map.chunks[c].vertices.size();

		int * mesh_triangles = (int *)(mesh_map.chunks[c].triangles.data());
        memcpy(faces_Ptr, mesh_triangles, 3 * sizeof(int) * mesh_map.chunks[c].triangles.size());
		faces_Ptr += 3 * mesh_map.chunks[c].triangles.size();

		float * mesh_normals = (float *)(mesh_map.chunks[c].normals.data());
        memcpy(normals_Ptr, mesh_normals, 3 * sizeof(float) * mesh_map.chunks[c].normals.size());
		normals_Ptr += 3 * mesh_map.chunks[c].normals.size();
		
	}
	return mesh;
}
*/
geometry_msgs::Transform Zed_ROS_Class::generate_imu_tf_msg(Translation tr, Orientation rot){

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

bool Zed_ROS_Class::start_moving_func (sl::SensorsData sensors_data, bool start_moving){

	if (sensors_data.imu.linear_acceleration.x >= 0.3 && start_moving == false)
		start_moving = true;

	return start_moving;
}

bool Zed_ROS_Class::getSensor2Base_Transf(){
	cout<<"Sensor2Base"<<endl;
	Sensor2Base_Transf_Valid = false;
	
	try{
		// Save the transformation
		geometry_msgs::TransformStamped b2s = tfBuffer.lookupTransform("left_camera", "base_link", ros::Time::now());//, ros::Duration(0));

		// Get the TF2 transformation
        tf2::fromMsg(b2s.transform, Sensor2Base_Transf);

		double roll, pitch, yaw;
        tf2::Matrix3x3(Sensor2Base_Transf.getRotation()).getRPY(roll, pitch, yaw);

	}catch(tf2::TransformException& ex){
		Sensor2Base_Transf.setIdentity();
		return false;
	}

	Sensor2Base_Transf_Valid = true;
	return true;
}

bool Zed_ROS_Class::getCamera2Sensor_Transf(){
	cout<<"Sensor2Camera"<<endl;
	Sensor2Camera_Transf_Valid = false;
	
	try{
		// Save the transformation
		geometry_msgs::TransformStamped c2s = tfBuffer.lookupTransform("left_camera", "camera_center", ros::Time::now());//, ros::Duration(0.1));
        
		// Get the TF2 transformation
        tf2::fromMsg(c2s.transform, Camera2Sensor_Transf);

		double roll, pitch, yaw;
        tf2::Matrix3x3(Camera2Sensor_Transf.getRotation()).getRPY(roll, pitch, yaw);

	}catch(tf2::TransformException& ex){
		Camera2Sensor_Transf.setIdentity();
		return false;
	}

	Sensor2Camera_Transf_Valid = true;
	return true;
}

bool Zed_ROS_Class::getBase2Camera_Transf(){
	cout<<"Camera2Base"<<endl;
	Camera2Base_Transf_Valid = false;

	try{
		// Save the transformation
		geometry_msgs::TransformStamped c2b = tfBuffer.lookupTransform("camera_center", "base_link", ros::Time(0));//, ros::Duration(0.1));
        
		// Get the TF2 transformation
        tf2::fromMsg(c2b.transform, Base2Camera_Transf);

		double roll, pitch, yaw;
        tf2::Matrix3x3(Base2Camera_Transf.getRotation()).getRPY(roll, pitch, yaw);

	}catch(tf2::TransformException& ex){
		Base2Camera_Transf.setIdentity();
		return false;
	}

	Camera2Base_Transf_Valid = true;
	return true;
}

tf2::Transform Zed_ROS_Class::getCamera2Odom_Transf(){
	//cout<<"Camera2Odom"<<endl;
	tf2::Transform cam2odom_transf;

	try{
		// Save the transformation
		geometry_msgs::TransformStamped c2o = tfBuffer.lookupTransform("odom", "camera_center", ros::Time(0));//, ros::Duration(0.1));
        
		// Get the TF2 transformation
        tf2::fromMsg(c2o.transform, cam2odom_transf);

	}catch(tf2::TransformException& ex){
		cam2odom_transf.setIdentity();
	}

	return cam2odom_transf;
}

tf2::Transform Zed_ROS_Class::generate_imu_tf (SensorsData sensors_data){
	tf2::Transform imu_tf;
	imu_tf.setIdentity();

	// quaternion to tf generation
	tf2::Quaternion imu_quat;
	imu_quat.setX(sensors_data.imu.pose.getOrientation().x);
	imu_quat.setY(sensors_data.imu.pose.getOrientation().y);
	imu_quat.setZ(sensors_data.imu.pose.getOrientation().z);
	imu_quat.setW(sensors_data.imu.pose.getOrientation().w);

	tf2::Transform cam2odom = getCamera2Odom_Transf();
	tf2::Quaternion cam2odom_quat = cam2odom.getRotation();
	imu_tf.setRotation(imu_quat * cam2odom_quat.inverse());

	return imu_tf;
}

void Zed_ROS_Class::publish_odom_frame (tf2::Transform Odom_Transf){

	if (!Sensor2Base_Transf_Valid){
		Sensor2Base_Transf_Valid = getSensor2Base_Transf();
	}
	if (!Sensor2Camera_Transf_Valid){
		Sensor2Camera_Transf_Valid = getCamera2Sensor_Transf();
	}
	if (!Camera2Base_Transf_Valid){
		Camera2Base_Transf_Valid = getBase2Camera_Transf();
	}

    geometry_msgs::TransformStamped transform;
	static tf2_ros::TransformBroadcaster st_br;
    
	transform.header.stamp = ros::Time::now();
    //cout<<"mapa: "<<pMapFrameId<<endl;
    //cout<<"odometria: "<<pOdometryFrameId<<endl;
	transform.header.frame_id = pMapFrameId;
	transform.child_frame_id = pOdometryFrameId;
	transform.transform = tf2::toMsg (Odom_Transf);
	st_br.sendTransform(transform);
}

void Zed_ROS_Class::publish_base_frame (tf2::Transform Odom_Transf){

	if (!Sensor2Base_Transf_Valid){
		Sensor2Base_Transf_Valid = getSensor2Base_Transf();
	}
	if (!Sensor2Camera_Transf_Valid){
		Sensor2Camera_Transf_Valid = getCamera2Sensor_Transf();
	}
	if (!Camera2Base_Transf_Valid){
		Camera2Base_Transf_Valid = getBase2Camera_Transf();
	}

    geometry_msgs::TransformStamped transform;
	tf2_ros::TransformBroadcaster br;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pOdometryFrameId;
	transform.child_frame_id = pBaseFrameId;
	transform.transform = tf2::toMsg (Odom_Transf);
    br.sendTransform(transform);
}

void Zed_ROS_Class::publish_camera_frame (tf2::Transform Camera_Transf){

	if (!Sensor2Base_Transf_Valid){
		Sensor2Base_Transf_Valid = getSensor2Base_Transf();
	}
	if (!Sensor2Camera_Transf_Valid){
		Sensor2Camera_Transf_Valid = getCamera2Sensor_Transf();
	}
	if (!Camera2Base_Transf_Valid){
		Camera2Base_Transf_Valid = getBase2Camera_Transf();
	}

    geometry_msgs::TransformStamped transform;
	tf2_ros::TransformBroadcaster br;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pBaseFrameId;
	transform.child_frame_id = pCameraFrameId;
	transform.transform = tf2::toMsg (Camera_Transf);
    br.sendTransform(transform);
}

void Zed_ROS_Class::publish_gps_frame (tf2::Transform Camera_Transf){

    geometry_msgs::TransformStamped transform;
	tf2_ros::TransformBroadcaster br;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pBaseFrameId;
	transform.child_frame_id = "gps";
	transform.transform = tf2::toMsg (Camera_Transf);
    br.sendTransform(transform);
}

void Zed_ROS_Class::publish_leftsensor_frame (tf2::Transform Camera_Transf){

	if (!Sensor2Base_Transf_Valid){
		Sensor2Base_Transf_Valid = getSensor2Base_Transf();
	}
	if (!Sensor2Camera_Transf_Valid){
		Sensor2Camera_Transf_Valid = getCamera2Sensor_Transf();
	}
	if (!Camera2Base_Transf_Valid){
		Camera2Base_Transf_Valid = getBase2Camera_Transf();
	}

    geometry_msgs::TransformStamped transform;
	tf2_ros::TransformBroadcaster br;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pCameraFrameId;
	transform.child_frame_id = pLeftCameraFrameId;
	transform.transform = tf2::toMsg (Camera_Transf);
    br.sendTransform(transform);
}

void Zed_ROS_Class::publish_imu_frame (tf2::Transform Imu_Transf){

	if (!Sensor2Base_Transf_Valid){
		Sensor2Base_Transf_Valid = getSensor2Base_Transf();
	}
	if (!Sensor2Camera_Transf_Valid){
		Sensor2Camera_Transf_Valid = getCamera2Sensor_Transf();
	}
	if (!Camera2Base_Transf_Valid){
		Camera2Base_Transf_Valid = getBase2Camera_Transf();
	}

    geometry_msgs::TransformStamped transform;
	tf2_ros::TransformBroadcaster br;
    
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = pLeftCameraFrameId;
	transform.child_frame_id = pImuFrameId;
	transform.transform = tf2::toMsg (Imu_Transf);
    br.sendTransform(transform);
}

std::vector<std::string> Zed_ROS_Class::split_string(const std::string& s, char seperator){
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

// ROS Services functions
bool Zed_ROS_Class::stop_serv(std_srvs::Empty::Request &req ,std_srvs::Empty::Response &resp){
	stop = true;
}

bool Zed_ROS_Class::start_remote_stream(node_zed::start_remote_stream::Request& req, node_zed::start_remote_stream::Response& res){
	
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

		printf("hola1");
        Streaming = false;

        res.result = false;
        res.info = "`gop_size` wrong (";
        res.info += params.gop_size;
        res.info += "). Remote streaming not started";

        //cout<<res.info<<endl;
		printf("hola2");
        NODELET_ERROR_STREAM(res.info);
        return false;
    }

	printf("hola3");
    if (params.port % 2 != 0) {
        printf("hola4");
		Streaming = false;

        res.result = false;
        res.info = "`port` must be an even number. Remote streaming not started";

        //cout<<res.info<<endl;
        NODELET_ERROR_STREAM(res.info);
		printf("hola5");
        return false;
    }
	printf("hola6");
    ERROR_CODE err = zed.enableStreaming(params);
	printf("hola7");

    if (err != sl::ERROR_CODE::SUCCESS) {
		printf("hola8");
        Streaming = false;
		printf("hola9");

        res.result = false;
        res.info = sl::toString(err).c_str();
		printf("hola10");

        //cout<<"Remote streaming not started (" << res.info << ")"<<endl;
        NODELET_ERROR_STREAM("Remote streaming not started (" << res.info << ")");

		printf("hola11");
        return false;
    }
	printf("hola12");
    Streaming = true;
	printf("hola13");

    //cout<<"Remote streaming STARTED"<<endl;
    NODELET_INFO_STREAM("Remote streaming STARTED");
	printf("hola14");

    res.result = true;
    res.info = "Remote streaming STARTED";
    cout<<res.info<<endl;
	printf("hola15");
    return true;

}

bool Zed_ROS_Class::stop_remote_stream(node_zed::stop_remote_stream::Request& req, node_zed::stop_remote_stream::Response& res) {
    if (Streaming) {
        zed.disableStreaming();
    }
    
    Streaming = false;
    //cout<<"SVO remote streaming STOPPED"<<endl;
    NODELET_INFO_STREAM("SVO remote streaming STOPPED");

    res.done = true;
    return true;
}

void Zed_ROS_Class::main_function(){

	// Main Loop
	while (end_svo == false && stop == false) {

		// Timer and frame count
		timer++;
		cout<<"timer: "<<timer<<endl;
		if(open_from_svo){
			svo_position = zed.getSVOPosition();
			cout << "svo_position :" << svo_position <<endl;
		}

		// IMU Data
		zed.getSensorsData(sensors_data, TIME_REFERENCE::IMAGE);
		start_moving = start_moving_func(sensors_data, start_moving);
		cout<<"Start moving: "<<start_moving<<endl;
		
		if (zed.grab() == ERROR_CODE::SUCCESS && start_moving == true){

			// Enable Positional Tracking and Spatial Mapping
			if (err_pos_track != ERROR_CODE::SUCCESS || err_spa_map != ERROR_CODE::SUCCESS){
				err_pos_track = zed.enablePositionalTracking(tracking_parameters);
				cout<<"Positional Tracking Status: "<<err_pos_track<<endl;
				err_spa_map = zed.enableSpatialMapping(mapping_parameters);
				cout<<"Spatial Mapping Status: "<<err_spa_map<<endl;
			}

			// Point Cloud
			int num_subs_pc_topic = pc_pub.getNumSubscribers();
			if (timer%7 == 0 && num_subs_pc_topic != 0){
				zed.retrieveMeasure(Point_Cloud, sl::MEASURE::XYZBGRA, sl::MEM::CPU);
				pc_msg = generate_pointcloud_msg (Point_Cloud);
				pc_pub.publish(pc_msg);
			}

			// Map Construction
				// Request an update of the mesh every 30 frames (0.5s in HD720 mode)
			int num_subs_fused_pc_topic = fused_pc_pub.getNumSubscribers();
			if (timer%30 == 0)// && num_subs_fused_pc_topic != 0)
				zed.requestSpatialMapAsync();
		    
				// Retrieve spatial map when ready
			if (zed.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS){
				if (mesh && timer >= 1000){
					//zed.extractWholeSpatialMap(mesh_map);
					zed.retrieveSpatialMapAsync(mesh_map); // AQUÍ SE CARGA EL MESH

					/*
					zed.retrieveSpatialMapAsync(mesh_map);
					
					printf("entra\n");
					mesh_msg = generate_mesh_msg(mesh_map);	// AQUÍ INTENTO GENERAR EL MENSAJE
					std::cout<<"Se fue"<<std::endl;
                    zed_ros_obj.mesh_pub.publish(mesh_msg); // AQUÍ PUBLICO EL MENSAJE
					*/
				}
				else{
					zed.retrieveSpatialMapAsync(fused_map);
					fused_pc_msg = generate_fused_pc_msg(fused_map);
					fused_pc_pub.publish(fused_pc_msg);
				}
			}

			// Get Transformations
			if (!Sensor2Base_Transf_Valid){
				Sensor2Base_Transf_Valid = getSensor2Base_Transf();
			}
			if (!Sensor2Camera_Transf_Valid){
				Sensor2Camera_Transf_Valid = getCamera2Sensor_Transf();
			}
			if (!Camera2Base_Transf_Valid){
				Camera2Base_Transf_Valid = getBase2Camera_Transf();
			}

			// Pose
			pose_state = zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD); // Get the pose of the left eye of the camera with reference to the world frame
			std::cout<<"Pose Confidence: "<<zed_pose.pose_confidence<<std::endl;
			switch(pose_state){
				case POSITIONAL_TRACKING_STATE::OK:{
					pose_state_msg.data = "OK";

					geometry_msgs::Transform map2sens_msg;
					tf2::Transform map2sens_Transf;

					Translation transl = zed_pose.getTranslation();
					Orientation quat = zed_pose.getOrientation();

					map2sens_msg.translation.x = transl(0);
					map2sens_msg.translation.y = transl(1);
					map2sens_msg.translation.z = transl(2);
					map2sens_msg.rotation.x = quat(0);
					map2sens_msg.rotation.y = quat(1);
					map2sens_msg.rotation.z = quat(2);
					map2sens_msg.rotation.w = quat(3);
					
					tf2::fromMsg(map2sens_msg, map2sens_Transf);
					
					// Base position in map frame
					Map2Base_Transf = map2sens_Transf * Sensor2Base_Transf;
					Map2Odom_Transf = Map2Base_Transf * Odom2Base_Transf.inverse();

					// Construct and publish pose topic
					int num_subs_pose_topic = pose_pub.getNumSubscribers();
					if(num_subs_pose_topic != 0){
						pose_msg = construct_Pose();
						pose_pub.publish(pose_msg);
					}

					// Construct odom_path message
					Pose_Path_msg = construct_Pose_Path();
				}
					break;

				case POSITIONAL_TRACKING_STATE::SEARCHING:
					pose_state_msg.data = "SEARCHING";
					printf("Searching.\n");
					break;
				
				case POSITIONAL_TRACKING_STATE::FPS_TOO_LOW:
					pose_state_msg.data = "FPS_TOO_LOW";
					printf("FPS too low, try to set a lower resolution.\n");
					break;
				
				case POSITIONAL_TRACKING_STATE::OFF:
					pose_state_msg.data = "OFF";
					printf("Positional Tracking not enabled.\n");
					break;
			}
				// Publish Pose State topics
			pose_state_pub.publish(pose_state_msg);


			// Odometry
			odom_state = zed.getPosition(zed_odom, REFERENCE_FRAME::CAMERA);
			std::cout<<"Odom Confidence: "<<zed_odom.pose_confidence<<std::endl;
			odom_confidence.data = (int8_t) zed_odom.pose_confidence;
			odom_conf_pub.publish(odom_confidence);
			switch(odom_state){
				case POSITIONAL_TRACKING_STATE::OK:{
					odom_state_msg.data = "OK";

					geometry_msgs::Transform odom2sens_msg;
					tf2::Transform odom2sens_Transf;

					odom2sens_msg.translation.x = zed_odom.getTranslation().tx;
					odom2sens_msg.translation.y = zed_odom.getTranslation().ty;
					odom2sens_msg.translation.z = zed_odom.getTranslation().tz;
					odom2sens_msg.rotation.x = zed_odom.getOrientation().ox;
					odom2sens_msg.rotation.y = zed_odom.getOrientation().oy;
					odom2sens_msg.rotation.z = zed_odom.getOrientation().oz;
					odom2sens_msg.rotation.w = zed_odom.getOrientation().ow;
					
					tf2::fromMsg(odom2sens_msg, odom2sens_Transf);

					// Odom from sensor to base frame
					tf2::Transform OdomTf_base = Sensor2Base_Transf.inverse() * odom2sens_Transf * Sensor2Base_Transf;

					// Propagation of Odom transform in time
					Odom2Base_Transf = Odom2Base_Transf * OdomTf_base;

					// Construct and publish odom topic
					int num_subs_odom_topic = odom_pub.getNumSubscribers();
					if(num_subs_odom_topic != 0){
						odom_msg = construct_Odom(zed_odom);
						odom_pub.publish(odom_msg);	
					}

					// Construct odom_path message
					Odom_Path_msg = construct_Odom_Path();	
				}
					break;

				case POSITIONAL_TRACKING_STATE::SEARCHING:
					odom_state_msg.data = "SEARCHING";
					printf("Searching.\n");
					break;

				case POSITIONAL_TRACKING_STATE::FPS_TOO_LOW:
					odom_state_msg.data = "FPS_TOO_LOW";
					printf("FPS too low, try to set a lower resolution.\n");
					break;

				case POSITIONAL_TRACKING_STATE::OFF:
					odom_state_msg.data = "OFF";
					printf("Positional Tracking not enabled.\n");
					break;
			}
				// Publish Odom State topics
			odom_state_pub.publish(odom_state_msg);


			// Construct IMU messages
			imu_msg = generate_imu_msg(sensors_data);
			imu_pub.publish(imu_msg);
			imu_tf = generate_imu_tf(sensors_data);

			// Publish Paths
			odom_path.publish(Odom_Path_msg);
			pose_path.publish(Pose_Path_msg);
			
			// Publish Frames Tree (TFs)
			if (pPublish_Tf_Frames){
				publish_odom_frame (Map2Odom_Transf); // Publish Odom frame
				publish_base_frame (Odom2Base_Transf); // Publish base_link frame
				publish_camera_frame (Base2Camera_Transf); // Publish camera frame
				publish_leftsensor_frame (Camera2Sensor_Transf); // Publish left camera frame
				publish_imu_frame(imu_tf);
			}

			if (open_from_svo && svo_position == 0 && start_moving) {
				cont++;
				if (cont == 2)
					end_svo = true;
			}
		}
		
		else if (zed.grab() == ERROR_CODE::END_OF_SVOFILE_REACHED) {
			std::cout << "SVO end has been reached. Looping back to first frame" << std::endl;
			zed.setSVOPosition(0);
			end_svo = true;
		}
		ros::spinOnce();
	}

	if (mesh && err_pos_track == ERROR_CODE::SUCCESS){
		printf("Creating mesh.\n");
		// Process the mesh
		printf("Applying filter.\n");
		mesh_map.filter(MeshFilterParameters::MESH_FILTER::MEDIUM); // Filter the mesh
		printf("Applying texture.\n");
		mesh_map.applyTexture(MESH_TEXTURE_FORMAT::RGB); // Apply the texture
		printf("Saving mesh.\n");
		mesh_map.save("/home/isa/mesh.obj", MESH_FILE_FORMAT::OBJ); // Save the mesh in .obj format
		printf("Mesh saved.\n");
	}else if (mesh == false && err_pos_track == ERROR_CODE::SUCCESS)
	{
		printf("Saving fused point cloud.\n");
		fused_map.save("/home/isa/fused_pc.obj", MESH_FILE_FORMAT::OBJ);
		printf("Fused point cloud saved.\n");
	}
	
	// Disable spatial mapping, positional tracking and close the camera
	if (err_pos_track == ERROR_CODE::SUCCESS){
		zed.disableSpatialMapping();
		printf("Spatial Mapping disabled.\n");
		zed.disablePositionalTracking();
		printf("Positional Tracking disabled.\n");
	}

	printf("Closing program.\n");
	zed.close();
}
}
