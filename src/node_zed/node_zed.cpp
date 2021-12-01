#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
        ros::init(argc, argv, "node_zed");

        // Start the ZED Nodelet
        nodelet::Loader nodelet;
        nodelet::M_string remap(ros::names::getRemappings());
        nodelet::V_string nargv;
        nodelet.load(ros::this_node::getName(), "node_zed_nodelet/node_zed_nodelet", remap, nargv);

	ros::spin();
	return 0;
}







/*	
	void signalHandler(int signum){
		//stop = true;
		exit(1);
	}


	Zed_ROS_Class zed_ros_obj;
	printf("hola1");

	zed_ros_obj.onInit();

	//zed_ros_obj.readParameters();
	printf("hola2");

	//zed_ros_obj.main_function();
	printf("hola fin");
*/	















/*
 *
 *                         // Display the translation and timestamp
        printf("Translation: Tx: %.3f, Ty: %.3f, Tz: %.3f, Timestamp: %llu\n", zed_pose.getTranslation().tx, zed_pose.getTranslation().ty, zed_pose.getTranslation().tz, zed_pose.timestamp);

        // Display the orientation quaternion
        printf("Orientation: Ox: %.3f, Oy: %.3f, Oz: %.3f, Ow: %.3f\n\n", zed_pose.getOrientation().ox, zed_pose.getOrientation().oy, zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);


            // Get the IMU data, in SVO mode only IMAGE isavailable
            zed.getSensorsData(sensors_data, TIME_REFERENCE::IMAGE);
                    std::cout << " - IMU:\n"<< std::endl;
                    std::cout << " \t Orientation: {" << sensors_data.imu.pose.getOrientation() << "}\n"<< std::endl;
                    std::cout << " \t Acceleration: {" << sensors_data.imu.linear_acceleration << "} [m/sec^2]\n"<< std::endl;
                    std::cout << " \t Angular Velocitiy: {" << sensors_data.imu.angular_velocity << "} [deg/sec]\n"<< std::endl;
                    std::cout << " - Magnetometer\n \t Magnetic Field: {" << sensors_data.magnetometer.magnetic_field_calibrated << "} [uT]\n"<< std::endl;
                    std::cout << " - Barometer\n \t Atmospheric pressure:" << sensors_data.barometer.pressure << " [hPa]\n"<< std::endl;



//pose_msg = generate_pose_msg(zed_pose);
//odom_msg = generate_odom_msg(zed_odom);

geometry_msgs::PoseStamped generate_pose_msg(Pose zed_pose){
	    
	    geometry_msgs::PoseStamped Pose;

            Pose.header.stamp = ros::Time::now();
            Pose.header.frame_id = "map";

            //set the position
            Pose.pose.position.x = zed_pose.getTranslation().tx;
            Pose.pose.position.y = zed_pose.getTranslation().ty;
            Pose.pose.position.z = zed_pose.getTranslation().tz;

            Pose.pose.orientation.x = zed_pose.getOrientation().ox;
            Pose.pose.orientation.y = zed_pose.getOrientation().oy;
            Pose.pose.orientation.z = zed_pose.getOrientation().oz;
            Pose.pose.orientation.w = zed_pose.getOrientation().ow;

	return Pose;
	
}


nav_msgs::Odometry generate_odom_msg (Pose zed_odom){

	nav_msgs::Odometry odom;
	
	// Header
	odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";
	
	// Pose
	odom.pose.pose.position.x = zed_odom.getTranslation().tx;
    odom.pose.pose.position.y = zed_odom.getTranslation().ty;
	odom.pose.pose.position.z = zed_odom.getTranslation().tz;
	
    odom.pose.pose.orientation.x = zed_odom.getOrientation().ox;
	odom.pose.pose.orientation.y = zed_odom.getOrientation().oy;
	odom.pose.pose.orientation.z = zed_odom.getOrientation().oz;
	odom.pose.pose.orientation.w = zed_odom.getOrientation().ow;

	memcpy(&odom.pose.covariance, zed_odom.pose_covariance, 36*sizeof(float));

	std::cout<<"Odom Confidence: "<<zed_odom.pose_confidence<<std::endl;

    	// Twist
  	odom.twist.twist.linear.x = zed_odom.twist[0];
    odom.twist.twist.linear.y = zed_odom.twist[1];
	odom.twist.twist.linear.z = zed_odom.twist[2];
	
	odom.twist.twist.angular.x = zed_odom.twist[3];
	odom.twist.twist.angular.y = zed_odom.twist[4];
	odom.twist.twist.angular.z = zed_odom.twist[5];

	memcpy(&odom.twist.covariance, zed_odom.twist_covariance, 36*sizeof(float));

	return odom;
}




void tf_transforms_init(){

        // ROS TF
        tf2_ros::TransformBroadcaster mTransformPoseBroadcaster;
        tf2_ros::TransformBroadcaster mTransformOdomBroadcaster;
        tf2_ros::TransformBroadcaster mTransformImuBroadcaster;

        std::string mCloudFrameId;
        std::string mPointCloudFrameId;
        std::string mMapFrameId;
        std::string mOdometryFrameId;
        std::string mBaseFrameId;
        std::string mCameraFrameId;

        // initialization Transform listener
        boost::shared_ptr<tf2_ros::Buffer> mTfBuffer;
        boost::shared_ptr<tf2_ros::TransformListener> mTfListener;


        // TF Transforms
        tf2::Transform mMap2OdomTransf;         // Coordinates of the odometry frame in map frame
        tf2::Transform mOdom2BaseTransf;        // Coordinates of the base in odometry frame
        tf2::Transform mMap2BaseTransf;         // Coordinates of the base in map frame
        tf2::Transform mMap2CameraTransf;       // Coordinates of the camera in base frame
        tf2::Transform mSensor2BaseTransf;      // Coordinates of the base frame in sensor frame
        tf2::Transform mSensor2CameraTransf;    // Coordinates of the camera frame in sensor frame
        tf2::Transform mCamera2BaseTransf;      // Coordinates of the base frame in camera frame

        // Dynamic transforms
        mOdom2BaseTransf.setIdentity();     // broadcasted if `publish_tf` is true
        mMap2OdomTransf.setIdentity();      // broadcasted if `publish_map_tf` is true
        mMap2BaseTransf.setIdentity();      // used internally, but not broadcasted
        mMap2CameraTransf.setIdentity();    // used internally, but not broadcasted

}



void transforms (){

        // Camera to Base Link (Static Transformation)
        // Save the transformation
        geometry_msgs::TransformStamped c2b = mTfBuffer->lookupTransform(mCameraFrameId, mBaseFrameId, ros::Time(0), ros::Duration(0.1));
        // Get the TF2 transformation
        tf2::fromMsg(c2b.transform, mCamera2BaseTransf);

        // Sensor to Camera Center (Static Transformation)
        // Save the transformation
        geometry_msgs::TransformStamped s2c = mTfBuffer->lookupTransform(mDepthFrameId, mCameraFrameId, ros::Time(0), ros::Duration(0.1));
        // Get the TF2 transformation
        tf2::fromMsg(s2c.transform, mSensor2CameraTransf);

        // Sensor to Base Link
        // Save the transformation
        geometry_msgs::TransformStamped s2b = mTfBuffer->lookupTransform(mDepthFrameId, mBaseFrameId, ros::Time(0), ros::Duration(0.1));
        // Get the TF2 transformation
        tf2::fromMsg(s2b.transform, mSensor2BaseTransf);
}



void TF_odom (){

        tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.x = q.y();
        transform.transform.rotation.x = q.z();
        transform.transform.rotation.x = q.w();
        br.sendTransform(transform);
}


void publish_TF_Odom (tf2::Transform odomTransf, string mOdometryFrameId, string mBaseFrameId){

        tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = mOdometryFrameId;
        transformStamped.child_frame_id = mBaseFrameId;
        // conversion from Tranform to message
        //transformStamped.transform = tf2::toMsg(odomTransf);
        // Publish transformation
        br.sendTransform(transformStamped);


}

*/