#include "Triangulation_Node.hpp"

namespace ns_3DMNE{
    Class_Triangulation_Node::Class_Triangulation_Node () : Nodelet () {}

    Class_Triangulation_Node::~Class_Triangulation_Node(){}

    void Class_Triangulation_Node::onInit(){
        NODELET_INFO_STREAM("Initializing Triangulation Node...");
        
        ros::NodeHandle n = getMTNodeHandle();
        ros::NodeHandle n_priv = getMTPrivateNodeHandle();

        // ROS Services
        Save_Reconstructed_Map = n_priv.advertiseService("Save_reconstructed_map", &Class_Triangulation_Node::Save_reconstructed_map , this);

        // ROS Subscribers
        sub_pose = n_priv.subscribe("/zed2/zed/pose", 1, &Class_Triangulation_Node::callback_pose, this);
        sub_fused_pc = n_priv.subscribe("/zed2/zed/point_cloud_fused", 1, &Class_Triangulation_Node::callback_fused_pc, this);
        NODELET_INFO_STREAM("Initialized.");

        // ROS Parameters
        n_priv.getParam("directorio_mapa", File_Path);
    }

    bool Class_Triangulation_Node::Save_reconstructed_map (std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
	    SaveMap_mode = true;
    }

    void Class_Triangulation_Node::callback_pose (const geometry_msgs::PoseStamped::Ptr& msg_pose){
	    pose_msg = msg_pose;
    }

    void Class_Triangulation_Node::callback_fused_pc(const sensor_msgs::PointCloud2::ConstPtr& msg){
	    pcl::fromROSMsg(*msg, *cloud_input); // Convert from Msg to Cloud Map

	    if (cloud_input ->size() >= last_size || first_loop){
            if (cloud_input->size()!=last_size){
                last_size = cloud_input->size();
                if (first_loop){
                    last_pose = pose_msg;
                }

                bool flag = distance_poses (pose_msg, last_pose);
                if (flag || first_loop){
                    pose2trans_rot(pose_msg);

                    cloud_cropbox = cropbox_filter(cloud_input, 1);
                    cloud_cropbox = cropbox_filter(cloud_cropbox, 2);
                    cloud_cropbox -> header.frame_id = cloud_input->header.frame_id;
                    voxel_downsampling(cloud_cropbox, cloud_voxel);
                    
                    if (cloud_voxel -> size() > 0){
                        cloud_subt = subtract_points(cloud_voxel, last_cloud);
                        *last_cloud = *cloud_voxel;
                        triangles = triangulation(cloud_subt);
                        pcl::PolygonMesh::concatenate(triangles_buffer,triangles);

                        *cloud_buffer += *cloud_voxel;
                        cloud_buffer -> header.frame_id=cloud_input->header.frame_id;

                        last_pose = pose_msg;
                    }
                }
                if (first_loop){
                    first_loop = false;
                }
            }
            else{
                NODELET_INFO_STREAM("No recents points.");
            }
        }
	    else{
		    save(triangles_buffer);
		    session_counter ++;
		    first_loop = true;
	    }

            if(SaveMap_mode){
                    save(triangles_buffer);
                    SaveMap_mode = false;
            }
    }

    bool Class_Triangulation_Node::distance_poses (geometry_msgs::PoseStamped::Ptr pose, geometry_msgs::PoseStamped::Ptr last_pose){
        float despl;
        float rot;

        despl = sqrt((pose->pose.position.x - last_pose->pose.position.x) * (pose->pose.position.x - last_pose->pose.position.x) +
        (pose->pose.position.y - last_pose->pose.position.y) * (pose->pose.position.y - last_pose->pose.position.y) +
        (pose->pose.position.z - last_pose->pose.position.z) * (pose->pose.position.z - last_pose->pose.position.z));

        rot = abs(pose->pose.orientation.x - last_pose->pose.orientation.x);
        std::cout<<rot<<std::endl;

        if (despl > 1.0 || rot > 3 * PI / 180){
            return true;
        }
        else{
            return false;
        }
    }

    void Class_Triangulation_Node::pose2trans_rot(geometry_msgs::PoseStamped::Ptr pose){
	    // Translation
        translation[0] = pose->pose.position.x;
        translation[1] = pose->pose.position.y;
        translation[2] = pose->pose.position.z;

        // Quaternion to RPY conversion
            // Roll (x-rotation)
        double sinr_cosp = 2 * (pose->pose.orientation.w * pose->pose.orientation.x + pose->pose.orientation.y * pose->pose.orientation.z);
        double cosr_cosp = 1 - 2 * (pose->pose.orientation.x * pose->pose.orientation.x + pose->pose.orientation.y * pose->pose.orientation.y);
        rotation[0] = std::atan2(sinr_cosp, cosr_cosp);

            // Pitch (y-rotation)
        double sinp = 2 * (pose->pose.orientation.w * pose->pose.orientation.y - pose->pose.orientation.z * pose->pose.orientation.x);
        if (std::abs(sinp) >= 1)
            rotation[1] = std::copysign(3.141596 / 2, sinp); // use 90 degrees if out of range
        else
            rotation[1] = std::asin(sinp);

            // Yaw (z-rotation)
        double siny_cosp = 2 * (pose->pose.orientation.w * pose->pose.orientation.z + pose->pose.orientation.x * pose->pose.orientation.y);
        double cosy_cosp = 1 - 2 * (pose->pose.orientation.y * pose->pose.orientation.y + pose->pose.orientation.z * pose->pose.orientation.z);
        rotation[2] = std::atan2(siny_cosp, cosy_cosp);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Class_Triangulation_Node::cropbox_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, int box){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::CropBox<pcl::PointXYZRGB> boxFilter;

        switch (box){
            case 1:
                boxFilter.setNegative(false);
                boxFilter.setMin(Eigen::Vector4f(0, -8, -2, 1.0));
                boxFilter.setMax(Eigen::Vector4f(6, 8, 3, 1.0));
                break;

            case 2:
                boxFilter.setNegative(true);
                boxFilter.setMin(Eigen::Vector4f(0, -1.2, -3, 1.0));
                boxFilter.setMax(Eigen::Vector4f(6, 0.8, -1.4, 1.0));
                break;
        }

        boxFilter.setTranslation(translation);
        boxFilter.setRotation(rotation);
        boxFilter.setInputCloud(cloud_input);
        boxFilter.filter(*cloud_out);

        return cloud_out;
    }

    void Class_Triangulation_Node::voxel_downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output){
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud (input);
        vg.setLeafSize (0.05f, 0.05f, 0.05f);
        vg.filter (*output);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Class_Triangulation_Node::subtract_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_buffer){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

        if(cloud_buffer->size()>0){
            // kd-tree object for searches.
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            kdtree->setInputCloud(cloud_input);

            pcl::SegmentDifferences<pcl::PointXYZRGB> p;
            pcl::PCLBase<pcl::PointXYZRGB>initCompute();
            p.setInputCloud (cloud_input);
            p.setTargetCloud (cloud_buffer);
            p.setSearchMethod (kdtree);
            p.setDistanceThreshold (0.001);
            p.segment(*cloud);
            pcl::PCLBase<pcl::PointXYZRGB>deinitCompute();
        }
        else{
            *cloud = *cloud_input;
        }
        return cloud;
    }

    pcl::PolygonMesh Class_Triangulation_Node::triangulation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input){
	    // Normal estimation
	    pcl::PolygonMesh output;
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud (input);
        n.setInputCloud (input);
        n.setSearchMethod (tree);
        n.setKSearch (80);
        n.compute (*normals);
        // * normals should not contain the point normals + surface curvatures

        // Concatenate the XYZ and normal fields*
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::concatenateFields (*input, *normals, *cloud_with_normals);

        // Create search tree*
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
        tree2->setInputCloud (cloud_with_normals);

        // Initialize objects
        pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius (0.15);

        // Set typical values for the parameters
        gp3.setMu (2);
        gp3.setMaximumNearestNeighbors (150);
        gp3.setMaximumSurfaceAngle(M_PI); // 45 degrees
        gp3.setMinimumAngle(M_PI/18); // 10 degrees
        gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
        gp3.setNormalConsistency(false);

        // Get result
        gp3.setInputCloud (cloud_with_normals);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (output);

        // Additional vertex information
        std::vector<int> parts = gp3.getPartIDs();
        std::vector<int> states = gp3.getPointStates();

	    return output;
    }

    void Class_Triangulation_Node::save(pcl::PolygonMesh input){
	    NODELET_INFO_STREAM("Saving Map...");
	
        pcl::io::savePLYFileBinary(File_Path + "Triang_Map" + std::to_string(session_counter) + ".ply", input);

        NODELET_INFO_STREAM("Map saved.");
    }
}
