# TFM-ZedMappingNode
This is my Master's thesis titled “Construction of Three-Dimensional Maps in Unstructured Environments for Search and Rescue Tasks Using Stereo Vision.”

The thesis aims to develop a real-time algorithm for constructing high-fidelity 3D maps in unstructured, natural environments during robotic rescue operations, based on SLAM techniques. This objective was achieved by employing open-source software—including ROS Melodic Morenia, the Point Cloud Library (PCL), and C++—all containerized with Docker, along with a commercial stereo camera ZED of StereoLabs.

The project was divided into two stages: first, generating a point cloud for each scene; and second, reconstructing a 3D map from the generated point cloud. In order to optimize resou![Prueba_nodo](https://github.com/user-attachments/assets/09147b59-dcf9-455f-9dde-3218a41d8a69)
rces, communication between the processes was established via ROS Nodelets, enabling the efficient transfer of millions of points with low latency.

To validate the algorithm’s performance, an exhaustive comparative study was conducted against other widely used approaches, such as RTAB-Map and the one provided by the manufacturer under various conditions (e.g., different extents, speeds, and scenarios). The results demonstrate that the 3DMNE algorithm is more robust than the alternatives tested.
