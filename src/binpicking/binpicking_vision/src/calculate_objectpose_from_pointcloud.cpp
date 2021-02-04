#include <ros/ros.h>
#include "binpicking_vision/CalculateObjectposeFromPointcloud.h"
#include "binpicking_vision/CalculateObjectposeFromPointcloudRequest.h"
#include "binpicking_vision/CalculateObjectposeFromPointcloudResponse.h"
#include "binpicking_vision/CapturePointcloud.h"
#include "binpicking_vision/CapturePointcloud.h"
#include "binpicking_vision/CapturePointcloud.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//Image message
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/common.h>
#include <pcl/PointIndices.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>

#include <geometry_msgs/Vector3.h>

#include <bits/stdc++.h>
#include <math.h>       /* sqrt */
#include <vector>
#include <iostream>
#include <exception>
#include <algorithm>

sensor_msgs::Image image; //cache the image message

using namespace std;
using namespace pcl;
geometry_msgs::Vector3 calculate_objectpose_from_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,int &degrees) 
{

	/*Create PTr search tree for normals*/
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	//Search for clusters 
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(140);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(120);
	reg.setInputCloud(cloud);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	ROS_INFO("calculate_oject_pose_from_pointcloud: Number of clusters/objects is equal to %i", clusters.size());

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	int cnt = 0;
	pcl::PointXYZ CenterPoint;
	pcl::PointXYZ minPtN, maxPtN; //N for nearest
	
	pcl::PointXYZ minX(10,10,10);
	pcl::PointXYZ minY(10,10,10);
	pcl::PointXYZ maxY(-10,-10,-10);
	pcl::PointXYZ maxX(-10,-10,-10);

	pcl::PointXYZ CenterPointMin;
	// Find neraest object
	for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
	{	
		/* Create pointcloud from cluster */
		cloud_cluster->points.clear();
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud->points[*pit]); 
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// Find centroid of the object
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud_cluster, centroid);
		

		// Find distance of the object
		pcl::PointXYZ minPt, maxPt;
		pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

		// Determinate neraest object from previos objects
		CenterPoint.x = centroid[0];
		CenterPoint.y = centroid[1];
		CenterPoint.z = minPt.z;
		
		if(!cnt){
			CenterPointMin = CenterPoint;
			minPtN = minPt;
			maxPtN = maxPt;
			for(pcl::PointXYZ point : cloud_cluster->points)
			{
				if(point.x > maxX.x) maxX = point;
				if(point.y > maxY.y) maxY = point;
				if(point.x < minX.x) minX = point;
				if(point.y < minY.y) minY = point;
			}
		}
		else{
			if(CenterPoint.z < CenterPointMin.z && CenterPoint.z > 0.4)
			{
				CenterPointMin = CenterPoint;
				minPtN = minPt;
				maxPtN = maxPt;
				for(pcl::PointXYZ point : cloud_cluster->points)
				{
					if(point.x > maxX.x) maxX = point;
					if(point.y > maxY.y) maxY = point;
					if(point.x < minX.x) minX = point;
					if(point.y < minY.y) minY = point;
				}
			}
		}
		ROS_INFO("calculate_oject_pose_from_pointcloud: The middlepoint of object (%i): %f, %f, %f (x,y,z) %i (Size)", cnt, CenterPoint.x, CenterPoint.y, CenterPoint.z,cloud_cluster->points.size()); 
		cnt++;
	}
	//std::cout << "points: maxX " << maxX  << "minX" << minX << " maxY " << maxY << " minY " << minY << std::endl;
	// "maxY.x" << maxY.x << endl;
	if(maxY.x > -10)
	{
		try{
			//pcl::PointXYZ bottomLeft;
			//pcl::PointXYZ bottomRight;
			//pcl::PointXYZ topLeft;
			//pcl::PointXYZ topRight;
			
			//vector<PointXYZ> bottom;
			//vector<PointXYZ> top;

			//if(minX.y > CenterPoint.y) top.push_back(minX);
			//else bottom.push_back(minX);
			//if(minY.y > CenterPoint.y) top.push_back(minY);
			//else bottom.push_back(minY);
			//if(maxX.y > CenterPoint.y) top.push_back(maxX);
			//else bottom.push_back(maxX);
			//if(maxY.y > CenterPoint.y) top.push_back(maxY);
			//else bottom.push_back(maxY);

			//cout << "bottom: " << bottom[0] << " bottom: " << bottom[1] << endl;
			//cout << "top: " << top[0] << " top: " << top[1] << endl;

			//if(bottom[0].x < bottom[1].x)
			//{
			//	bottomLeft = bottom[0];
			//	bottomRight = bottom[1];
			//}else{
			//	bottomLeft = bottom[1];
			//	bottomRight = bottom[0];
			//}

			//if(top[0].x < top[1].x)
			//{
			//	topLeft = top[0];
			//	topRight = top[1];
			//}else{
			//	topLeft = top[1];
			//	topRight = top[0];
			//}

			//cout << "bottomLeft: " << bottomLeft << " bottomRight: " << bottomRight << " topLeft: " <<
			//topLeft << " topRight: " << topRight << endl;



			float minX_maxY = sqrt(pow(minX.x - maxY.x,2) + pow(minX.y - maxY.y,2));
			float maxY_maxX = sqrt(pow(maxY.x - maxX.x,2) + pow(maxY.y - maxX.y,2));
			float maxX_minY = sqrt(pow(maxX.x - minY.x,2) + pow(maxX.y - minY.y,2));
			float minY_minX = sqrt(pow(minY.x - minX.x,2) + pow(minY.y - minX.y,2));

			//float botLeft_topLeft = sqrt(pow(bottomLeft.x - topLeft.x,2) + pow(bottomLeft.y - topLeft.y,2));
			//float topLeft_topRight = sqrt(pow(topLeft.x - topRight.x,2) + pow(topLeft.y - topRight.y,2));
			//float topRight_botRight = sqrt(pow(topRight.x - bottomRight.x,2) + pow(topRight.y - bottomRight.y,2));
			//float botRight_botLeft = sqrt(pow(bottomRight.x - bottomLeft.x,2) + pow(bottomRight.y - bottomLeft.y,2));

			//cout << "botLeft_topLeft: " << botLeft_topLeft << " topLeft_topRight: " << topLeft_topRight
			// << " topRight_botRight: " << topRight_botRight << " botRight_botLeft: " << botRight_botLeft << endl;

			//cout << "minX_maxY: " << minX_maxY << " maxY_maxX: " << maxY_maxX
			//<< " maxX_minY: " << maxX_minY << " minY_minX: " << minY_minX << endl;
			vector<float> sides;
			sides.push_back(minX_maxY);
			sides.push_back(maxY_maxX);
			sides.push_back(maxX_minY);
			sides.push_back(minY_minX);

			////float lenghtDig = sqrt(pow(maxPtN.x - minPtN.x,2) + pow(maxPtN.y - minPtN.y,2));
			////cout << "BIngo"  << lenghtDig << endl;

			//float maxSide = *std::max_element(begin(sides), end(sides));
			//std::vector<float>::iterator it = std::find(sides.begin(), sides.end(), maxSide);
			//int index = std::distance(sides.begin(), it);
			float turnDegreesinRad = 0;
			turnDegreesinRad = 0.5 * M_PI - acos(abs(maxX.x -minY.x) / maxX_minY);

			////turnDegreesinRad =  atan( (maxPtN.y - minPtN.y) / (maxPtN.x - minPtN.x));

			//turnDegreesinRad = 2 * (acos(abs(maxPtN.x -minPtN.x) / lenghtDig));

			//cout << " index = " << index << endl;
			//cout << " maxSide = " << maxSide << endl;
			//switch(index){
			//	case 0:
			//	turnDegreesinRad = acos(abs(minX.x -maxY.x) / maxSide);
			//	break;
			//	case 1:
			//	turnDegreesinRad = acos(abs(maxY.x -maxX.x) / maxSide);
			//	break;
			//	case 2:
			//	turnDegreesinRad = acos(abs(maxX.x -minY.x) / maxSide);
			//	break;
			//	case 3:
			//	turnDegreesinRad = acos(abs(minY.x -minX.x) / maxSide);
			//	break;
			//}
			
			degrees = round(turnDegreesinRad * (180 / M_PI));

		}catch (std::exception &ex) {
			std::cerr << ex.what();
		}
	}
	

	
	
	ROS_INFO("calculate_oject_pose_from_pointcloud: The centerpoint of the nearest object: %f, %f, %f (x,y,z)", CenterPointMin.x, CenterPointMin.y, CenterPointMin.z); 

	geometry_msgs::Vector3 result;
	
	result.x = CenterPointMin.x;
	result.y = CenterPointMin.y;
	result.z = CenterPointMin.z;

	return result;
}

bool CalculateObjectposeFromPointcloud(binpicking_vision::CalculateObjectposeFromPointcloud::Request  &request,
         binpicking_vision::CalculateObjectposeFromPointcloud::Response &response)
{

	ROS_INFO("calculate_oject_pose_from_pointcloud: CalculateObjectposeFromPointcloud start");
	
	/*Convert pointcloud from request to pcl2*/
	pcl::PCLPointCloud2* point_cloud2 = new pcl::PCLPointCloud2;
	pcl_conversions::toPCL(request.pointcloud, *point_cloud2);  // From ROS-PCL to PCL2

	/*Convert pointcloud from pcl2 to pcl*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*point_cloud2, *point_cloud); // From PCL2 to PCL


	/*Use a CropBox filter to filter out the robot and buttons from the camera*/
	pcl::CropBox<pcl::PointXYZ> boxFilter;

	/*Filter for camera 1*/
	if(request.camera == 1)
	{
		boxFilter.setMin(Eigen::Vector4f(-0.5, -0.5, 0, 1.0));
		boxFilter.setMax(Eigen::Vector4f(0,-0.08, 0.8, 1.0));
		boxFilter.setNegative(true);

		/*Create a filter for the egde*/
		pcl::CropBox<pcl::PointXYZ> boxFilterEdge;
		boxFilterEdge.setMin(Eigen::Vector4f(-0.22, -0.22, 0, 1.0));
		boxFilterEdge.setMax(Eigen::Vector4f(0.23,0.19, 0.8, 1.0));
		boxFilterEdge.setInputCloud(point_cloud);
		boxFilterEdge.filter(*point_cloud);
	}else {
		boxFilter.setMin(Eigen::Vector4f(0.020, -0.5, 0, 1.0));
		boxFilter.setMax(Eigen::Vector4f(0.4,-0.10, 0.8, 1.0));
		boxFilter.setNegative(true);

		/*Create a filter for the stop button*/
		pcl::CropBox<pcl::PointXYZ> boxFilter;
		boxFilter.setMin(Eigen::Vector4f(-0.5, 0.12, 0, 1.0));
		boxFilter.setMax(Eigen::Vector4f(-0.19,0.5, 0.8, 1.0));
		boxFilter.setNegative(true);
		boxFilter.setInputCloud(point_cloud);
		boxFilter.filter(*point_cloud);

		/*Create a filter for the egde*/
		pcl::CropBox<pcl::PointXYZ> boxFilterEdge;
		boxFilterEdge.setMin(Eigen::Vector4f(-0.22, -0.22, 0, 1.0));
		boxFilterEdge.setMax(Eigen::Vector4f(0.23,0.16, 0.8, 1.0));
		boxFilterEdge.setInputCloud(point_cloud);
		boxFilterEdge.filter(*point_cloud);
	}

	boxFilter.setInputCloud(point_cloud);
	int degrees;
	pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	boxFilter.filter(*bodyFiltered);

	/*Call the fuction that calculates the object position from the camera*/ 
	geometry_msgs::Vector3 object_position = calculate_objectpose_from_pointcloud(bodyFiltered,degrees);
	if(request.camera == 1) degrees = -1 * degrees;
	if(object_position.z == 0)
	{
		response.degrees = 0;
		response.succes = false;
		return true;
	}
	response.degrees = degrees;

	/*Create the variables need for transforming the object to the world*/
	geometry_msgs::TransformStamped static_transformStamped;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;


	ROS_INFO("calculate_oject_pose_from_pointcloud: add static transform to object");
	/* add static transform to object */
	static_transformStamped.header.stamp = ros::Time::now();
	static_transformStamped.header.frame_id = "camera" +  std::to_string(request.camera) + "_depth_optical_frame";
	static_transformStamped.child_frame_id = "object_to_grasp";

	static_transformStamped.transform.translation = object_position;
	static_transformStamped.transform.rotation.x = 0;
	static_transformStamped.transform.rotation.y = 0;
	static_transformStamped.transform.rotation.z = 0;
	static_transformStamped.transform.rotation.w = 1.0;
	static_broadcaster.sendTransform(static_transformStamped);

	/* wait a while is needed*/
	ros::Duration(0.8).sleep();
	ROS_INFO("calculate_oject_pose_from_pointcloud: get object pose relative to world ");
	/* get object pose relative to world */
	try{
		geometry_msgs::TransformStamped transformStamped;
		geometry_msgs::PoseStamped poseStamped;
		poseStamped.pose.orientation.x = 0;
		poseStamped.pose.orientation.y = 0;
		poseStamped.pose.orientation.z = 0;
		poseStamped.pose.orientation.w = 1.0;
		transformStamped = tfBuffer.lookupTransform("world", "object_to_grasp", ros::Time(0));
		tf2::doTransform(poseStamped, poseStamped, transformStamped); // object_to_grasp is the PoseStamped I want to transform

		static tf2_ros::StaticTransformBroadcaster static_broadcaster;
		geometry_msgs::TransformStamped static_transformStamped;

		static_transformStamped.header.stamp = ros::Time::now();
		static_transformStamped.header.frame_id = "world";
		static_transformStamped.child_frame_id = "object_to_grasp_capture";
		static_transformStamped.transform.translation.x = poseStamped.pose.position.x;
		static_transformStamped.transform.translation.y = poseStamped.pose.position.y;
		static_transformStamped.transform.translation.z = poseStamped.pose.position.z;

		static_transformStamped.transform.rotation = poseStamped.pose.orientation;
		static_broadcaster.sendTransform(static_transformStamped);

		response.succes = true;
		response.object_pose = poseStamped;

	}
	catch (tf2::TransformException &ex) {
		ROS_ERROR("calculate_oject_pose_from_pointcloud: Error lookupTransform.");
		response.succes = false;
		response.degrees = 0;
	}


	ROS_INFO("calculate_oject_pose_from_pointcloud: CalculateObjectposeFromPointcloud exit");
	/* return object pose in response*/
	return true;
}

int main(int argc, char **argv)
{
	/*Init the rosnode and ros*/
	ros::init(argc, argv, "calculate_object_pose_server");
	ros::NodeHandle n;

	/*Create service calculate_object_pose*/
	ros::ServiceServer service = n.advertiseService("calculate_object_pose", CalculateObjectposeFromPointcloud);

	ROS_INFO("calculate_oject_pose_from_pointcloud: calculate_object_pose Service is now available");
	ros::spin();

	return 0;
}