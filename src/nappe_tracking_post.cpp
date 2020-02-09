/* \author Yili Qin
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pcl/pcl_config.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>

#include <cpd/version.hpp>
#include <cpd/nonrigid.hpp>

#include <iostream>
#include <chrono>
#include <string>

#include "PointSet.h"
#include <nappe_tracking_msgs/TrackResult.h>
#include <nappe_tracking_msgs/TrackPostConfig.h>

#define FRAME_BASE 30
#define DESTINATION_RATE 15
#define OUTPUT_TIME_INFO true
#define OUTPUT_DEBUG_INFO false
#define POINTS_PER_ROW 3
#define POINTS_PER_COL 10
#define ROWS_POINTSET 10
#define COLS_POINTSET 3
#define VOXEL_H_MAX 360 

/** 1 - HSV filter
 *  2 - RGB filter */
#define COLOR_FILTER_SEL 1

ros::Publisher color_filter_pub;
ros::Publisher pointset_pub;
ros::Publisher match_result_pub;
//ros::Publisher marker_pub_point;
ros::Publisher marker_pub_line;
//ros::Publisher marker_pub_text;

int cntRun = 0;
bool trackRun = false;

double voxel_ts_H_l1 = 0.0;
double voxel_ts_H_h1 = 0.0;
double voxel_ts_H_l2 = 0.0;
double voxel_ts_H_h2 = 0.0;
double voxel_ts_S_l1 = 0.0;
double voxel_ts_S_h1 = 0.0;
double voxel_ts_V_l1 = 0.0;
double voxel_ts_V_h1 = 0.0;

/** HSV - Color based filter. */
pcl::PointCloud<pcl::PointXYZRGB> * colorHSV_filter(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
	pcl::PointCloud<pcl::PointXYZHSV> * ptrCloudHSV = new (pcl::PointCloud<pcl::PointXYZHSV>); 

	auto tic_hsv = std::chrono::high_resolution_clock::now();

	for (size_t i = 0; i < cloud.size(); i++)
	{
		pcl::PointXYZHSV p;
		pcl::PointXYZRGBtoXYZHSV(cloud.points[i], p);	
		p.x = cloud.points[i].x;
		p.y = cloud.points[i].y;
		p.z = cloud.points[i].z;
		if (OUTPUT_DEBUG_INFO == true)
			std::cout << "HSV: h=" << p.h << "; s=" << p.s << "; v=" << p.v << std::endl;
		if (((voxel_ts_H_l1*VOXEL_H_MAX <= p.h && p.h <= voxel_ts_H_h1*VOXEL_H_MAX) || (voxel_ts_H_l2*VOXEL_H_MAX <= p.h && p.h <= voxel_ts_H_h2*VOXEL_H_MAX)) &&
					(voxel_ts_S_l1 <= p.s && p.s <= voxel_ts_S_h1) &&
						(voxel_ts_V_l1 <= p.v && p.v <= voxel_ts_V_h1))
		{
			ptrCloudHSV->push_back(p);
		}
	}

	pcl::PointCloud<pcl::PointXYZRGB> * ptrCloudRGB = new (pcl::PointCloud<pcl::PointXYZRGB>);
		for (size_t i = 0; i < ptrCloudHSV->size(); i++)
		{
			pcl::PointXYZRGB p;
			pcl::PointXYZHSVtoXYZRGB(ptrCloudHSV->points[i], p);	
			p.x = ptrCloudHSV->points[i].x;
			p.y = ptrCloudHSV->points[i].y;
			p.z = ptrCloudHSV->points[i].z;
			ptrCloudRGB->push_back(p);
		}

	auto toc_hsv = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> dur_hsv_ms = toc_hsv - tic_hsv; 
	if (OUTPUT_TIME_INFO == true)
		std::cout << "HSV segmentation duration(ms) >>> " << dur_hsv_ms.count() << std::endl;
	
	return ptrCloudRGB;
}	

/** RGB - Color based filter. */
pcl::PointCloud<pcl::PointXYZRGB> * colorRGB_filter(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB> * ptrColorFilter = new pcl::PointCloud<pcl::PointXYZRGB>;
	int count = 0;

	auto tic_seg = std::chrono::high_resolution_clock::now();

	for (size_t i = 0; i < cloud.size(); i++)
	{
		// For cable
		//if cloud.points[i].r < 80 && cloud.points[i].g < 80 && cloud.points[i].b > 120)
		// For paper
		//if (cloud.points[i].r < 40 && cloud.points[i].g < 40 && cloud.points[i].b > 60)
		// For nappe
		if ((100 <= cloud.points[i].r && cloud.points[i].r <= 255) &&
					(0 <= cloud.points[i].g && cloud.points[i].g <= 100) && 
						(0 <= cloud.points[i].b && cloud.points[i].b <= 100))
		{
			ptrColorFilter->push_back(cloud.points[i]);
			count++;
		}
	}
	if (OUTPUT_DEBUG_INFO == true)
		std::cout << "RGB color filter: Numbers of points >>> " << count << std::endl;

	auto toc_seg = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double, std::milli> dur_seg_ms = toc_seg - tic_seg; 
	if (OUTPUT_TIME_INFO == true)
		std::cout << "RGB segmentation duration(ms) >>> " << dur_seg_ms.count() << std::endl;

	return ptrColorFilter;
}

/** CPD matching. */
pcl::PointCloud<pcl::PointXYZRGB> * cpd_matching(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
	// pcl::PointXYZRGB -> Eigen::MatrixXd
	size_t nrows = cloud.size();
	size_t ncols = 3;
	if (OUTPUT_DEBUG_INFO == true)
		std::cout << "Fixed point cloud size (dataFixed) >>> " << nrows << std::endl;
	Eigen::MatrixXd dataFixed(nrows, ncols);
	for (size_t i = 0; i < nrows; i++)
	{
		dataFixed(i, 0) = (double)cloud.points[i].x;
		dataFixed(i, 1) = (double)cloud.points[i].y;
		dataFixed(i, 2) = (double)cloud.points[i].z;
	}

	// Generate the point set
  static gen_point_set::PointSet gen_set(POINTS_PER_ROW, POINTS_PER_COL);
	static int loop_cpd = 0;
	loop_cpd++;
	if (OUTPUT_DEBUG_INFO == true)
		std::cout << "CPD looping ... " << loop_cpd << std::endl;
	if (loop_cpd == 1)
	{
		std::cout << "Initialze CPD ... " << std::endl;
    gen_set.gen_point_set();
	}
	else 
	{
    auto tic_cpd = std::chrono::high_resolution_clock::now();

		cpd::Nonrigid nonrigid;
		nonrigid.correspondence("true");
		nonrigid.outliers(0.1);
		nonrigid.tolerance(1e-5);
		cpd::NonrigidResult result = nonrigid.run(dataFixed, gen_set.pointSet);
		gen_set.pointSet = result.points;

    auto toc_cpd = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> dur_cpd_ms = toc_cpd - tic_cpd;
    if (OUTPUT_TIME_INFO == true)
      std::cout << "CPD duration(ms) >>> " << dur_cpd_ms.count() << std::endl;
	}
  
	// Convert Eigen::MatrixXf to PointXYZRGB
	pcl::PointCloud<pcl::PointXYZRGB> * pointsDisplay = new (pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB p;

	//for (size_t i = 0; i < gen_pointset.rows(); i++)
	for (size_t i = 0; i < gen_set.pointSet.rows(); i++)
	{
		// Generate the point cloud
		p.x = gen_set.pointSet(i, 0);
		p.y = gen_set.pointSet(i, 1);
		p.z = gen_set.pointSet(i, 2);

		// Generate the point cloud
    if (i%POINTS_PER_COL == 0)
    { 
      p.r = 0; p.g = 255; p.b = 0;
    }
    else if (i%POINTS_PER_COL == POINTS_PER_COL/2)
    { 
      p.r = 0; p.g = 255; p.b = 255;
    }
    else if (i%POINTS_PER_COL == POINTS_PER_COL-1)
    { 
      p.r = 255; p.g = 255; p.b = 0;
    }
    else
    {
      p.r = 0; p.g = 0; p.b = 255;
    }
		// Push the generated point
		pointsDisplay->push_back(p);
	}

	return pointsDisplay;
}

/** Generate and publish the markers for visualization */
void generate_marker(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
		// Generate markers
//			// Marker: points (CPD result)
//			visualization_msgs::Marker point_list;
//			point_list.header.frame_id = "camera_depth_optical_frame";
//			point_list.header.stamp = ros::Time::now();
//			point_list.ns = "points";
//			point_list.action = visualization_msgs::Marker::ADD;
//			point_list.pose.orientation.w = 1.0;
//			point_list.id = 0;
//			point_list.type = visualization_msgs::Marker::POINTS;
//			point_list.scale.x = 0.006;
//			point_list.scale.y = 0.006;
//			point_list.color.g = 1.0;
//			point_list.color.a = 1.0;
//
//			for (size_t i = 0; i < POINTS_PER_ROW * POINTS_PER_COL; i++)
//			{
//				geometry_msgs::Point p;
//				p.x = cloud.points[i].x;
//				p.y = cloud.points[i].y;
//				p.z = cloud.points[i].z;
//				point_list.points.push_back(p);
//			}
//			marker_pub_point.publish(point_list);

		// Marker: lines
		visualization_msgs::Marker line_list;
		line_list.header.frame_id = "camera_depth_optical_frame";
		line_list.header.stamp = ros::Time::now();
		line_list.ns = "lines";
		line_list.action = visualization_msgs::Marker::ADD;
		line_list.pose.orientation.w = 1.0;
		line_list.id = 1;
		line_list.type = visualization_msgs::Marker::LINE_LIST;
		line_list.scale.x = 0.004;
		line_list.color.r = 1.0;
		line_list.color.a = 1.0;
		for (size_t i = 0; i < POINTS_PER_ROW; i++)
		{
			for (size_t j = 0; j < (POINTS_PER_COL - 1); j++)
			{
				geometry_msgs::Point p;
				int n = i * POINTS_PER_COL + j;
				p.x = cloud.points[n].x;  
				p.y = cloud.points[n].y;  
				p.z = cloud.points[n].z;  
				line_list.points.push_back(p);
				p.x = cloud.points[n+1].x;  
				p.y = cloud.points[n+1].y;  
				p.z = cloud.points[n+1].z;  
				line_list.points.push_back(p);
			}
		}
		for (size_t i = 0; i < POINTS_PER_COL; i++)
		{
			for (size_t j = 0; j < (POINTS_PER_ROW - 1); j++)
			{
				geometry_msgs::Point p;
				int n = i + j * POINTS_PER_COL;
				p.x = cloud.points[n].x;  
				p.y = cloud.points[n].y;  
				p.z = cloud.points[n].z;  
				line_list.points.push_back(p);
				p.x = cloud.points[n+POINTS_PER_COL].x;  
				p.y = cloud.points[n+POINTS_PER_COL].y;  
				p.z = cloud.points[n+POINTS_PER_COL].z;  
				line_list.points.push_back(p);
			}
		}
		marker_pub_line.publish(line_list);

//			// Marker: texts
//			visualization_msgs::Marker text_list;
//			text_list.header.frame_id = "camera_depth_optical_frame";
//			text_list.header.stamp = ros::Time::now();
//			text_list.ns = "basic_shapes";
//			text_list.action = visualization_msgs::Marker::ADD;
//			text_list.pose.orientation.w = 1.0;
//			text_list.id = 2;
//			text_list.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//			text_list.pose.position.x = 0.0;
//			text_list.pose.position.y = 0.0;
//			text_list.pose.position.z = 0.0;
//			text_list.scale.x = 1.0;
//			text_list.scale.y = 1.0;
//			text_list.scale.z = 1.0;
//			text_list.color.r = 1.0;
//			text_list.color.g = 0.0;
//			text_list.color.b = 0.0;
//			text_list.text="bababa";
//				
//			marker_pub_text.publish(text_list);

}

/** Tracking the surface of the deformable object. */
void nappeTrackingPostCallback(const sensor_msgs::PointCloud2ConstPtr & msg)
{
  
	if (trackRun == true)
	{
		cntRun++;

		if (OUTPUT_DEBUG_INFO == true)
			std::cout << "Callback running ... " << cntRun << std::endl;

		if (cntRun == FRAME_BASE / DESTINATION_RATE)
		{
			cntRun = 0;

			// Convert pcl::PCLPointCloud2 to pcl::PointCloud<T>
			pcl::PointCloud<pcl::PointXYZRGB> * ptrCloudRGB = new pcl::PointCloud<pcl::PointXYZRGB>;
			pcl::fromROSMsg(*msg, *ptrCloudRGB);

			// Color filter
			pcl::PointCloud<pcl::PointXYZRGB> * ptrColorFilter = new (pcl::PointCloud<pcl::PointXYZRGB>);
			switch (COLOR_FILTER_SEL)
			{
				case 1:	ptrColorFilter = colorHSV_filter(*ptrCloudRGB); break;
				case 2: ptrColorFilter = colorRGB_filter(*ptrCloudRGB); break;
				default: break;
			}

			// pcl::PCLPointCloud2 -> pcl::PointCloud<T>
			sensor_msgs::PointCloud2 output;

			// Publish color filter result 
			pcl::toROSMsg(*ptrColorFilter, output);
			output.header.stamp = ros::Time::now();
			output.header.frame_id = "camera_depth_optical_frame";
			color_filter_pub.publish(output);

			// CPD matching
			pcl::PointCloud<pcl::PointXYZRGB> * ptrResultCPD = new (pcl::PointCloud<pcl::PointXYZRGB>);
			ptrResultCPD = cpd_matching(*ptrColorFilter);

			// Publish CPD match result for rviz
			pcl::toROSMsg(*ptrResultCPD, output);
			output.header.stamp = ros::Time::now();
			output.header.frame_id = "camera_depth_optical_frame";
			pointset_pub.publish(output);

			// Publish CPD match result for mc_rtc
			nappe_tracking_msgs::TrackResult pubTrackResultMsg;
			pubTrackResultMsg.row = ROWS_POINTSET;
			pubTrackResultMsg.col = COLS_POINTSET; 
			pubTrackResultMsg.data.resize(POINTS_PER_ROW * POINTS_PER_COL);
			for (size_t i = 0; i < POINTS_PER_ROW * POINTS_PER_COL; i++)
			{
				pubTrackResultMsg.data[i].x = ptrResultCPD->points[i].x; 
				pubTrackResultMsg.data[i].y = ptrResultCPD->points[i].y; 
				pubTrackResultMsg.data[i].z = ptrResultCPD->points[i].z; 
			}
			// Publish Registration Result
			match_result_pub.publish(pubTrackResultMsg);

			generate_marker(*ptrResultCPD);
		}
	}
	else
	{
		cntRun = 0;
	}
}

/** Process the command from nappe controller. */
void postConfigCallback(const nappe_tracking_msgs::TrackPostConfig & msg)
{
	voxel_ts_H_l1 = msg.voxel_ts_H_l1;
	voxel_ts_H_h1 = msg.voxel_ts_H_h1;
	voxel_ts_H_l2 = msg.voxel_ts_H_l2;
	voxel_ts_H_h2 = msg.voxel_ts_H_h2;
	voxel_ts_S_l1 = msg.voxel_ts_S_l1;
	voxel_ts_S_h1 = msg.voxel_ts_S_h1;
	voxel_ts_V_l1 = msg.voxel_ts_V_l1;
	voxel_ts_V_h1 = msg.voxel_ts_V_h1;

	if (msg.track_cmd == "Start")
	{
		std::cout << "Start tracking ..." << std::endl;
		trackRun = true;
	}
	else if (msg.track_cmd == "Finish")
	{
		std::cout << "Finish tracking ..." << std::endl;
		trackRun = false;
	}
	else
		trackRun = trackRun;
}

int main(int argc, char * argv[])
{

	// Print out system info
	std::cout << "PCL Version: " << PCL_VERSION << std::endl;
	std::cout << "CPD Version: " << cpd::version() << std::endl; 
	if (COLOR_FILTER_SEL == 1)
		std::cout << "Color filter: HSV" << std::endl;
	else if (COLOR_FILTER_SEL == 2)
		std::cout << "Color filter: RGB" << std::endl;
	else
		;
	std::cout << "Nappe Post-process running ... " << std::endl;

	// Initialize ROS
	ros::init(argc, argv, "nappe_tracking_post");
	ros::NodeHandle nh;

	// Create ROS subscriber & publisher 
	ros::Subscriber tracking_pre_sub = nh.subscribe("/nappe/filter/voxel", 1, nappeTrackingPostCallback);
	ros::Subscriber controller_sub = nh.subscribe("/nappe/config/post", 1, postConfigCallback);

	color_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("/nappe/filter/color", 1);
	pointset_pub = nh.advertise<sensor_msgs::PointCloud2>("/nappe/pc/pointset", 1);
	match_result_pub = nh.advertise<nappe_tracking_msgs::TrackResult>("/nappe/controller/registration", 1);
  marker_pub_line = nh.advertise<visualization_msgs::Marker>("/nappe/marker/lines", 1);
  //marker_pub_text = nh.advertise<visualization_msgs::Marker>("/nappe/marker/texts", 1);
	//marker_pub_point = nh.advertise<visualization_msgs::Marker>("/nappe/marker/points", 1);

	// Spin
	ros::spin();
	
	return 0;
}
