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
#include <pcl/filters/voxel_grid.h>
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
#include <nappe_tracking_msgs/TrackingResult.h>
#include <nappe_tracking_msgs/TrackingCmd.h>

#define FRAME_BASE 30
#define DESTINATION_RATE 15
#define OUTPUT_TIME_INFO true
#define OUTPUT_DEBUG_INFO false
#define POINTS_PER_ROW 3
#define POINTS_PER_COL 10
#define ROW_POINTSET 10
#define COL_POINTSET 3
/* 1 - Voxel filtering result
 * 2 - Color filtering result
 *
 */
#define DISPLAY_SELECT 2

/* 1 - HSV filter
 * 2 - RGB filter
 *
 */
#define COLOR_FILTER_SEL 1
#define TRACK_START 20
#define TRACK_FINISH 30

ros::Publisher gen_set_pub;
ros::Publisher cloud_pub;
ros::Publisher post_result_pub;
ros::Publisher marker_pub_line;
ros::Publisher marker_pub_text;

int cntRun = 0;
bool trackRun = false;

/* Voxel filter. */
pcl::PointCloud<pcl::PointXYZRGB> * voxel_filter(pcl::PointCloud<pcl::PointXYZRGB> & cloud)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrTmp(&cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cptrCloud(ptrTmp);
	pcl::PointCloud<pcl::PointXYZRGB> * ptrCloudVoxel = new (pcl::PointCloud<pcl::PointXYZRGB>);

	auto tic_voxel = std::chrono::high_resolution_clock::now();

	vg.setInputCloud(cptrCloud);
	// For cable
	//ds.setLeafSize(0.01, 0.01, 0.01);
	// For paper
	vg.setLeafSize(0.02, 0.02, 0.02);
	// For nappe
	//vg.setLeafSize(0.02, 0.02, 0.02);
	vg.setLeafSize(0.04, 0.04, 0.04);
	vg.filter(*ptrCloudVoxel);

	auto toc_voxel = std::chrono::high_resolution_clock::now();
	std::chrono::microseconds dur_ms;
	std::chrono::duration<double, std::milli> dur_voxel_ms = toc_voxel - tic_voxel; 
	if (OUTPUT_TIME_INFO == true)
			std::cout << "Voxel filtering duration(ms) >>> " << dur_voxel_ms.count() << std::endl;

	return ptrCloudVoxel;
}

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
		if (((0 <= p.h && p.h <= 30) || (320 <= p.h && p.h <= 360)) &&
					(0.45 <= p.s && p.s <= 0.99) &&
						(0.23 <= p.v && p.v <= 0.4))
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
      p.r = 0;
      p.g = 255;
      p.b = 0;
    }
    else if (i%POINTS_PER_COL == POINTS_PER_COL/2)
    { 
      p.r = 0;
      p.g = 255;
      p.b = 255;
    }
    else if (i%POINTS_PER_COL == POINTS_PER_COL-1)
    { 
      p.r = 255;
      p.g = 255;
      p.b = 0;
    }
    else
    {
      p.r = 0;
      p.g = 0;
      p.b = 255;
    }
		// Push the generated point
		pointsDisplay->push_back(p);
	}

	return pointsDisplay;
}

/** Tracking the surface of the deformable object. */
void nappeTrackingPostCallback(const sensor_msgs::PointCloud2ConstPtr & input)
{
  
	if (trackRun == true)
	{
		cntRun++;

		if (OUTPUT_DEBUG_INFO == true)
			std::cout << "Tracking ... " << cntRun << std::endl;

		if (cntRun == FRAME_BASE / DESTINATION_RATE)
		{
			cntRun = 0;

			// Convert pcl::PCLPointCloud2 to pcl::PointCloud<T>
			pcl::PointCloud<pcl::PointXYZRGB> * ptrCloudRGB = new pcl::PointCloud<pcl::PointXYZRGB>;
			pcl::fromROSMsg(*input, *ptrCloudRGB);

			//// Voxel filter
			//pcl::PointCloud<pcl::PointXYZRGB> * ptrVoxelFilter = new pcl::PointCloud<pcl::PointXYZRGB>;
			//ptrVoxelFilter = voxel_filter(*ptrCloudRGB);

			// Color filter
			pcl::PointCloud<pcl::PointXYZRGB> * ptrColorFilter = new (pcl::PointCloud<pcl::PointXYZRGB>);
			switch (COLOR_FILTER_SEL)
			{
				//case 1:	ptrColorFilter = colorHSV_filter(*ptrVoxelFilter); break;
				//case 2: ptrColorFilter = colorRGB_filter(*ptrVoxelFilter); break;
				case 1:	ptrColorFilter = colorHSV_filter(*ptrCloudRGB); break;
				case 2: ptrColorFilter = colorRGB_filter(*ptrCloudRGB); break;
				default: break;
			}
			
			// CPD matching
			pcl::PointCloud<pcl::PointXYZRGB> * ptrResultCPD = new (pcl::PointCloud<pcl::PointXYZRGB>);
			ptrResultCPD = cpd_matching(*ptrColorFilter);
			
			// Convert pcl::PCLPointCloud2 to pcl::PointCloud<T>
			pcl::PointCloud<pcl::PointXYZRGB> * displayCloud = new (pcl::PointCloud<pcl::PointXYZRGB>);
			sensor_msgs::PointCloud2 output;

			// Publish cloud
			switch (DISPLAY_SELECT)
			{
				//case 1: displayCloud = ptrVoxelFilter; break;
				case 1: displayCloud = ptrCloudRGB; break;
				case 2: displayCloud = ptrColorFilter; break;
				default: break;
			}
			pcl::toROSMsg(*displayCloud, output);
			output.header.stamp = ros::Time::now();
			output.header.frame_id = "camera_rgb_optical_frame";
			cloud_pub.publish(output);

			// Publish CPD result for mc_rtc
			nappe_tracking_msgs::TrackingResult pubTrackingResultMsg;
			pubTrackingResultMsg.row = ROW_POINTSET;
			pubTrackingResultMsg.col = COL_POINTSET; 
			pubTrackingResultMsg.data.resize(POINTS_PER_ROW * POINTS_PER_COL);
			for (size_t i = 0; i < POINTS_PER_ROW * POINTS_PER_COL; i++)
			{
				//pubTrackingResultMsg.data[i].x = 0.1; 
				//pubTrackingResultMsg.data[i].y = 0.2; 
				//pubTrackingResultMsg.data[i].z = 0.3; 
				pubTrackingResultMsg.data[i].x = ptrResultCPD->points[i].x; 
				pubTrackingResultMsg.data[i].y = ptrResultCPD->points[i].y; 
				pubTrackingResultMsg.data[i].z = ptrResultCPD->points[i].z; 

			}

			post_result_pub.publish(pubTrackingResultMsg);

			// Publish CPD result as marker
			displayCloud = ptrResultCPD;
			pcl::toROSMsg(*displayCloud, output);
			output.header.stamp = ros::Time::now();
			output.header.frame_id = "camera_rgb_optical_frame";
			gen_set_pub.publish(output);

			// Generate line markers and publish them
			visualization_msgs::Marker line_list;
			line_list.header.frame_id = "camera_rgb_optical_frame";
			line_list.header.stamp = ros::Time::now();
			line_list.ns = "points_and_lines";
			line_list.action = visualization_msgs::Marker::ADD;
			line_list.pose.orientation.w = 1.0;
			line_list.id = 2;
			line_list.type = visualization_msgs::Marker::LINE_LIST;
			line_list.scale.x = 0.004;
			line_list.color.r = 1.0;
			line_list.color.a = 1.0;
			geometry_msgs::Point p;
			for (size_t i = 0; i < POINTS_PER_ROW; i++)
			{
				for (size_t j = 0; j < (POINTS_PER_COL - 1); j++)
				{
					int n = i * POINTS_PER_COL + j;
					p.x = ptrResultCPD->points[n].x;  
					p.y = ptrResultCPD->points[n].y;  
					p.z = ptrResultCPD->points[n].z;  
					line_list.points.push_back(p);
					p.x = ptrResultCPD->points[n+1].x;  
					p.y = ptrResultCPD->points[n+1].y;  
					p.z = ptrResultCPD->points[n+1].z;  
					line_list.points.push_back(p);
				}
			}
			for (size_t i = 0; i < POINTS_PER_COL; i++)
			{
				for (size_t j = 0; j < (POINTS_PER_ROW - 1); j++)
				{
					int n = i + j * POINTS_PER_COL;
					p.x = ptrResultCPD->points[n].x;  
					p.y = ptrResultCPD->points[n].y;  
					p.z = ptrResultCPD->points[n].z;  
					line_list.points.push_back(p);
					p.x = ptrResultCPD->points[n+POINTS_PER_COL].x;  
					p.y = ptrResultCPD->points[n+POINTS_PER_COL].y;  
					p.z = ptrResultCPD->points[n+POINTS_PER_COL].z;  
					line_list.points.push_back(p);
				}
			}
			marker_pub_line.publish(line_list);

			// Generate text markers and publish them
			visualization_msgs::Marker text_list;
			text_list.header.frame_id = "camera_rgb_optical_frame";
			text_list.header.stamp = ros::Time::now();
			text_list.ns = "basic_shapes";
			text_list.action = visualization_msgs::Marker::ADD;
			//text_list.pose.orientation.w = 1.0;
			text_list.id = 100;
			text_list.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			text_list.pose.position.x = 0.0;
			text_list.pose.position.y = 0.0;
			text_list.pose.position.z = 0.0;
			text_list.scale.x = 1.0;
			text_list.scale.y = 1.0;
			text_list.scale.z = 1.0;
			text_list.color.r = 1.0;
			text_list.color.g = 0.0;
			text_list.color.b = 0.0;
			text_list.text="bababa";
			
			marker_pub_text.publish(text_list);

		}
	}
	else
	{
		cntRun = 0;
	}
}

/** Process the command from nappe controller. */
void visionCmdCallback(const nappe_tracking_msgs::TrackingCmd & msg)
{
	std::cout << "visionCmdCallback  ..." << std::endl;

	if (msg.cmd == TRACK_START)
	{
		std::cout << "Start the vision ..." << std::endl;
		trackRun = true;
	}
	else if (msg.cmd == TRACK_FINISH)
	{
		std::cout << "Finish the vision ..." << std::endl;
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
	std::cout << "Nappe Post-processing ...  " << std::endl;

	// Initialize ROS
	ros::init(argc, argv, "nappe_tracking_post");
	ros::NodeHandle nh;

	// Create ROS subscriber & publisher 
	ros::Subscriber tracking_pre_sub = nh.subscribe("/nappe/filter/voxel", 1, nappeTrackingPostCallback);
	ros::Subscriber controller_sub = nh.subscribe("/nappe/vision_cmd", 1, visionCmdCallback);

	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/nappe/filter/color", 1);
	post_result_pub = nh.advertise<nappe_tracking_msgs::TrackingResult>("/nappe/registration", 1);
  marker_pub_line = nh.advertise<visualization_msgs::Marker>("/nappe/marker/grid", 10);
  marker_pub_text = nh.advertise<visualization_msgs::Marker>("/nappe/marker/text", 10);
	gen_set_pub = nh.advertise<sensor_msgs::PointCloud2>("/nappe/marker/result", 1);

	// Spin
	ros::spin();
	
	return 0;
}
