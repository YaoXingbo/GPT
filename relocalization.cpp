#include <ros/ros.h>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//#include <opencv/cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "slam_algorithm/srv_relocalization.h"  //edit


/*
构图时，不知道将要覆盖多少面积，所以不能确定一个包含所有点的三维栅格数组；所以限定了一个最大index，不断移动栅格区域。
重定位时，先验地图已经确定，可以直接给出三维栅格数组的最大index，所以直接设置一个大数组即可。
（问题：所以先验地图太大时，内存可能承受不了，得有一个动态读取先验文件的机制）
*/

typedef pcl::PointXYZI  PointType;

const double PI = 3.1415926;
const float scanPeriod = 0.1;

const int stackFrameNum = 1;
const int mapFrameNum = 5;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;
bool newLaserOdometry = false;

//先验地图的坐标边界
float maxx = -999999.0, minx = 999999.0;
float maxy = -999999.0, miny = 999999.0;
float maxz = -999999.0, minz = 999999.0;

int laserCloudWidth = 0;    //栅格个数
int laserCloudHeight = 0;
int laserCloudDepth = 0;
int laserCloudNum = 0;
int test;
const float voxel_range = 30.0;  //栅格大小

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

pcl::PointCloud<pcl::PointXYZI>::Ptr priorSurfFeature(new pcl::PointCloud<pcl::PointXYZI>());//先验地图平面点  //edit all normal
pcl::PointCloud<pcl::PointXYZI>::Ptr priorCornerFeature(new pcl::PointCloud<pcl::PointXYZI>());//先验地图边缘点

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerStack2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfStack2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudOri(new pcl::PointCloud<pcl::PointXYZI>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSel(new pcl::PointCloud<pcl::PointXYZI>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCorr(new pcl::PointCloud<pcl::PointXYZI>());
//pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudProj(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr* laserCloudCornerArray;
pcl::PointCloud<pcl::PointXYZI>::Ptr* laserCloudSurfArray;
pcl::PointCloud<pcl::PointXYZI>::Ptr* laserCloudCornerArray2;
pcl::PointCloud<pcl::PointXYZI>::Ptr* laserCloudSurfArray2;

pcl::PointCloud<pcl::PointXYZI>::Ptr LoadedCloud(new pcl::PointCloud<pcl::PointXYZI>());
sensor_msgs::PointCloud2 LoadCloudMsg;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<pcl::PointXYZI>());



float transformSum[6] = {0};
float transformIncre[6] = {0};
float transformTobeMapped[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};



string surf_bfile = "/home/robot/e_ws/data/map/t1_surf.map";//平面点文件名
string corner_bfile = "/home/robot/e_ws/data/map/t1_corner.map";//边缘点文件名

struct VMapPoint{
    float x;
    float y;
    float z;
};

bool get_prior_map()
{
	// =======  获取先验map文件中点的个数  =======
	ifstream fin_surf;
	fin_surf.open(surf_bfile.c_str(), ios::in | ios::binary);
	if(fin_surf.is_open()){
		ROS_INFO("load success!");
	}
	if(!fin_surf.is_open())
        return false;
	fin_surf.seekg(0, std::ios::end);
	int filesize_surf = fin_surf.tellg();
	fin_surf.seekg(0, std::ios::beg);
	fin_surf.close();
	
	ifstream fin_corner;
	fin_corner.open(corner_bfile.c_str(), ios::in | ios::binary);
	if(!fin_corner.is_open())
        return false;
	fin_corner.seekg(0, std::ios::end);
	int filesize_corner = fin_corner.tellg();
	fin_corner.seekg(0, std::ios::beg);
	fin_corner.close();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr fullMapPoints_surf(new pcl::PointCloud<pcl::PointXYZ>());
	fullMapPoints_surf->reserve(filesize_surf/sizeof(VMapPoint));  //预先申请好内存，提高点效率
	pcl::PointCloud<pcl::PointXYZ>::Ptr fullMapPoints_corner(new pcl::PointCloud<pcl::PointXYZ>());
	fullMapPoints_corner->reserve(filesize_corner/sizeof(VMapPoint));  //预先申请好内存，提高点效率
	
	
	// ======   读取，放入到数组fullMapPoints中, 并整理出坐标边界  =================
	VMapPoint tmp;
	pcl::PointXYZ tmpnode;
	
	ifstream fin_surf1;
	fin_surf1.open(surf_bfile.c_str(), ios::in | ios::binary);
	if(!fin_surf1.is_open())
        return false;
	while(fin_surf1.read((char*)&tmp, sizeof(VMapPoint)))
	{
		tmpnode.x = tmp.x;    tmpnode.y = tmp.y;    tmpnode.z = tmp.z;
		fullMapPoints_surf->push_back(tmpnode);
		if(tmpnode.x > maxx) maxx = tmpnode.x;			if(tmpnode.x < minx) minx = tmpnode.x;
		if(tmpnode.y > maxy) maxy = tmpnode.y;			if(tmpnode.y < miny) miny = tmpnode.y;
		if(tmpnode.z > maxz) maxz = tmpnode.z;			if(tmpnode.z < minz) minz = tmpnode.z;
	}
    fin_surf1.close();
	
	ifstream fin_corner1;
	fin_corner1.open(corner_bfile.c_str(), ios::in | ios::binary);
	if(!fin_corner1.is_open())
        return false;
	while(fin_corner1.read((char*)&tmp, sizeof(VMapPoint)))
	{
		tmpnode.x = tmp.x;    tmpnode.y = tmp.y;    tmpnode.z = tmp.z;
		fullMapPoints_corner->push_back(tmpnode);
		if(tmpnode.x > maxx) maxx = tmpnode.x;			if(tmpnode.x < minx) minx = tmpnode.x;
		if(tmpnode.y > maxy) maxy = tmpnode.y;			if(tmpnode.y < miny) miny = tmpnode.y;
		if(tmpnode.z > maxz) maxz = tmpnode.z;			if(tmpnode.z < minz) minz = tmpnode.z;
	}
	fin_corner1.close();
	
	
	// ======   整理出空间栅格数组  =================
	laserCloudWidth = int((maxx - minx + voxel_range/2.0) / voxel_range) + 1;
	laserCloudHeight = int((maxy - miny + voxel_range/2.0) / voxel_range) + 1;
	laserCloudDepth = int((maxz - minz + voxel_range/2.0) / voxel_range) + 1;
	laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
	
	laserCloudSurfArray = new pcl::PointCloud<pcl::PointXYZI>::Ptr[laserCloudNum] ;  //edit all normal
	laserCloudCornerArray = new pcl::PointCloud<pcl::PointXYZI>::Ptr[laserCloudNum] ;
	laserCloudSurfArray2 = new pcl::PointCloud<pcl::PointXYZI>::Ptr[laserCloudNum] ;
	laserCloudCornerArray2 = new pcl::PointCloud<pcl::PointXYZI>::Ptr[laserCloudNum] ;
	for (int i = 0; i < laserCloudNum; i++) 
	{
		laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
		laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
		laserCloudCornerArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
		laserCloudSurfArray2[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
	}
	
	pcl::PointXYZI ptmp;  //edit normal
	int numMapPoints_surf = fullMapPoints_surf->points.size();
	for(int i=0; i<numMapPoints_surf; i++)
	{
		int cubeI = int((fullMapPoints_surf->points[i].x - minx + voxel_range/2.0) / voxel_range);
		int cubeJ = int((fullMapPoints_surf->points[i].y - miny + voxel_range/2.0) / voxel_range);
		int cubeK = int((fullMapPoints_surf->points[i].z - minz + voxel_range/2.0) / voxel_range);

		int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
		ptmp.x = fullMapPoints_surf->points[i].x;
		ptmp.y = fullMapPoints_surf->points[i].y;
		ptmp.z = fullMapPoints_surf->points[i].z;
		laserCloudSurfArray[cubeInd]->push_back(ptmp);
	}

	int numMapPoints_corner= fullMapPoints_corner->points.size();
	for(int i=0; i<numMapPoints_corner; i++)
	{
		int cubeI = int((fullMapPoints_corner->points[i].x - minx + voxel_range/2.0) / voxel_range);
		int cubeJ = int((fullMapPoints_corner->points[i].y - miny + voxel_range/2.0) / voxel_range);
		int cubeK = int((fullMapPoints_corner->points[i].z - minz + voxel_range/2.0) / voxel_range);

		int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
		ptmp.x = fullMapPoints_corner->points[i].x;
		ptmp.y = fullMapPoints_corner->points[i].y;
		ptmp.z = fullMapPoints_corner->points[i].z;
		laserCloudCornerArray[cubeInd]->push_back(ptmp);
	}
	
	
	// ======   降采样一下  =================
	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner;  //edit all normal
	downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
	downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

	for (int i = 0; i < laserCloudNum; i++)
	{
		laserCloudCornerArray2[i]->clear();
		downSizeFilterCorner.setInputCloud(laserCloudCornerArray[i]);
		downSizeFilterCorner.filter(*laserCloudCornerArray2[i]);

		laserCloudSurfArray2[i]->clear();
		downSizeFilterSurf.setInputCloud(laserCloudSurfArray[i]);
		downSizeFilterSurf.filter(*laserCloudSurfArray2[i]);

		laserCloudCornerArray[i] = laserCloudCornerArray2[i];
		laserCloudSurfArray[i] = laserCloudSurfArray2[i];
	}
	//======= 推送最初的cloud 降采样到最低 ==============
	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterLoaded;
	pcl::PointCloud<pcl::PointXYZI>::Ptr unuseLoadedCloud(new pcl::PointCloud<pcl::PointXYZI>());
	LoadedCloud->clear();
	downSizeFilterLoaded.setLeafSize(0.4, 0.4, 0.4);
	for(int i = 0; i < laserCloudNum; i++){
		*LoadedCloud += *laserCloudCornerArray2[i];
		*LoadedCloud += *laserCloudSurfArray2[i];
	}
	cout << "LOADED " << LoadedCloud->size() << " POINTS" << endl;
	// downSizeFilterLoaded.setInputCloud(LoadedCloud);
	// downSizeFilterLoaded.filter(*unuseLoadedCloud);

	//pcl::toROSMsg(*unuseLoadedCloud, LoadCloudMsg);
	pcl::toROSMsg(*LoadedCloud, LoadCloudMsg);
	LoadCloudMsg.header.frame_id = "camera";

	return true;
}

//服务回调函数，开始relocalization
bool relocalizationHandler(slam_algorithm::srv_relocalization::Request &req,
				     slam_algorithm::srv_relocalization::Response &res)  //edit
{
	surf_bfile.clear();
    corner_bfile.clear();
       
    if("error" == req.str_map_name){
        res.feedback = "Priormap File  Error!"; 
        return false;
    }
     
    surf_bfile = "/home/robot/e_ws/data/map/" + req.str_map_name + "_surf.map";
    corner_bfile = "/home/robot/e_ws/data/map/" + req.str_map_name + "_corner.map";

    // surf_bfile = "/home/robot/e_ws/data/map/t1_surf.map";
    // corner_bfile = "/home/robot/e_ws/data/map/t1_corner.map";
    
    if(get_prior_map())
	{
		ROS_INFO("load map success!");
		res.feedback = "get priormap OK!";
		return true;
    }
	else
	{
		ROS_ERROR("load map failed!");
		res.feedback = "get priormap failed!";
		return false;
	}
}


void transformAssociateToMap()
{
   float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
            - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
   float y1 = transformBefMapped[4] - transformSum[4];
   float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3])
            + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

   float x2 = x1;
   float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
   float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

   transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
   transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
   transformIncre[5] = z2;

   float sbcx = sin(transformSum[0]);
   float cbcx = cos(transformSum[0]);
   float sbcy = sin(transformSum[1]);
   float cbcy = cos(transformSum[1]);
   float sbcz = sin(transformSum[2]);
   float cbcz = cos(transformSum[2]);

   float sblx = sin(transformBefMapped[0]);
   float cblx = cos(transformBefMapped[0]);
   float sbly = sin(transformBefMapped[1]);
   float cbly = cos(transformBefMapped[1]);
   float sblz = sin(transformBefMapped[2]);
   float cblz = cos(transformBefMapped[2]);

   float salx = sin(transformAftMapped[0]);
   float calx = cos(transformAftMapped[0]);
   float saly = sin(transformAftMapped[1]);
   float caly = cos(transformAftMapped[1]);
   float salz = sin(transformAftMapped[2]);
   float calz = cos(transformAftMapped[2]);

   float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly)
             - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
             - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
             - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
             - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
   transformTobeMapped[0] = -asin(srx);

   float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
   float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz)
                - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz)
                - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly)
                - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx)
                + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
   transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                  crycrx / cos(transformTobeMapped[0]));

   float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz)
                - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx)
                - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly)
                + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx)
                - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz
                + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz)
                + calx*cblx*salz*sblz);
   float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly)
                - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx)
                + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly)
                + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly
                - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz)
                - calx*calz*cblx*sblz);
   transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                  crzcrx / cos(transformTobeMapped[0]));

   x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
   y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
   z1 = transformIncre[5];

   x2 = x1;
   y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
   z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

   transformTobeMapped[3] = transformAftMapped[3]
                          - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
   transformTobeMapped[4] = transformAftMapped[4] - y2;
   transformTobeMapped[5] = transformAftMapped[5]
                          - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void transformUpdate()
{
   if (imuPointerLast >= 0) {
     float imuRollLast = 0, imuPitchLast = 0;
     while (imuPointerFront != imuPointerLast) {
       if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
         break;
       }
       imuPointerFront = (imuPointerFront + 1) % imuQueLength;
     }

     if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
       imuRollLast = imuRoll[imuPointerFront];
       imuPitchLast = imuPitch[imuPointerFront];
     } else {
       int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
       float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack])
                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
       float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod)
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

       imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
       imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
     }

     transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
     transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
   }

   //transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * transformSum[0];
   //transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * transformSum[2];

   for (int i = 0; i < 6; i++) {
     transformBefMapped[i] = transformSum[i];
     transformAftMapped[i] = transformTobeMapped[i];
   }
}

void pointAssociateToMap(pcl::PointXYZI *pi, pcl::PointXYZI *po)  //edit nromal
{
   float x1 = cos(transformTobeMapped[2]) * pi->x
            - sin(transformTobeMapped[2]) * pi->y;
   float y1 = sin(transformTobeMapped[2]) * pi->x
            + cos(transformTobeMapped[2]) * pi->y;
   float z1 = pi->z;

   float x2 = x1;
   float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
   float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

   po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
         + transformTobeMapped[3];
   po->y = y2 + transformTobeMapped[4];
   po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
         + transformTobeMapped[5];
   po->intensity = pi->intensity;
   //po->curvature = pi->curvature;  //edit
}

void pointAssociateTobeMapped(pcl::PointXYZI *pi, pcl::PointXYZI *po)  //edit normal
{
   float x1 = cos(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
            - sin(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);
   float y1 = pi->y - transformTobeMapped[4];
   float z1 = sin(transformTobeMapped[1]) * (pi->x - transformTobeMapped[3])
            + cos(transformTobeMapped[1]) * (pi->z - transformTobeMapped[5]);

   float x2 = x1;
   float y2 = cos(transformTobeMapped[0]) * y1 + sin(transformTobeMapped[0]) * z1;
   float z2 = -sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

   po->x = cos(transformTobeMapped[2]) * x2
         + sin(transformTobeMapped[2]) * y2;
   po->y = -sin(transformTobeMapped[2]) * x2
         + cos(transformTobeMapped[2]) * y2;
   po->z = z2;
   po->intensity = pi->intensity;
   //po->curvature = pi->curvature;  //edit
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerLast2)
{
   timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

   laserCloudCornerLast->clear();
   pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);
   
	/*std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*laserCloudCornerLast,*laserCloudCornerLast, indices);
	indices.clear();*/

   newLaserCloudCornerLast = true;
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurfLast2)
{
   timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

   laserCloudSurfLast->clear();
   pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);

   	/*std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*laserCloudSurfLast,*laserCloudSurfLast, indices);
	indices.clear();*/
   
   newLaserCloudSurfLast = true;
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
   timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

   laserCloudFullRes->clear();
   pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);

    /*std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*laserCloudFullRes, *laserCloudFullRes, indices);
	indices.clear();*/
   
   newLaserCloudFullRes = true;
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
   timeLaserOdometry = laserOdometry->header.stamp.toSec();

   double roll, pitch, yaw;
   geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

   transformSum[0] = -pitch;
   transformSum[1] = -yaw;
   transformSum[2] = roll;

   transformSum[3] = laserOdometry->pose.pose.position.x;
   transformSum[4] = laserOdometry->pose.pose.position.y;
   transformSum[5] = laserOdometry->pose.pose.position.z;

   newLaserOdometry = true;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
   double roll, pitch, yaw;
   tf::Quaternion orientation;
   tf::quaternionMsgToTF(imuIn->orientation, orientation);
   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

   imuPointerLast = (imuPointerLast + 1) % imuQueLength;

   imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
   imuRoll[imuPointerLast] = roll;
   imuPitch[imuPointerLast] = pitch;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "slam_algorithm");
	ros::NodeHandle nh;

	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>
											 ("/p2_corner", 2, laserCloudCornerLastHandler);

	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>
										   ("/p2_surf", 2, laserCloudSurfLastHandler);

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>
									  ("/p2_odom", 5, laserOdometryHandler);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
										  ("/p2_noise", 2, laserCloudFullResHandler);  //edit

	ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

	//ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>
	//									  ("/laser_cloud_surround", 1);

	ros::Publisher pubLoadedCloud = nh.advertise<sensor_msgs::PointCloud2>("/LoadedCloud", 10);//for verification

	ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
										 ("/final_cloud_registered", 2);  //edit

	ros::Publisher pubPose = nh.advertise<geometry_msgs::PoseStamped> ("/current_pose", 10);
	
	//ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc1", 2);

	//ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc2", 2);

	//ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc3", 2);

	//ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc4", 2);

	ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/final_aft_mapped_to_init", 5);

	ros::ServiceServer srv_relocalization = nh.advertiseService("srv_relocalization", relocalizationHandler);
	//这里直接读取地图，方便使用
	get_prior_map();

	nav_msgs::Odometry odomAftMapped;
	// odomAftMapped.header.frame_id = "slam_final";
	odomAftMapped.header.frame_id = "slam_final";
	odomAftMapped.child_frame_id = "aft_mapped";

	tf::TransformBroadcaster tfBroadcaster;
	tf::StampedTransform aftMappedTrans;
	aftMappedTrans.frame_id_ = "slam_final";
	aftMappedTrans.child_frame_id_ = "aft_mapped";

	std::vector<int> pointSearchInd;
	std::vector<float> pointSearchSqDis;

	pcl::PointXYZI pointOri, pointSel, pointProj, coeff;  //edit normal

	cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
	cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

	cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
	cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

	bool isDegenerate = false;
	cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterCorner;  //edit all normal
	downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
	downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterMap;
	downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);
	

	int frameCount = stackFrameNum - 1;
	int mapFrameCount = mapFrameNum - 1;
	ros::Rate rate(100);
	//bool status = ros::ok();
	while (ros::ok())
	{
		//ros::spinOnce();
		if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry &&
			fabs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
			fabs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
			fabs(timeLaserCloudFullRes - timeLaserOdometry) < 0.005) 
		{
			newLaserCloudCornerLast = false;
			newLaserCloudSurfLast = false;
			newLaserCloudFullRes = false;
			newLaserOdometry = false;

			frameCount++;
			if (frameCount >= stackFrameNum)
			{
				transformAssociateToMap();
				int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
				for (int i = 0; i < laserCloudCornerLastNum; i++) 
				{
					pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
					laserCloudCornerStack2->push_back(pointSel);
				}
				int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
				for (int i = 0; i < laserCloudSurfLastNum; i++)
				{
					pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
					laserCloudSurfStack2->push_back(pointSel);  //注意，和lego的mapping不一样，这里没加入outlier，但mapping中surf都包含outlier
				}
			}

			if (frameCount >= stackFrameNum)
			{
				frameCount = 0;

				//用来测试全局地图上的点，是否在当前视角内
				pcl::PointXYZI pointOnYAxis;  //edit normal
				pointOnYAxis.x = 0.0;
				pointOnYAxis.y = 10.0;  //
				pointOnYAxis.z = 0.0;
				pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

				//设备位于哪个栅格
				int centerCubeI = int((transformTobeMapped[3] - minx + voxel_range/2.0) / voxel_range);
				int centerCubeJ = int((transformTobeMapped[4] - miny + voxel_range/2.0) / voxel_range);
				int centerCubeK = int((transformTobeMapped[5] - minz + voxel_range/2.0) / voxel_range);

				
				//下面从历史全局地图上，寻找当前位置周围2+1+2个栅格内的所有点
				int laserCloudValidNum = 0;
				int laserCloudSurroundNum = 0;
				float half_voxel_range = voxel_range / 2.0;
				for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
				{
					for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
					{
						for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
						{
							if (i >= 0 && i < laserCloudWidth &&
									j >= 0 && j < laserCloudHeight &&
									k >= 0 && k < laserCloudDepth)
							{
								float centerX = minx + voxel_range * i;
								float centerY = miny + voxel_range * j;
								float centerZ = minz + voxel_range * k;

								bool isInLaserFOV = false;
								//多向外扩展半个栅格来检验
								for (int ii = -1; ii <= 1; ii += 2)
								{
									for (int jj = -1; jj <= 1; jj += 2)
									{
										for (int kk = -1; kk <= 1; kk += 2)
										{
											float cornerX = centerX + half_voxel_range * ii;
											float cornerY = centerY + half_voxel_range * jj;
											float cornerZ = centerZ + half_voxel_range * kk;

											float squaredSide1 = (transformTobeMapped[3] - cornerX)
																		  * (transformTobeMapped[3] - cornerX)
																		  + (transformTobeMapped[4] - cornerY)
																		  * (transformTobeMapped[4] - cornerY)
																		  + (transformTobeMapped[5] - cornerZ)
																		  * (transformTobeMapped[5] - cornerZ);

											float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX)
															  + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
															  + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

											//保证所获取的点在当前视角的正负30度之间，之外的点用不到（a*a+b*b-c*c = 2*a*b*cos(theta)）
											float check1 = 100.0 + squaredSide1 - squaredSide2
														- 17.32 * sqrt(squaredSide1);

											float check2 = 100.0 + squaredSide1 - squaredSide2
														+ 17.32 * sqrt(squaredSide1);

											if (check1 < 0 && check2 > 0) {
												isInLaserFOV = true;
											}
										}
									}
								}

								if (isInLaserFOV) 
								{
									//有效点的index存下来
									laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
									laserCloudValidNum++;
								}
								laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
								laserCloudSurroundNum++;
							}
						}
					}
				}

				
				
				laserCloudCornerFromMap->clear();
				laserCloudSurfFromMap->clear();
				for (int i = 0; i < laserCloudValidNum; i++) {
					*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
					*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
				}
				int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
				int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

				
				//当前特征点降采样一下
				int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();
				for (int i = 0; i < laserCloudCornerStackNum2; i++) {
					pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
				}

				int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();
				for (int i = 0; i < laserCloudSurfStackNum2; i++) {
					pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
				}

				laserCloudCornerStack->clear();
				downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
				downSizeFilterCorner.filter(*laserCloudCornerStack);
				int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

				laserCloudSurfStack->clear();
				downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
				downSizeFilterSurf.filter(*laserCloudSurfStack);
				int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

				laserCloudCornerStack2->clear();
				laserCloudSurfStack2->clear();
				
				
				//下面真正开始求解
				if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100)
				{
					kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
					kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

					//开始迭代啦
					for (int iterCount = 0; iterCount < 30; iterCount++)
					{
						laserCloudOri->clear();
						//laserCloudSel->clear();
						//laserCloudCorr->clear();
						//laserCloudProj->clear();
						coeffSel->clear();

						for (int i = 0; i < laserCloudCornerStackNum; i++)
						{
							pointOri = laserCloudCornerStack->points[i];
							pointAssociateToMap(&pointOri, &pointSel);
							kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

							if (pointSearchSqDis[4] < 1.0)
							{
								float cx = 0;
								float cy = 0;
								float cz = 0;
								for (int j = 0; j < 5; j++)
								{
									cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
									cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
									cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
								}
								cx /= 5;
								cy /= 5;
								cz /= 5;

								float a11 = 0;
								float a12 = 0;
								float a13 = 0;
								float a22 = 0;
								float a23 = 0;
								float a33 = 0;
								for (int j = 0; j < 5; j++)
								{
									float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
									float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
									float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

									a11 += ax * ax;
									a12 += ax * ay;
									a13 += ax * az;
									a22 += ay * ay;
									a23 += ay * az;
									a33 += az * az;
								}
								a11 /= 5;
								a12 /= 5;
								a13 /= 5;
								a22 /= 5;
								a23 /= 5;
								a33 /= 5;
								//构造协方差矩阵
								matA1.at<float>(0, 0) = a11;
								matA1.at<float>(0, 1) = a12;
								matA1.at<float>(0, 2) = a13;
								matA1.at<float>(1, 0) = a12;
								matA1.at<float>(1, 1) = a22;
								matA1.at<float>(1, 2) = a23;
								matA1.at<float>(2, 0) = a13;
								matA1.at<float>(2, 1) = a23;
								matA1.at<float>(2, 2) = a33;
								//特征值和特征向量
								cv::eigen(matA1, matD1, matV1);

								if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
								{
									float x0 = pointSel.x;
									float y0 = pointSel.y;
									float z0 = pointSel.z;
									float x1 = cx + 0.1 * matV1.at<float>(0, 0);
									float y1 = cy + 0.1 * matV1.at<float>(0, 1);
									float z1 = cz + 0.1 * matV1.at<float>(0, 2);
									float x2 = cx - 0.1 * matV1.at<float>(0, 0);
									float y2 = cy - 0.1 * matV1.at<float>(0, 1);
									float z2 = cz - 0.1 * matV1.at<float>(0, 2);

									float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										  * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										  + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
										  * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
										  + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
										  * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

									float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

									float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

									float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
										- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

									float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
										+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

									float ld2 = a012 / l12;

									pointProj = pointSel;
									pointProj.x -= la * ld2;
									pointProj.y -= lb * ld2;
									pointProj.z -= lc * ld2;

									float s = 1 - 0.9 * fabs(ld2);

									coeff.x = s * la;
									coeff.y = s * lb;
									coeff.z = s * lc;
									coeff.intensity = s * ld2;

									if (s > 0.1)
									{
										laserCloudOri->push_back(pointOri);
										//laserCloudSel->push_back(pointSel);
										//laserCloudProj->push_back(pointProj);
										//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[0]]);
										//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[1]]);
										//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[2]]);
										//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[3]]);
										//laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[4]]);
										coeffSel->push_back(coeff);
									}
								}
							}
						}

						for (int i = 0; i < laserCloudSurfStackNum; i++)
						{
							pointOri = laserCloudSurfStack->points[i];
							pointAssociateToMap(&pointOri, &pointSel);
							kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

							if (pointSearchSqDis[4] < 1.0)
							{
								for (int j = 0; j < 5; j++)
								{
									matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
									matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
									matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
								}
								cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

								float pa = matX0.at<float>(0, 0);
								float pb = matX0.at<float>(1, 0);
								float pc = matX0.at<float>(2, 0);
								float pd = 1;

								float ps = sqrt(pa * pa + pb * pb + pc * pc);
								pa /= ps;
								pb /= ps;
								pc /= ps;
								pd /= ps;

								bool planeValid = true;
								for (int j = 0; j < 5; j++)
								{
									if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
											pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
											pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
									{
										planeValid = false;
										break;
									}
								}

								if (planeValid)
								{
									float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

									pointProj = pointSel;
									pointProj.x -= pa * pd2;
									pointProj.y -= pb * pd2;
									pointProj.z -= pc * pd2;

									float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
										   + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

									coeff.x = s * pa;
									coeff.y = s * pb;
									coeff.z = s * pc;
									coeff.intensity = s * pd2;

									if (s > 0.1)
									{
										laserCloudOri->push_back(pointOri);
										//laserCloudSel->push_back(pointSel);
										//laserCloudProj->push_back(pointProj);
										//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[0]]);
										//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[1]]);
										//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[2]]);
										//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[3]]);
										//laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[4]]);
										coeffSel->push_back(coeff);
									}
								}
							}
						}

						float srx = sin(transformTobeMapped[0]);
						float crx = cos(transformTobeMapped[0]);
						float sry = sin(transformTobeMapped[1]);
						float cry = cos(transformTobeMapped[1]);
						float srz = sin(transformTobeMapped[2]);
						float crz = cos(transformTobeMapped[2]);

						int laserCloudSelNum = laserCloudOri->points.size();
						if (laserCloudSelNum < 50) {
							continue;
						}

						cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
						cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
						cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
						cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
						cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
						cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
						for (int i = 0; i < laserCloudSelNum; i++)
						{
							pointOri = laserCloudOri->points[i];
							coeff = coeffSel->points[i];

							float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
									 + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
									 + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

							float ary = ((cry*srx*srz - crz*sry)*pointOri.x
									 + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
									 + ((-cry*crz - srx*sry*srz)*pointOri.x
									 + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

							float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
									 + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
									 + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

							matA.at<float>(i, 0) = arx;
							matA.at<float>(i, 1) = ary;
							matA.at<float>(i, 2) = arz;
							matA.at<float>(i, 3) = coeff.x;
							matA.at<float>(i, 4) = coeff.y;
							matA.at<float>(i, 5) = coeff.z;
							matB.at<float>(i, 0) = -coeff.intensity;
						}
						cv::transpose(matA, matAt);
						matAtA = matAt * matA;
						matAtB = matAt * matB;
						cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

						if (iterCount == 0)
						{
							cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
							cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
							cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

						   cv::eigen(matAtA, matE, matV);
						   matV.copyTo(matV2);

						   isDegenerate = false;
						   float eignThre[6] = {100, 100, 100, 100, 100, 100};
							for (int i = 5; i >= 0; i--)
							{
								if (matE.at<float>(0, i) < eignThre[i])
								{
									for (int j = 0; j < 6; j++) {
										matV2.at<float>(i, j) = 0;
									}
									isDegenerate = true;
								}
								else {
									break;
								}
							}
								matP = matV.inv() * matV2;
						}

						if (isDegenerate /*&& 0*/)
						{
							cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
							matX.copyTo(matX2);
							matX = matP * matX2;
							//ROS_INFO ("laser mapping degenerate");
						}

						transformTobeMapped[0] += matX.at<float>(0, 0);
						transformTobeMapped[1] += matX.at<float>(1, 0);
						transformTobeMapped[2] += matX.at<float>(2, 0);
						transformTobeMapped[3] += matX.at<float>(3, 0);
						transformTobeMapped[4] += matX.at<float>(4, 0);
						transformTobeMapped[5] += matX.at<float>(5, 0);

						float deltaR = sqrt(matX.at<float>(0, 0) * 180 / PI * matX.at<float>(0, 0) * 180 / PI
								  + matX.at<float>(1, 0) * 180 / PI * matX.at<float>(1, 0) * 180 / PI
								  + matX.at<float>(2, 0) * 180 / PI * matX.at<float>(2, 0) * 180 / PI);
						float deltaT = sqrt(matX.at<float>(3, 0) * 100 * matX.at<float>(3, 0) * 100
								  + matX.at<float>(4, 0) * 100 * matX.at<float>(4, 0) * 100
								  + matX.at<float>(5, 0) * 100 * matX.at<float>(5, 0) * 100);

						if (deltaR < 0.05 && deltaT < 0.05) {
							break;
						}

						//ROS_INFO ("iter: %d, deltaR: %f, deltaT: %f", iterCount, deltaR, deltaT);
					}//。。次迭代的结束

					transformUpdate();
				}//特征点数量的判断结束

           
		   

				//将当前位置提取出的一定范围内的历史map上的点发送出去
				/*mapFrameCount++;
				if (mapFrameCount >= mapFrameNum)
				{
					mapFrameCount = 0;

					laserCloudSurround2->clear();
					for (int i = 0; i < laserCloudSurroundNum; i++){
						int ind = laserCloudSurroundInd[i];
						*laserCloudSurround2 += *laserCloudCornerArray[ind];
						*laserCloudSurround2 += *laserCloudSurfArray[ind];
					}

					laserCloudSurround->clear();
					downSizeFilterCorner.setInputCloud(laserCloudSurround2);
					downSizeFilterCorner.filter(*laserCloudSurround);

					sensor_msgs::PointCloud2 laserCloudSurround3;
					pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
					laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
					laserCloudSurround3.header.frame_id = "camera_init";
					pubLaserCloudSurround.publish(laserCloudSurround3);
				}*/

				
				//点云转到全局，然后发送出去
				int laserCloudFullResNum = laserCloudFullRes->points.size();
				PointType point;
				float tmppp;
				for (int i = 0; i < laserCloudFullResNum; i++) {
					pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
					
					//坐标转换（省着在server里转了）：
					tmppp = laserCloudFullRes->points[i].x;
					laserCloudFullRes->points[i].x = laserCloudFullRes->points[i].z;
					laserCloudFullRes->points[i].z = laserCloudFullRes->points[i].y;
					laserCloudFullRes->points[i].y = tmppp;
				}
				
				laserCloudFullRes->is_dense = false;
				std::vector<int> indices;
				pcl::removeNaNFromPointCloud(*laserCloudFullRes, *laserCloudFullRes, indices);
				indices.clear();
	
				sensor_msgs::PointCloud2 laserCloudFullRes3;
				pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
				laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				laserCloudFullRes3.header.frame_id = "slam_final";
				pubLaserCloudFullRes.publish(laserCloudFullRes3);


				if(pubLoadedCloud.getNumSubscribers() != 0){
					LoadCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
					pubLoadedCloud.publish(LoadCloudMsg);
				}

				//位姿发送出去
				geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
									   (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

				/*odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
				odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
				odomAftMapped.pose.pose.orientation.z = geoQuat.x;
				odomAftMapped.pose.pose.orientation.w = geoQuat.w;
				odomAftMapped.pose.pose.position.x = transformAftMapped[3];
				odomAftMapped.pose.pose.position.y = transformAftMapped[4];
				odomAftMapped.pose.pose.position.z = transformAftMapped[5];
				odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
				odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
				odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
				odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
				odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
				odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
				pubOdomAftMapped.publish(odomAftMapped);*/
				
				
				//坐标转换（省着在server里转了）：
				odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				odomAftMapped.pose.pose.orientation.x = geoQuat.x;
				odomAftMapped.pose.pose.orientation.y = -geoQuat.y;
				odomAftMapped.pose.pose.orientation.z = -geoQuat.z;
				odomAftMapped.pose.pose.orientation.w = geoQuat.w;
				odomAftMapped.pose.pose.position.x = transformAftMapped[5];
				odomAftMapped.pose.pose.position.y = transformAftMapped[3];
				odomAftMapped.pose.pose.position.z = transformAftMapped[4];
				//odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
				//odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
				//odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
				//odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
				//odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
				//odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
				pubOdomAftMapped.publish(odomAftMapped);
				

				geometry_msgs::PoseStamped current_pose;
				current_pose.header.frame_id = "map";
				current_pose.header.stamp = ros::Time::now();
				current_pose.pose = odomAftMapped.pose.pose;
				pubPose.publish(current_pose);

				aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
				aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
				aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3],
												  transformAftMapped[4], transformAftMapped[5]));
				tfBroadcaster.sendTransform(aftMappedTrans);

				/*sensor_msgs::PointCloud2 pc12;
				pcl::toROSMsg(*laserCloudCornerStack, pc12);
				pc12.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				pc12.header.frame_id = "camera";
				pub1.publish(pc12);

				sensor_msgs::PointCloud2 pc22;
				pcl::toROSMsg(*laserCloudSurfStack, pc22);
				pc22.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				pc22.header.frame_id = "camera";
				pub2.publish(pc22);

				sensor_msgs::PointCloud2 pc32;
				pcl::toROSMsg(*laserCloudSel, pc32);
				pc32.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				pc32.header.frame_id = "camera";
				pub3.publish(pc32);

				sensor_msgs::PointCloud2 pc42;
				pcl::toROSMsg(*laserCloudProj, pc42);
				pc42.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				pc42.header.frame_id = "camera";
				pub4.publish(pc42);*/
			}//if(frameCount >= stackFrameNum)的结束
		}//新点进来和时间间隔满足条件：判断的括号结束

		ros::spinOnce();
		//status = ros::ok();
		rate.sleep();

	}//while循环结束

	return 0;
}              
