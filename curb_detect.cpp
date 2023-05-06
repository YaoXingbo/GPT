//***ouster激光和velodyne激光在rviz中的xy值相反，在这个程序中，针对ouster激光已经进行了坐标转换，x=-x，y=-y，而且left实际为right，right实际为left，很混乱，
#include <sensor_msgs/PointCloud2.h>
#include "ros/ros.h"
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/LaserScan.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>


#include <visualization_msgs/Marker.h>

using namespace std;



// 帮助对点云的排序。
// 升序。（针对y大于0的点）
bool comp_up(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.y < b.y;
}
// 降序。（针对y小于0的点）
bool comp_down(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.y > b.y;
}

sensor_msgs::LaserScan PointCloudToLaserscan(pcl::PointCloud<pcl::PointXYZ>& _pointcloud)
{
    float angle_min, angle_max, range_min, range_max, angle_increment;
    
    //需要自行调整的参数
    angle_min = -3.14159;
    angle_max =  3.14159;
    range_min = 0.5;
    range_max = 20;
    //角度分辨率，分辨率越小，转换后的误差越小
    angle_increment = 0.005;

    //计算扫描点个数
    unsigned int beam_size = ceil((angle_max - angle_min) / angle_increment);

    sensor_msgs::LaserScan output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "os_lidar";
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    output.time_increment = 0.0;
    output.scan_time = 0.0;
    
    //先将所有数据用nan填充
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

    for (auto point : _pointcloud.points)
    {
        float range = hypot(point.x, point.y);
        float angle = atan2(point.y, point.x);
        int index = (int)((angle - output.angle_min) / output.angle_increment);
        if (index >= 0 && index < beam_size)
        {
        //如果当前内容为nan，则直接赋值
        if (isnan(output.ranges[index]))
        {
            output.ranges[index] = range;
        }
        //否则，只有距离小于当前值时，才可以重新赋值
        else
        {
            if (range < output.ranges[index])
            {
            output.ranges[index] = range;
            }
        }
        // output.intensities[index] = point.intensity;
        }
    }
    return output;
}


// 此类用于检测curb。输入点云，返回检测到的curb点组成的点云。主执行函数为detector。
class curbDetector
{
    public:
    curbDetector(){}

    pcl::PointCloud<pcl::PointXYZ> detector(const std::vector<pcl::PointCloud<pcl::PointXYZ> > input)
    {
        pc_in = input;
        pcl::PointCloud<pcl::PointXYZ> look_test;

        for (int i = 0; i < 10; i++)
        // 对于每一环进行处理、检测。由于之前我们这里取了64线lidar的10线，所以这里循环次数为10.
        {
            pcl::PointCloud<pcl::PointXYZ> pointsInTheRing = pc_in[i]; // 储存此线上的点。
            pcl::PointCloud<pcl::PointXYZ> pc_left; // 储存y大于0的点（左侧点）。
            pcl::PointCloud<pcl::PointXYZ> pc_right; // 储存y小于0的点（右侧点）。

            pcl::PointCloud<pcl::PointXYZ> cleaned_pc_left; //储存经过算法1处理过后的左侧点。
            pcl::PointCloud<pcl::PointXYZ> cleaned_pc_right; //储存经过算法1处理过后的右侧点。

            pcl::PointXYZ point; // 点的一个载体。
            size_t numOfPointsInTheRing = pointsInTheRing.size(); // 此线上的点的数量。

            for (int idx = 0; idx < numOfPointsInTheRing; idx++)
            // 分开左、右侧点，分别储存到对应的点云。
            {
                point = pointsInTheRing[idx];
                if (point.y >= 0)
                {pc_left.push_back(point);}
                else
                {pc_right.push_back(point);}
            }

            // 排序。（按绝对值升序）
            sort(pc_left.begin(), pc_left.end(), comp_up);
            sort(pc_right.begin(), pc_right.end(), comp_down);


            // 滑动检测curb点。（对应算法2）

            slideForGettingPoints(pc_left, true);
            slideForGettingPoints(pc_right, false);

        }
        for(int i = 0; i < curb_left.size(); i++)
        {
            curb_left[i].z = 0;
        }
        for(int i = 0; i < curb_left.size(); i++)
        {
            curb_left[i].z = 0;
        }




    return curb_left + curb_right;

    }

    //点水平距离
    double get_dist_xy(pcl::PointXYZ point1, pcl::PointXYZ point2)
    {
        return sqrt(((point1.y - point2.y) * (point1.y - point2.y)) + ((point1.x - point2.x) *(point1.x - point2.x)));
    }
    //点垂直距离
    double get_dist_z(pcl::PointXYZ point1, pcl::PointXYZ point2)
    {
        return (point2.z - point1.z);
    }

    //上升检测
    bool z_up(pcl::PointXYZ point1, pcl::PointXYZ point2)
    {
        if(point1.z < point2.z){return true;}
        else{return false;}
    }
    //平坦度检测
    bool flat_detect(pcl::PointXYZ point1, pcl::PointXYZ point2, pcl::PointXYZ point3)
    {
        float flat_thresh = 0.05;
        if(fabs(point1.z - point2.z) < flat_thresh && fabs(point1.z - point3.z) < flat_thresh && fabs(point2.z - point3.z) < flat_thresh)
            {
                return true;
            }
        else {return false;}

    }
 

    int slideForGettingPoints(pcl::PointCloud<pcl::PointXYZ> points, bool isLeftLine)
    {
        int w_0 = 10;
        //从一个点开始查找后续30个点
        int w_d = 30;
        int i = 0;

        // some important parameters influence the final performance.
        // float xy_thresh = 0.1;
        // float z_thresh = 0.06;

        //路演突变角点距离差值阈值
        float xy_thresh = 0.1;
        //路沿上升角点距离差值阈值
        // float z_thresh = 0.1;
        //检查高度范围阈值
        float z_thresh_max = 0.3;
        float z_thresh_min = 0.06;
        //路面平坦阈值
        float flat_thresh = 0.05;

        int points_num = points.size();

        //从头到尾开始查找30个点
        while((i + w_d) < points_num)
        {
            float z_max = points[i].z;
            float z_min = points[i].z;

            int idx_ = 0;
            float z_dis = 0;

            for (int i_ = 0; i_ < w_d; i_++)
            {
                float dis = fabs(points[i+i_].z - points[i+i_+1].z);
                //找到从起始点开始后续30个点中相邻点Z距离相差最大的
                if (dis > z_dis) {z_dis = dis; idx_ = i+i_;}
                //找出从起始点开始后续30个点的最高和最低点
                if (points[i+i_].z < z_min){z_min = points[i+i_].z;}
                if (points[i+i_].z > z_max){z_max = points[i+i_].z;}
            }
            //原版
            // if (fabs(z_max - z_min) >= z_thresh_min)
            // {
            //     for (int i_ = 0; i_ < (w_d - 1); i_++)
            //     {
            //         float p_dist = get_dist_xy(points[i + i_],points[i + 1 + i_]);
            //         if (p_dist >= xy_thresh)
            //         {
            //             if (isLeftLine) {curb_left.push_back(points[i_ + i]);return 0;}
            //             else {curb_right.push_back(points[i_ + i]);return 0;}
            //         }
            //     }
            //     // if (isLeftLine) {curb_left.push_back(points[idx_]);return 0;}
            //     // else {curb_right.push_back(points[idx_]);return 0;}
            // }

             //lmx修改 简洁3.0
            if (fabs(z_max - z_min) >= z_thresh_min)
            {
                int corner_index_l = -1;
                int corner_index_h = -1;

                for (int i_ = 0; i_ < (w_d - 1); i_++)
                {
                    float p_dist = get_dist_xy(points[i + i_],points[i + 1 + i_]);
                    //原论文找到cornor点方法，添加后续点云上升检测，以及前面平坦检测
                    if(p_dist >= xy_thresh 
                    && (z_up(points[i+i_],points[i+i_+1]) && z_up(points[i+i_],points[i+i_+2])) 
                    && (flat_detect(points[i+i_-1],points[i+i_-2],points[i+i_-3])) )
                    {
                        //初步找到路沿角起点******
                        corner_index_l = i_ + i;

                        for(int j = 2;j < 10; j++)
                        {
                            double dist_z = get_dist_z(points[corner_index_l],points[corner_index_l+j]);
                            if(flat_detect(points[corner_index_l+j],points[corner_index_l+j+1],points[corner_index_l+j+2]) 
                            && dist_z > z_thresh_min && dist_z < z_thresh_max
                            && points[corner_index_l+j].z < 0
                            && flat_detect(points[corner_index_l+j+3],points[corner_index_l+j+1],points[corner_index_l+j+2])  
                            )
                            {
                                //找到路沿角终点******
                                corner_index_h = corner_index_l+j;
                                if (isLeftLine) 
                                {
                                    for(int index = corner_index_l; index < corner_index_h; index++)
                                    {
                                        double angle = atan(points[index].x/-points[index].y) * 180 / M_PI;
                                        if(/*angle < 60 && angle > 50 || */fabs(points[index].y) < 0.5){}
                                        else{
                                            curb_left.push_back(points[index]);
                                        }
                                    }
                                    return 0;
                                }
                                else 
                                {
                                    for(int index = corner_index_l; index < corner_index_h; index++)
                                    {
                                        double angle = atan(points[index].x/-points[index].y) * 180 / M_PI;
                                        if(/*angle < 60 && angle > 50 || */fabs(points[index].y) < 0.5){}
                                        else{
                                            curb_right.push_back(points[index]);
                                        }
                                    }
                                    return 0;
                                }
                            }
                            
                        }
                    }
                }

            }


            //             //判断路沿角点后续是否点会上升2.5
            //             for(int j = corner_index_l; j < (w_d - 2); j++)
            //             {
            //                 //上升判断:一直上升，且不平坦
            //                 if(get_dist_xy(points[j],points[j+1]) > xy_thresh){}
            //                 //找到第一个平坦点或者下降点，判断后面点云是否平坦
            //                 else
            //                 {
            //                     //如果后面三个点都平坦，那么是路沿，否则break
            //                     if(fabs(points[j].z - points[j+2].z) < flat_thresh && fabs(points[j].z - points[j+3].z) < flat_thresh)
            //                     {
            //                         corner_index_h = j;
            //                         //防止检测到的两种角点是同一个点
            //                         if(corner_index_h - corner_index_l <= 10 && corner_index_h - corner_index_l >= 1)
            //                         {
            //                             if (isLeftLine) {curb_left.push_back(points[corner_index_l]);curb_left.push_back(points[corner_index_h]);return 0;}
            //                             else {curb_right.push_back(points[corner_index_l]);curb_right.push_back(points[corner_index_h]);return 0;}
            //                         }
            //                         else{break;}
            //                     }else{break;}
            //                     break;                               
            //                 }
            //             }break;   
            //         }
            //     }
            // }

            //  //lmx修改 只考虑距离版本2.0
            // if (fabs(z_max - z_min) >= z_thresh_min)
            // {
            //     int corner_index_l = -1;
            //     int corner_index_h = -1;
            //     for (int i_ = 0; i_ < (w_d - 1); i_++)
            //     {
            //         float p_dist = get_dist_xy(points[i + i_],points[i + 1 + i_]);
            //         if(p_dist >= xy_thresh || (z_up(points[i+i_],points[i+i_+1]) && z_up(points[i+i_+1],points[i+i_+2]) && z_up(points[i+i_+2],points[i+i_+3]))  )
            //         {
            //             //初步找到路沿角点
            //             corner_index_l = i_ + i;
            //             //判断路沿角点后续是否点会上升
            //             for(int j = corner_index_l; j < (w_d - 2); j++)
            //             {
            //                 //上升判断:一直上升，且不平坦
            //                 if(get_dist_xy(points[j],points[j+1]) > xy_thresh){}
            //                 //找到第一个平坦点或者下降点，判断后面点云是否平坦
            //                 else
            //                 {
            //                     //如果后面三个点都平坦，那么是路沿，否则break
            //                     if(fabs(points[j].z - points[j+2].z) < flat_thresh && fabs(points[j].z - points[j+3].z) < flat_thresh)
            //                     {
            //                         corner_index_h = j;
            //                         //防止检测到的两种角点是同一个点
            //                         if(corner_index_h - corner_index_l <= 10 && corner_index_h - corner_index_l >= 1)
            //                         {
            //                             if (isLeftLine) {curb_left.push_back(points[corner_index_l]);curb_left.push_back(points[corner_index_h]);
            //                                                                     double angle = atan(points[corner_index_l].z / sqrt(points[corner_index_l].x * points[corner_index_l].x + points[corner_index_l].y * points[corner_index_l].y));
            //                                                                     cout << "scanID" << int(((angle * 180 / M_PI) + 11.25) * (31/22.5) + 0.5) << endl;return 0;
            //                             }
            //                             else {curb_right.push_back(points[corner_index_l]);curb_right.push_back(points[corner_index_h]);return 0;}
            //                         }
            //                         else{break;}
            //                     }else{break;}
            //                     break;                               
            //                 }
            //             }break;   
            //         }
            //     }
            // }


            // //lmx修改 平坦上升平坦版本1.0
            // if (fabs(z_max - z_min) >= z_thresh_min && fabs(z_max - z_min) <= z_thresh_max)
            // {
            //     int corner_index_l = -1;
            //     int corner_index_h = -1;
            //     for (int i_ = 0; i_ < (w_d - 1); i_++)
            //     {
            //         float p_dist = get_dist_xy(points[i + i_],points[i + 1 + i_]);
            //         if(p_dist >= xy_thresh)
            //         {
            //             //初步找到路沿角点
            //             corner_index_l = i_ + i;
            //             //判断路沿角点后续是否点会上升
            //             for(int j = corner_index_l; j < (w_d - 2); j++)
            //             {
            //                 //上升判断:一直上升，且不平坦
            //                 if(/*points[j+1].z >= points[j].z &&*/ fabs(points[j].z - points[j+1].z) > flat_thresh){}
            //                 //找到第一个平坦点或者下降点，判断后面点云是否平坦
            //                 else
            //                 {
            //                     //如果后面三个点都平坦，那么是路沿，否则break
            //                     if(fabs(points[j+1].z - points[j].z) < flat_thresh && fabs(points[j+2].z - points[j].z) < flat_thresh && fabs(points[j+3].z - points[j].z) < flat_thresh)
            //                     {
            //                         corner_index_h = j;
            //                         //防止检测到的两种角点是同一个点
            //                         if(corner_index_h - corner_index_l <= 10 && corner_index_h - corner_index_l >= 1)
            //                         {
            //                             if (isLeftLine) {curb_left.push_back(points[corner_index_l]);curb_left.push_back(points[corner_index_h]);return 0;}
            //                             else {curb_right.push_back(points[corner_index_l]);curb_right.push_back(points[corner_index_h]);return 0;}
            //                         }
            //                         else{break;}
            //                     }else{break;}
            //                     break;                               
            //                 }
            //             }break;   
            //         }
            //     }
            // }

            i += w_0;
        }
        return 0;
    }

    private:
    std::vector<pcl::PointCloud<pcl::PointXYZ> > pc_in;
    pcl::PointCloud<pcl::PointXYZ> curb_left;
    pcl::PointCloud<pcl::PointXYZ> curb_right;
};


class rosTransPoints
{
    public:
    rosTransPoints()
    {
        //设置感兴趣区域的参数。前四个参数设置了此感兴趣区域矩形的
        // 范围，而ring_idx_设置了我们取那几环的点云。
        x_range_up =20;
        x_range_down = 0;
        y_range_up = 8;
        y_range_down = -8;
        z_range_down = -0.6;

        // int ring_idx_[10] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
        int ring_idx_[10] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
        // int ring_idx_[10] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        memcpy(ring_idx, ring_idx_, sizeof(ring_idx_));

        // 定义接受和发布。
        pub = nh.advertise<sensor_msgs::PointCloud2>("/curb_detection_result", 1);
        pub_line = nh.advertise<sensor_msgs::PointCloud2>("/curb_line", 1);
        pub_curb_lasercan = nh.advertise<sensor_msgs::LaserScan>("/curb_lasercan", 1);
        // pub_curb_lasercan = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
        sub = nh.subscribe("/velodyne_points",3,&rosTransPoints::call_back,this);
        pub_view = nh.advertise<sensor_msgs::PointCloud2>("/inlier",1);


        
        // sub = nh.subscribe("/points_raw",3,&rosTransPoints::call_back,this);
    }

    void call_back(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
    {
        // 回调函数。每次获得点云都会自动执行的函数。
        clock_t startTime,endTime;
        startTime = ros::Time::now().toNSec();
        curbDetector cd;
        pcl::fromROSMsg(*input, cloud_in);


        // Here to add the code for processing the pc(pointcloud).
        cloud_cleaned = cleanPoints(cloud_in);
        // 执行检测。
        curb_point = cd.detector(cloud_cleaned);

        //ouster激光在这需要转换坐标系
        for(int i = 0; i < curb_point.points.size(); i++)
        {
            curb_point.points[i].x = -curb_point.points[i].x;
            curb_point.points[i].y = -curb_point.points[i].y;
        }


        

        pcl::PointCloud<pcl::PointXYZ> left_curb_points;
        pcl::PointCloud<pcl::PointXYZ> right_curb_points;
        for(int i = 0; i < curb_point.points.size(); i++)
        {
            pcl::PointXYZ temp;
            temp.x = curb_point.points[i].x;
            temp.y = curb_point.points[i].y;
            temp.z = 0;
            if(curb_point.points[i].y > 0)
            {
                left_curb_points.push_back(temp);
            }else{
                right_curb_points.push_back(temp);
            }
        }



        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);

        *cloud_left = left_curb_points;
        *cloud_right = right_curb_points;
        curb_line.clear();


        if(right_curb_points.size() > 15)
        {
            //创建一个模型参数对象，用于记录结果
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
            pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
            seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
            seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);  // Mandatory-设置目标几何形状
            seg.setMethodType(6);     //分割方法：随机采样法
            seg.setDistanceThreshold(0.05);         //设置误差容忍范围，也就是阈值
            seg.setMaxIterations(100);
            seg.setInputCloud(cloud_right);               //输入点云
            seg.setAxis(Eigen::Vector3f(1, 0, 0));
            seg.setEpsAngle(20 * M_PI/180.0);//设置角度误差
            seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量
            if(inliers->indices.size() > 8)
            {
                 cout << "拟合路沿" << endl;
                //直线方程
                double a = coefficients->values[0];
                double b = coefficients->values[1];
                double c = coefficients->values[2];
                double d = coefficients->values[3];
                double e = coefficients->values[4];
                double f = coefficients->values[5];

                for(double x = -5; x < 0; x+=0.2)
                {
                    pcl::PointXYZ curb_line_point;
                    double y = (x-a)*e/d+b;
                    curb_line_point.x = x;curb_line_point.y = y;curb_line_point.z = 0;
                    curb_line.push_back(curb_line_point);
                }
            }

        }
        if(left_curb_points.size() > 15)
        {
            //创建一个模型参数对象，用于记录结果
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers表示误差能容忍的点 记录的是点云的序号
            pcl::SACSegmentation<pcl::PointXYZ> seg;     // 创建一个分割器
            seg.setOptimizeCoefficients(true);      // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
            seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);  // Mandatory-设置目标几何形状
            seg.setMethodType(pcl::SAC_RANSAC);     //分割方法：随机采样法 
            seg.setDistanceThreshold(0.05);         //设置误差容忍范围，也就是阈值
            seg.setMaxIterations(100);
            seg.setInputCloud(cloud_left);               //输入点云
            seg.setAxis(Eigen::Vector3f(1, 0, 0));
            seg.setEpsAngle(20 * M_PI/180.0);//设置角度误差
            seg.segment(*inliers, *coefficients);   //分割点云，获得平面和法向量



            pcl::PointCloud<pcl::PointXYZ> view_points;
            for(int i = 0; i < inliers->indices.size(); i++)
            {
                view_points.points.push_back(cloud_left->points[inliers->indices[i]]);
            }
            pcl::toROSMsg(view_points, view_inlier);
            view_inlier.header.stamp = ros::Time::now();
            view_inlier.header.frame_id = "os_lidar";
            // cloud_final.header.frame_id = "velodyne";
            pub_view.publish (view_inlier);


            
            if(inliers->indices.size() > 8)
            {
                cout << "拟合路沿" << endl;
                //直线方程
                double a = coefficients->values[0];
                double b = coefficients->values[1];
                double c = coefficients->values[2];
                double d = coefficients->values[3];
                double e = coefficients->values[4];
                double f = coefficients->values[5];

                // cout << "a：" << coefficients->values[0] << endl;
                // cout << "b：" << coefficients->values[1] << endl;
                // cout << "c：" << coefficients->values[2] << endl;
                // cout << "d：" << coefficients->values[3] << endl;
                // cout << "e：" << coefficients->values[4] << endl;
                // cout << "f：" << coefficients->values[5] << endl;
                for(double x = -5; x < 0; x+=0.2)
                {
                    pcl::PointXYZ curb_line_point;
                    double y = (x-a)*e/d+b;
                    curb_line_point.x = x;curb_line_point.y = y;curb_line_point.z = 0;
                    curb_line.push_back(curb_line_point);
                }

            }

        }





        //输出检测到的路沿点
        pcl::toROSMsg(curb_point, cloud_final);
        cloud_final.header.stamp = ros::Time::now();
        cloud_final.header.frame_id = "os_lidar";
        // cloud_final.header.frame_id = "velodyne";
        pub.publish (cloud_final);


        
        curb_line.insert(curb_line.end(),curb_point.begin(),curb_point.end());
        for(int i = 0; i < curb_line.points.size(); i++)
        {
            curb_line.points[i].z = 0;
            // curb_line.points[i].x = -curb_line.points[i].x;
            // curb_line.points[i].y = -curb_line.points[i].y;
        }

        //输出路沿线laserscan
        // final_laserscan = PointCloudToLaserscan(curb_line);

        // final_laserscan.header.stamp = ros::Time::now();
        // final_laserscan.header.frame_id = "os_lidar";
        // pub_curb_lasercan.publish(final_laserscan);



        //输出路沿线pointcloud2
        pcl::toROSMsg(curb_line, curb_line_out);
        curb_line_out.header.stamp = ros::Time::now();
        curb_line_out.header.frame_id = "os_lidar";
        pub_line.publish (curb_line_out);

 
        endTime = ros::Time::now().toNSec();
        cout << "The run time is:" << (double)(endTime - startTime) / 10e6 << "ms" << endl;
    }

    int get_ring(pcl::PointXYZ p)
    {
        // 计算点所属于的环数。计算过程经过简化，
        // 具体原理可参考此代码：https://github.com/luhongquan66/loam_velodyne/blob/master/src/lib/MultiScanRegistration.cpp
        double angle;
        int scanID;
        angle = atan(p.z / sqrt(p.x * p.x + p.y * p.y));
        scanID = int(((angle * 180 / M_PI) + 22.5) * (31/45.0) + 0.5);
        // scanID = (int)(angle * 134.18714161056457 + 58.81598513011153);

        return scanID;
    }

    


    std::vector<pcl::PointCloud<pcl::PointXYZ> > cleanPoints(pcl::PointCloud<pcl::PointXYZ> pc)
    {
        // 函数作用：
        // 1. 去除Nan点
        // 2. 根据设定的感兴趣区域取点。（包括矩形区域和环数。）
        // 3. 将取得的不同环数的点分别储存，返还。
        size_t cloudSize = pc.size();
        // size_t ringSize;
        pcl::PointXYZ point;
        int scanID_;
        // pcl::PointCloud<pcl::PointXYZ> _laserCloud;
        std::vector<pcl::PointCloud<pcl::PointXYZ> > laserCloudScans(10);

        for (int i = 0; i < cloudSize; i++)
        {
            point.x = pc[i].x;
            point.y = pc[i].y;
            point.z = pc[i].z;
            // cout << point << endl;

            if (!pcl_isfinite(point.x) || 
            !pcl_isfinite(point.y) || 
            !pcl_isfinite(point.z))
            {continue;}
            if ((point.x < x_range_down) || (point.x > x_range_up) || (point.y < y_range_down) || (point.y > y_range_up) || (point.z > z_range_down))
            {continue;}

            scanID_ = get_ring(point);
            // cout << scanID_ << endl;
            for (int ring_num = 0;ring_num < 10; ring_num++)
            {
                if (scanID_ == ring_idx[ring_num])
                {laserCloudScans[ring_num].push_back(point);}
            }
        }

        return laserCloudScans;
    }

    private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher pub_line;
    ros::Publisher pub_curb_lasercan;
    ros::Publisher pub_view;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointCloud<pcl::PointXYZ> curb_point;
    std::vector<pcl::PointCloud<pcl::PointXYZ> > cloud_cleaned;
    sensor_msgs::PointCloud2 cloud_final;
    sensor_msgs::PointCloud2 curb_line_out;
    pcl::PointCloud<pcl::PointXYZ> curb_line;
    sensor_msgs::LaserScan final_laserscan;
    sensor_msgs::PointCloud2 view_inlier;


    // parameters help to select the detection scope.
    int x_range_up;
    int x_range_down;
    int y_range_up;
    int y_range_down;
    float z_range_down;
    int ring_idx[10];

    // size_t maxpc = 0;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "curb_detector");
    rosTransPoints start_detec;
    ros::spin();
}
