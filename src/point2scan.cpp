#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <cmath>

ros::Publisher scan_pub;
bool if_use_original_point_intensity;
float z_max;
float z_min;
float range_min;
float range_max;
float resolution;
float angle_increment;
int num_angles;

// 将点云转换为二维激光扫描数据
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
    // 创建PCL对象用于存储点云数据
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);

    if (cloud->empty()) {
        ROS_WARN("Received empty point cloud");
        return;
    }

    // 创建PassThrough滤波器，过滤Z轴高度
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");            // 过滤Z轴
    pass.setFilterLimits(z_min, z_max);          // 设置Z轴范围为z_min到z_max
    pass.filter(*cloud_filtered);            // 应用过滤器

    // 将所有点的Z值映射为0
    for (auto& point : *cloud_filtered) {
        point.z = 0.0; // 将Z值设置为0
    }

    // 创建LaserScan消息
    sensor_msgs::LaserScan scan;
    scan.header = input->header; // 保持时间戳和框架一致
    scan.angle_min = -M_PI; // 以正前方为0度，左侧为负，右侧为正
    scan.angle_max = M_PI;
    scan.angle_increment = angle_increment; // 每 RESOLUTION 度一个增量
    scan.range_min = range_min; // 激光的最小有效距离
    scan.range_max = range_max; // 激光的最大有效距离

    // 初始化 ranges 和 intensities
    scan.ranges.resize(num_angles, scan.range_max); // 初始化范围，默认值为最大距离
    scan.intensities.resize(num_angles, 0.0);       // 初始化强度为 0

    // 遍历点云数据，计算每个点的角度和距离
    for (const auto& point : *cloud_filtered) {
        // 计算每个点的角度和距离
        float angle = atan2(point.y, point.x); // 计算点的极坐标角度（弧度）
        float range = hypot(point.x, point.y); // 计算点到原点的距离

        // 将角度转换为索引
        int angle_index = static_cast<int>((angle + M_PI) / scan.angle_increment);

        // 确保索引在有效范围内
        if (angle_index >= 0 && angle_index < num_angles) {
            // 只记录距离更近的点，并保存其强度
            if (range < scan.ranges[angle_index]) {
                scan.ranges[angle_index] = range;
                if (if_use_original_point_intensity) {
                    scan.intensities[angle_index] = point.intensity; 
                } else {
                    scan.intensities[angle_index] = 55; // 自定义强度值
                }
            }
        }
    }

    // 发布LaserScan数据
    ROS_INFO("LaserScan ranges length: %lu", scan.ranges.size());
    scan_pub.publish(scan);
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "point_cloud_to_laserscan");
    ros::NodeHandle nh;

    // 使用私有命名空间的 NodeHandle
	ros::NodeHandle private_node("~");

	// 从私有参数服务器中获取参数
	private_node.param("resolution", resolution, 0.5f); // 默认分辨率为0.5度
	private_node.param("z_min", z_min, 0.0f); // 默认Z轴最小值
	private_node.param("z_max", z_max, 1.0f); // 默认Z轴最大值
	private_node.param("range_min", range_min, 0.0f); // 默认最小距离
	private_node.param("range_max", range_max, 100.0f); // 默认最大距离
	private_node.param("if_use_original_point_intensity", if_use_original_point_intensity, false);


    // 计算角度数量和角度增量
    num_angles = static_cast<int>(360 / resolution); // 总角度数
    angle_increment = resolution * M_PI / 180.0; // 每度转换为弧度

    // 打印加载的参数
    ROS_INFO("Loaded parameters:");
    ROS_INFO("  resolution: %.2f", resolution);
    ROS_INFO("  z_min: %.2f", z_min);
    ROS_INFO("  z_max: %.2f", z_max);
    ROS_INFO("  range_min: %.2f", range_min);
    ROS_INFO("  range_max: %.2f", range_max);
    ROS_INFO("  if_use_original_point_intensity: %s", if_use_original_point_intensity ? "true" : "false");
    ROS_INFO("  num_angles: %d", num_angles);
    ROS_INFO("  angle_increment: %.5f rad", angle_increment);

    // 订阅/points_raw话题
    ros::Subscriber sub = nh.subscribe("/points_raw", 1, cloud_cb);

    // 创建发布者，发布到/scan话题
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);

    // 循环等待回调
    ros::spin();

    return 0;
}

