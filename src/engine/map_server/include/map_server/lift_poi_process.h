#ifndef LIFT_POI_PROCESS_HPP_
#define LIFT_POI_PROCESS_HPP_

#include <set>
#include <queue>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "map_structure.h"

#include <Eigen/SVD>
#include <Eigen/Dense>  
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include "laser_geometry/laser_geometry.hpp"

#include "sps_common_msgs/msg/pixel_pose.hpp"

class LiftPoiProcess
{
private:
    static LiftPoiProcess* m_instance;
    static pthread_mutex_t mutex;

    int img_width_;
    int img_height_;

    cv::Mat map_img_;
    double resolution_;
    vecPixelPoses lift_pix_;
    std::vector<double> origin_;
    // just for test
    cv::Mat map_show_;
    // int group_id_;
    std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>> lift_wall_datas_;

public:
    static LiftPoiProcess* getInstance();

    // add by devin
    bool ProcessLiftPoi(const cv::Mat &grid_img, const std::vector<double> &origin, 
        const float resolution, std::unordered_map<std::string, LiftPoi>& src_lift_poi);

private:
    LiftPoiProcess();
    ~LiftPoiProcess();

    void SortLiftPoi(std::unordered_map<std::string, LiftPoi>& src_lift_poi);
    double CrossProduct(double x1, double y1, double x2, double y2);
    bool ComputeLiftPoints(const cv::Mat & img, 
        std::unordered_map<std::string, LiftPoi>& ordered_lift_poi,
        std::unordered_map<std::string, std::vector<cv::Point>> &lift_points);

    // bool getWorldPoint(float p_x, float p_y, geometry_msgs::Point32 &w_point);
    std::vector<cv::Point> getLiftWall(cv::Mat bin_img, cv::Point &p0, cv::Point &p1, 
                                       cv::Point &p2, float k, float b);
    std::vector<cv::Point> getAdjointPixels(cv::Mat &img, cv::Point center);
    // bool judgeTruncation(cv::Point p, float wall_thresh, float k, float b);
    bool judgeAdjointPixels(cv::Mat &img, cv::Point center, float k);
    void getAdjointPixels(std::queue<cv::Point> &candidate_points, 
                          std::vector<cv::Point> &visited_points, 
                          cv::Mat &img, cv::Point center);

    std::vector<cv::Point> fillWallPoints(cv::Mat img,
                            std::vector<sps_common_msgs::msg::PixelPose> &mark_points,
                            std::vector<std::pair<float, float>> wall_kbs,
                            std::vector<std::vector<cv::Point>> &effect_points);
    void fillSingleWallPoints(cv::Mat &img, std::vector<cv::Point> &points, 
                        cv::Point start, cv::Point end, float k, float b);

    std::vector<cv::Point> getLiftWall(cv::Mat bin_img, cv::Point center, cv::Point inner,
                                std::vector< std::pair<float, float> > walls_kb);

    void drawImg(std::vector<cv::Point> &points, cv::Mat &img, std::string img_name);
};

#endif