#include "map_server/lift_poi_process.h"

#include <unistd.h> //access()函数fillfill
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

rclcpp::Logger LIFTP_LOG = rclcpp::get_logger("LiftPoiProcess");

LiftPoiProcess* LiftPoiProcess::m_instance = nullptr;
pthread_mutex_t LiftPoiProcess::mutex = PTHREAD_MUTEX_INITIALIZER;

LiftPoiProcess::LiftPoiProcess()
{
}

LiftPoiProcess::~LiftPoiProcess()
{
}

LiftPoiProcess* LiftPoiProcess::getInstance()
{
    pthread_mutex_lock(&mutex);
    if(nullptr == m_instance) {
        m_instance = new LiftPoiProcess();
    }
    pthread_mutex_unlock(&mutex);
    return m_instance;
}

bool LiftPoiProcess::ProcessLiftPoi(const cv::Mat &grid_img, const std::vector<double> &origin,
        const float resolution, std::unordered_map<std::string, LiftPoi>& src_lift_poi) {
    if (grid_img.empty() || origin.empty() || src_lift_poi.empty() || resolution <= 1e-4) {
        RCLCPP_ERROR(LIFTP_LOG, "ProcessLiftPoi failed to process lift poi");
        return false;
    }

    map_img_ = grid_img.clone();
    origin_ = origin;
    resolution_ = resolution;
    img_width_ = map_img_.cols;
    img_height_ = map_img_.rows;

    //process
    cv::Mat map_bin = cv::Mat::zeros(map_img_.size(), CV_8UC1);
    cv::Mat map_gray = cv::Mat::zeros(map_img_.size(), CV_8UC1);
    if(map_img_.channels() == 3)
        cv::cvtColor(map_img_, map_gray, cv::COLOR_BGR2GRAY);
    else
        map_gray = map_img_.clone();

    cv::threshold(map_gray, map_bin, 10, 255, cv::THRESH_BINARY);
    map_show_ = cv::Mat::zeros(map_img_.size(), CV_8UC3);
    cv::cvtColor(map_bin, map_show_, cv::COLOR_GRAY2RGB);

    SortLiftPoi(src_lift_poi);
    std::unordered_map<std::string, std::vector<cv::Point>> lift_points;
    ComputeLiftPoints(map_bin, src_lift_poi, lift_points);
    return true;
}

void LiftPoiProcess::SortLiftPoi(std::unordered_map<std::string, LiftPoi>& src_lift_poi) {
    for (auto& lift_poi_tmp : src_lift_poi) {
        auto& poi_tmp = lift_poi_tmp.second;
        //sort entry poi
        if (poi_tmp.pixel_entry_poi.size() == 2) {
            double vec_1_x = poi_tmp.pixel_entry_poi[0].x - poi_tmp.pixel_inner_poi.x;
            double vec_1_y = poi_tmp.pixel_entry_poi[0].y - poi_tmp.pixel_inner_poi.y;
            double vec_2_x = poi_tmp.pixel_entry_poi[1].x - poi_tmp.pixel_inner_poi.x;
            double vec_2_y = poi_tmp.pixel_entry_poi[1].y - poi_tmp.pixel_inner_poi.y;

            if (CrossProduct(vec_1_x, vec_1_y, vec_2_x, vec_2_y) > 0) {
                //pt1在右侧， pt2在左侧，需要调换位置
                auto swap_tmp = poi_tmp.pixel_entry_poi[0];
                poi_tmp.pixel_entry_poi[0] = poi_tmp.pixel_entry_poi[1];
                poi_tmp.pixel_entry_poi[1] = swap_tmp;
            }
        }
        // sort corner poi
        if (poi_tmp.pixel_corner_poi.size() == 4) {
            // 根据角点到电梯外点的距离排序
            std::vector<int> sort_idx;
            std::vector<std::pair<int, float>> vec_idx_dists;

            for(int idx = 0; idx < 4; ++idx) {
                float dist = sqrt(pow(poi_tmp.pixel_corner_poi[idx].x - poi_tmp.pixel_entry_poi[0].x, 2)
                                + pow(poi_tmp.pixel_corner_poi[idx].y - poi_tmp.pixel_entry_poi[0].y, 2));
                vec_idx_dists.push_back(std::make_pair(idx, dist));
            }

            // 根据电梯角点到外点的距离进行排序(升序：从小到大)
            sort(vec_idx_dists.begin(), vec_idx_dists.end(),
                [](std::pair<int, float> a, std::pair<int, float> b){
                    return a.second < b.second;
                }
            );

            //获取最近的两个点(即门上的两个点),并排序
            int id_a = vec_idx_dists[0].first;
            int id_b = vec_idx_dists[1].first;
            auto door_a = poi_tmp.pixel_corner_poi[id_a];
            auto door_b = poi_tmp.pixel_corner_poi[id_b];
            double vec_a_x = door_a.x - poi_tmp.pixel_inner_poi.x;
            double vec_a_y = door_a.y - poi_tmp.pixel_inner_poi.y;
            double vec_b_x = door_b.x - poi_tmp.pixel_inner_poi.x;
            double vec_b_y = door_b.y - poi_tmp.pixel_inner_poi.y;

            if (CrossProduct(vec_a_x, vec_a_y, vec_b_x, vec_b_y) > 0.0) {
                sort_idx.push_back(id_b);
                sort_idx.push_back(id_a);
            }
            else {
                sort_idx.push_back(id_a);
                sort_idx.push_back(id_b);
            }
            //获取最远的两个点(后墙上的角点),并排序
            int id_c = vec_idx_dists[2].first;
            int id_d = vec_idx_dists[3].first;
            auto door_c = poi_tmp.pixel_corner_poi[id_c];
            auto door_d = poi_tmp.pixel_corner_poi[id_d];
            double vec_c_x = door_c.x - poi_tmp.pixel_inner_poi.x;
            double vec_c_y = door_c.y - poi_tmp.pixel_inner_poi.y;
            double vec_d_x = door_d.x - poi_tmp.pixel_inner_poi.x;
            double vec_d_y = door_d.y - poi_tmp.pixel_inner_poi.y;

            if (CrossProduct(vec_c_x, vec_c_y, vec_d_x, vec_d_y) > 0.0) {
                sort_idx.push_back(id_d);
                sort_idx.push_back(id_c);
            }
            else {
                sort_idx.push_back(id_c);
                sort_idx.push_back(id_d);
            }
            if (sort_idx.size() == 4) {
                std::vector<sps_common_msgs::msg::PixelPose> sorted_pixel_corner_poi;
                poi_tmp.pixel_door_poi.push_back(poi_tmp.pixel_corner_poi[sort_idx[0]]);
                poi_tmp.pixel_door_poi.push_back(poi_tmp.pixel_corner_poi[sort_idx[1]]);
                sorted_pixel_corner_poi.push_back(poi_tmp.pixel_corner_poi[sort_idx[2]]);
                sorted_pixel_corner_poi.push_back(poi_tmp.pixel_corner_poi[sort_idx[3]]);
                poi_tmp.pixel_corner_poi = sorted_pixel_corner_poi;
            }
        }
    }
}

double LiftPoiProcess::CrossProduct(double x1, double y1, double x2, double y2) {
    return x1 * y2 - y1 * x2;
}


bool LiftPoiProcess::ComputeLiftPoints(const cv::Mat & img, 
    std::unordered_map<std::string, LiftPoi>& ordered_lift_poi,
    std::unordered_map<std::string, std::vector<cv::Point>> &lift_points) {
    
    for(auto& lift_pix : ordered_lift_poi){
        auto& group_id = lift_pix.first;
        auto& cur_lift_poi = lift_pix.second;
        // 电梯左侧角点逆时针方向
        // 左侧: [door0, corner3]; 右侧: [door1, corner2]; 中间: [corner2, corner3]
        cv::Point inner(cur_lift_poi.pixel_inner_poi.x, cur_lift_poi.pixel_inner_poi.y);
        cv::Point outer(cur_lift_poi.pixel_entry_poi[0].x, cur_lift_poi.pixel_entry_poi[0].y);
        cv::Point door0(cur_lift_poi.pixel_door_poi[0].x, cur_lift_poi.pixel_door_poi[0].y);
        cv::Point door1(cur_lift_poi.pixel_door_poi[1].x, cur_lift_poi.pixel_door_poi[1].y);
        cv::Point corner2(cur_lift_poi.pixel_corner_poi[0].x, cur_lift_poi.pixel_corner_poi[0].y);
        cv::Point corner3(cur_lift_poi.pixel_corner_poi[1].x, cur_lift_poi.pixel_corner_poi[1].y);

        float mid_theta_1 = atan2(door1.y - door0.y, door1.x - door0.x);
        float mid_theta_2 = atan2(corner2.y - corner2.y, corner2.x - corner3.x);
        mid_theta_1 = mid_theta_1 > 0 ? mid_theta_1 : mid_theta_1 + 2 * M_PI;
        mid_theta_2 = mid_theta_2 > 0 ? mid_theta_2 : mid_theta_2 + 2 * M_PI;
        float prior_k = (tan(mid_theta_1) + tan(mid_theta_2)) / 2;
        
        // 计算左侧电梯壁: [door0, corner3]
        cv::Vec4f left_line; 
        std::vector<cv::Point> left_wall_points;
        std::vector<cv::Point> left_effect_points;
        float left_k = prior_k;
        float left_b = inner.y - left_k * inner.x;
        left_wall_points = getLiftWall(img, inner, door0, corner3, left_k, left_b);
        cv::fitLine(left_wall_points, left_line, cv::DIST_L2, 0, 1e-2, 1e-2);
        left_k = left_line[1] / left_line[0];
        left_b = left_line[3] - left_k * left_line[2];        

        // 计算右侧电梯壁: [door1, corner2]
        cv::Vec4f right_line; 
        std::vector<cv::Point> right_wall_points;
        std::vector<cv::Point> right_effect_points;
        float right_k = prior_k;
        float right_b = inner.y - right_k * inner.x;
        right_wall_points = getLiftWall(img, inner, door1, corner2, right_k, right_b);
        cv::fitLine(right_wall_points, right_line, cv::DIST_L2, 0, 1e-2, 1e-2);
        right_k = right_line[1] / right_line[0];
        right_b = right_line[3] - right_k * right_line[2];       

        // 计算中间电梯壁: [corner2, corner3]
        cv::Vec4f mid_line; 
        std::vector<cv::Point> mid_wall_points;
        std::vector<cv::Point> mid_effect_points;    
        float mid_k = -1.0 / prior_k;
        float mid_b = inner.y - mid_k * inner.x;
        mid_wall_points = getLiftWall(img, inner, corner2, corner3, mid_k, mid_b);
        cv::fitLine(mid_wall_points, mid_line, cv::DIST_L2, 0, 1e-2, 1e-2);
        mid_k = mid_line[1] / mid_line[0];
        mid_b = mid_line[3] - mid_k * mid_line[2];


        // 计算左右侧电梯点
        float door_x = (door0.x + door1.x) / 2.0;
        float door_y = (door0.y + door1.y) / 2.0;

        float door_k = mid_k;
        float door_b = door_y - mid_k * door_x;
        // std::cout << door_k << " --------------- " << door_b << std::endl;
        // std::cout << left_k << " --------------- " << left_b << std::endl;
        // std::cout << right_k << " --------------- " << right_b << std::endl;

        // std::cout << -1.0 / prior_k << std::endl;

        int left_x = -1, left_y = -1;
        int right_x = -1, right_y = -1;
        left_x = (left_b - door_b) / (door_k - left_k);
        right_x = (right_b - door_b) / (door_k - right_k);

        if(std::abs(left_k) > std::abs(door_k)){
            left_y = door_k * left_x + door_b;
            right_y = door_k * right_x + door_b;
        }
        else{
            left_y = left_k * left_x + left_b;
            right_y = right_k * right_x + right_b;
        }

        sps_common_msgs::msg::PixelPose corner0, corner1;
        corner0.x = left_x;
        corner0.y = left_y;
        corner1.x = right_x;
        corner1.y = right_y;
        cur_lift_poi.pixel_corner_poi.insert(cur_lift_poi.pixel_corner_poi.begin(), corner1);
        cur_lift_poi.pixel_corner_poi.insert(cur_lift_poi.pixel_corner_poi.begin(), corner0);

        std::pair<float, float> mid_kb(mid_k, mid_b);
        std::pair<float, float> door_kb(door_k, door_b);
        std::pair<float, float> left_kb(left_k, left_b);
        std::pair<float, float> right_kb(right_k, right_b);

        cv::Point left_point = left_wall_points.front();
        left_wall_points.clear();
        std::vector<std::pair<float, float>> left_kbs{left_kb, mid_kb, door_kb};
        left_wall_points = getLiftWall(img, left_point, inner, left_kbs);
        // cv::Mat draw_left = map_show_.clone();
        // drawImg(left_wall_points, draw_left, "left");

        cv::Point right_point = right_wall_points.front();
        right_wall_points.clear();
        std::vector<std::pair<float, float>> right_kbs{right_kb, mid_kb, door_kb};
        right_wall_points = getLiftWall(img, right_point, inner, right_kbs);
        // cv::Mat draw_right = map_show_.clone();
        // drawImg(right_wall_points, draw_right, "right");


        cv::Point mid_point = mid_wall_points.front();
        mid_wall_points.clear();
        std::vector<std::pair<float, float>> mid_kbs{mid_kb, left_kb, right_kb};
        mid_wall_points = getLiftWall(img, mid_point, inner, mid_kbs);
        // cv::Mat draw_mid = map_show_.clone();
        // drawImg(mid_wall_points, draw_mid, "mid");

        // 由于地图中电梯壁存在缺失, 对其进行补全
        std::vector<cv::Point> all_points;
        std::vector<std::pair<float, float>> wall_kbs;
        std::vector<std::vector<cv::Point>> wall_points;
        wall_kbs.emplace_back(std::make_pair(left_k, left_b));
        wall_kbs.emplace_back(std::make_pair(right_k, right_b));
        wall_kbs.emplace_back(std::make_pair(mid_k, mid_b));
        wall_points.emplace_back(left_wall_points);
        wall_points.emplace_back(right_wall_points);
        wall_points.emplace_back(mid_wall_points);
        all_points = fillWallPoints(img, cur_lift_poi.pixel_corner_poi, wall_kbs, wall_points);
        lift_points[group_id] = all_points;
        // cv::Mat draw_corner = map_show_.clone();
        // drawImg(all_points, draw_corner, "corner");        
    }
    return true;
}

std::vector<cv::Point> LiftPoiProcess::fillWallPoints(cv::Mat img,
                            std::vector<sps_common_msgs::msg::PixelPose> &mark_points,
                            std::vector<std::pair<float, float>> wall_kbs,
                            std::vector<std::vector<cv::Point>> &effect_points){
    std::vector<cv::Point> points;
    float left_k = wall_kbs[0].first;
    float left_b = wall_kbs[0].second;
    float right_k = wall_kbs[1].first;
    float right_b = wall_kbs[1].second;
    float mid_k = wall_kbs[2].first;
    float mid_b = wall_kbs[2].second;

    std::vector<cv::Point> &left_points = effect_points[0];
    std::vector<cv::Point> &right_points = effect_points[1];
    std::vector<cv::Point> &mid_points = effect_points[2];

    cv::Point left_start(mark_points[0].x, mark_points[0].y);
    cv::Point right_start(mark_points[1].x, mark_points[1].y);
    cv::Point right_end(mark_points[2].x, mark_points[2].y);
    cv::Point left_end(mark_points[3].x, mark_points[3].y);

    points.push_back(left_start);
    fillSingleWallPoints(img, left_points, left_start, left_end, left_k, left_b);
    fillSingleWallPoints(img, mid_points, left_end, right_end, mid_k, mid_b);
    fillSingleWallPoints(img, right_points, right_end, right_start, right_k, right_b);
    points.insert(points.end(), left_points.begin(), left_points.end());
    points.insert(points.end(), mid_points.begin(), mid_points.end());
    points.insert(points.end(), right_points.begin(), right_points.end());
    return points;
}

void LiftPoiProcess::fillSingleWallPoints(cv::Mat &img, std::vector<cv::Point> &points, 
        cv::Point start, cv::Point end, float k, float b){
    // 填充左测电梯壁点
    int inc_x = 0, inc_y = 0;
    if(abs(k) < 1.0)
        inc_x = end.x > start.x ? 1 : -1;
    else
        inc_y = end.y > start.y ? 1 : -1;

    std::unordered_set<int> points_x, points_y;
    for(auto &point : points){
        points_x.insert(point.x);
        points_y.insert(point.y);
    }


    int x = start.x;
    int y = start.y;
    // points.push_back(left_start);
    while(1){
        if(inc_x != 0){
            x += inc_x;
            y = k * x + b;
            if(points_x.count(x)){
                continue;
            }
            points_x.insert(x);
            points_y.insert(y);
        }
        else{
            y += inc_y;
            x = (y - b) / k;
            if(points_y.count(y)){
                continue;
            }
            points_x.insert(x);
            points_y.insert(y);
        }
        cv::Point point(x, y);
        points.push_back(point);

        if(x < 0 || x >= img_width_
            || y < 0 || y >= img_height_)
        {
            RCLCPP_ERROR(LIFTP_LOG, "fillSingleWallPoints (%d, %d), (%d, %d)", x, y, img_width_, img_height_);
            break;
        }
        if(inc_x == 1 && x >= end.x)  break;
        if(inc_x == -1 && x <= end.x) break;
        if(inc_y == 1 && y >= end.y)  break;
        if(inc_y == -1 && y <= end.y) break;
    }
}

std::vector<cv::Point> LiftPoiProcess::getLiftWall(cv::Mat bin_img,
    cv::Point center, cv::Point inner,
    std::vector< std::pair<float, float> > walls_kb) {
    // cv::Point inner(lift_pix[4].x, lift_pix[4].y);
    float theta = atan2(center.y - inner.y, center.x - inner.x);

    theta = theta > 0 ? theta : theta + 2 * M_PI;

    std::vector<cv::Point> wall_points;
    std::vector<cv::Point> visited_points;
    std::queue<cv::Point> candidate_points;

    candidate_points.push(center);
    visited_points.emplace_back(center);
    getAdjointPixels(candidate_points, visited_points, bin_img, center);

    float wall_thresh = 2;
    bool wall_fitting = false;
    while(!candidate_points.empty()){
        int size = candidate_points.size();
        
        for(int i = 0; i < size; i++){
            cv::Point candidate = candidate_points.front();
            candidate_points.pop();

            if(find(visited_points.begin(), visited_points.end(), 
                    candidate) != visited_points.end())
                continue;
            else
                visited_points.emplace_back(candidate);
                        
            bool res1 = judgeAdjointPixels(bin_img, candidate, theta);
            if(res1)
            {
                getAdjointPixels(candidate_points, visited_points, bin_img, candidate);

                float A1 = walls_kb[1].first * candidate.x - candidate.y + walls_kb[1].second;
                float B1 = sqrt(walls_kb[1].first * walls_kb[1].first + 1);
                if(std::abs(A1 / B1) < wall_thresh)
                    continue;

                bool res = center.y - (walls_kb[1].first * center.x + walls_kb[1].second) > 0;
                bool res1 = candidate.y - (walls_kb[1].first * candidate.x + walls_kb[1].second) > 0;
                // std::cout << "1: " << res << ", " << res1 << "; " << (res == res1) << std::endl;
                if(res != res1)
                    continue;
               
                float A2 = walls_kb[2].first * candidate.x - candidate.y + walls_kb[2].second;
                float B2 = sqrt(walls_kb[2].first * walls_kb[2].first + 1);
                if(std::abs(A2 / B2) < wall_thresh)
                    continue;
                res = center.y - (walls_kb[2].first * center.x + walls_kb[2].second) > 0;
                bool res2 = candidate.y - (walls_kb[2].first * candidate.x + walls_kb[2].second) > 0;
                // std::cout << "2: " << res << ", " << res2  << "; " << (res == res2) << std::endl;

                if(res != res2)
                    continue;

                wall_points.emplace_back(candidate);
            }
        }
    }
    
    return wall_points;
}

std::vector<cv::Point> LiftPoiProcess::getLiftWall(cv::Mat bin_img, 
                      cv::Point &p0, cv::Point &p1, cv::Point &p2, 
                      float k, float b){
    int inc_x = 0, inc_y = 0;
    if(abs(k) <= 1.0)
        inc_x = p1.x > p0.x ? 1 : -1;
    else
        inc_y = p1.y > p0.y ? 1 : -1;

    int center_x = p0.x;
    int center_y = p0.y;

    while(1){
        if(inc_x != 0){
            center_x += inc_x;
            center_y = k * center_x + b;
        }
        else{
            center_y += inc_y;
            center_x = (center_y - b) / k;
        }

        int val = (int)bin_img.at<uchar>(center_y, center_x);

        if(val != 0) continue;
        if(val == 0) break;
        
        if(center_x < 0 || center_x >= img_width_ 
            || center_y < 0 || center_y >= img_height_)
            break;
    }

    std::vector<cv::Point> wall_points;
    std::vector<cv::Point> visited_points;
    std::queue<cv::Point> candidate_points;
    cv::Point center = cv::Point(center_x, center_y);

    float wall_k = -1 / k;
    float wall_b = center_y - wall_k * center_x;
    
    candidate_points.push(center);
    visited_points.emplace_back(center);
    getAdjointPixels(candidate_points, visited_points, bin_img, center);

    float wall_thresh = 2;
    bool wall_fitting = false;
    while(!candidate_points.empty()){
        int size = candidate_points.size();
        
        for(int i = 0; i < size; i++){
            cv::Point candidate = candidate_points.front();
            candidate_points.pop();

            if(find(visited_points.begin(), visited_points.end(), 
                    candidate) != visited_points.end())
                continue;
            else
                visited_points.emplace_back(candidate);

            // if(wall_points.size() == 1.0 / resolution_){
            //     liftWallFitting(wall_points, wall_k, wall_b);  
            //     wall_fitting = true;
            // }
                        
            // bool res1 = judgeAdjointPixels(bin_img, candidate, theta);
            // if(res1)
            {
                getAdjointPixels(candidate_points, visited_points, bin_img, candidate);

                // if(wall_fitting){
                //     float A = wall_k * candidate.x - candidate.y + wall_b;
                //     float B = sqrt(wall_k * wall_k + 1);
                //     float dist = std::abs(A / B);
                //     if(std::abs(A / B) > wall_thresh)
                //         continue;
                // }

                float A = wall_k * candidate.x - candidate.y + wall_b;
                float B = sqrt(wall_k * wall_k + 1);
                float dist = std::abs(A / B);
                if(std::abs(A / B) > wall_thresh)
                    continue;


                wall_points.emplace_back(candidate);
            }
        }
    }

    // TODO

    cv::Mat draw_img = map_show_.clone();
    for(int i = 0; i < wall_points.size(); ++i){
        cv::circle(draw_img, cv::Point(wall_points[i]), 0, cv::Scalar(0, 0, 255), -1);
    }
    cv::circle(draw_img, center, 0, cv::Scalar(255, 0, 0), -1);

    // std::cout << wall_points.size() << std::endl;
    // cv::imwrite("/home/cloud/test.png", draw_img);

    return wall_points;
}

// 判断一个点是否为电梯壁点, a: 该点像素值为0; b: 有255的像素与其相连; c: 方向判断 
bool LiftPoiProcess::judgeAdjointPixels(cv::Mat &img, cv::Point center, float theta){
    
    std::vector<cv::Point> adjoint_pixels;
    adjoint_pixels = getAdjointPixels(img, center);

    std::vector<cv::Point> candidate_pixels;
    for(int i = 0; i < adjoint_pixels.size(); ++i){
        cv::Point adjoint = adjoint_pixels[i];
        if(adjoint.x == -1 || adjoint.y == -1)
            continue;        
        if(img.at<uchar>(adjoint.y, adjoint.x) != 0)
            candidate_pixels.emplace_back(adjoint);
    }

    if(candidate_pixels.empty())
        return false;

    for(int i = 0; i < candidate_pixels.size(); ++i){
        cv::Point candidate = candidate_pixels[i];
        float tmp = atan2(center.y  - candidate.y, 
                          center.x  - candidate.x);

        theta = theta > 0 ? theta : theta + 2 * M_PI;
        tmp = tmp > 0 ? tmp : tmp + 2 * M_PI;
        float delta_theta = abs(theta - tmp);
        delta_theta = delta_theta < M_PI ? delta_theta : 2 * M_PI - delta_theta; 
        if(delta_theta < M_PI / 2)
            return true;
    }

    return false;
}


void LiftPoiProcess::getAdjointPixels(std::queue<cv::Point> &candidate_points, 
    std::vector<cv::Point> &visited_points, cv::Mat &img, cv::Point center){
    
    std::vector<cv::Point> shifts {
        cv::Point(-1, -1), cv::Point(0, -1), cv::Point(1, -1),
        cv::Point(-1, 0), cv::Point(0, 0), cv::Point(1, 0),
        cv::Point(-1, 1), cv::Point(0, 1), cv::Point(1, 1)};
    
    for(int i = 0; i < shifts.size(); ++i){
        cv::Point shift = shifts[i];
        cv::Point adjoint = cv::Point(center.x + shift.x, center.y + shift.y);

        if(adjoint.x < 0 || adjoint.x >= img_width_
           || adjoint.y < 0 || adjoint.y >= img_height_)
            continue;

        if(find(visited_points.begin(), visited_points.end(), adjoint) == visited_points.end()){
            if(img.at<uchar>(adjoint.y, adjoint.x) == 0)
                candidate_points.push(adjoint);
        }
    }
}


// 获取像素临阈点: 3*3, 超出图像范围表示为[-1, -1]
std::vector<cv::Point> LiftPoiProcess::getAdjointPixels(cv::Mat &img, cv::Point center){
    std::vector<cv::Point> shifts {
        cv::Point(-1, -1), cv::Point(0, -1), cv::Point(1, -1),
        cv::Point(-1, 0), cv::Point(0, 0), cv::Point(1, 0),
        cv::Point(-1, 1), cv::Point(0, 1), cv::Point(1, 1)};
    
    std::vector<cv::Point> adjoint_pixels;

    for(int i = 0; i < shifts.size(); ++i){
        cv::Point shift = shifts[i];
        cv::Point adjoint = cv::Point(center.x + shift.x, 
                                      center.y + shift.y);

        if(adjoint.x < 0 || adjoint.x >= img_width_ 
           || adjoint.y < 0 || adjoint.y >= img_height_)
            adjoint = cv::Point(-1, -1);
        
        adjoint_pixels.emplace_back(adjoint);
    }
    return adjoint_pixels;
}

void LiftPoiProcess::drawImg(std::vector<cv::Point> &points, 
    cv::Mat &img, std::string img_name){

    if(!img.data) return;

    std::string save_path = "/home/cloud";
    cv::Mat draw_img = img.clone();
    if(img.channels() == 1)
        cv::cvtColor(draw_img, draw_img, cv::COLOR_GRAY2BGR);
    for(int i = 0; i < points.size(); ++i)
        cv::circle(draw_img, points[i], 0, cv::Scalar(0, 0, 255), -1);
   
    char name[256];
    snprintf(name, sizeof(name), "%s/%s.png", 
        save_path.c_str(), img_name.c_str()); 
    cv::imwrite(name, draw_img);
    cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
    // cv::imshow(name, draw_img);
    // cv::waitKey(0);
}
