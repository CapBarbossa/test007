#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
namespace PUBG {

    void color_coord(Mat &img, float &left, float &right, float &top, float &bottom, int &map_cnt,
                     int &frame_cnt);

    cv::Rect map_view(Mat &region_mat);

    void preprocess(Mat &frame, Mat &gray, Mat &edge, int &blk1, int &blk2, int &blk3, int &blk4,
                    int &target_height, int &target_width, int reset);

    void soft_has_map(Mat &edge, int &maybe_map);

    void
    m0_coord(Mat &frame, Mat &gray, Mat &edge, float &left, float &right, float &top, float &bottom,
             bool &m0_done);

    void m1_coord(Mat &frame, Mat &gray, Mat &edge, float &left_h, float &right_h, float &top_h,
                  float &bottom_h, float &left, float &right, float &top, float &bottom,
                  bool &m1_done);

    void
    coord_trans(int &blk1, int &blk2, int &blk3, int &blk4, int &target_width, int &target_height,
                int &x, int &y, int &width, int &height,
                int &x2, int &y2, int &w2, int &h2);

    void map_classify(Mat &tmp, vector<pair<float, int>> &top_results);

    void coord_log(int &a);

    void is_large(Mat &img, float &map_left, float &map_right, int &large);

    bool is_upload(Mat &img);

    void self_loc(Mat &map_view, Point &self_center, float &self_radius, int &has_orange);

    void orient1(Mat &map_view, Point &self_center, float &self_radius, float &self_atan);

    void
    orient(Mat &map_view, Point &self_center, float &self_radius, Mat &last_bina, Mat &this_bina,
           int &change_flag, float &self_atan);

    void
    orient3(Mat &map_view, Mat &last_bina, Mat &this_bina, int &change_flag, float &self_atan);//

    void obj_detect(Mat &map, vector<int> &kinds, vector<int> &thetas);

    void detect_black_frame(Mat &frame, int &blk1, int &blk2, int &blk3, int &blk4,
                            bool &black_flag);

    void shot_sight(Mat roi, int &min_len, int &max_len, int &yellow_shot, int reset);

    /**========================================以下是新接口========================================**/
    int pubg_shot_sight(cv::Mat &scene, int reset);

    bool pubg_in_game_icon(cv::Mat &scene);

    bool pubg_game_hall(cv::Mat &scene, Rect &out);
}
