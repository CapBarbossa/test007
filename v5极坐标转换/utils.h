#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

namespace PUBG {

    void calcu_nonzero_percent(Mat img, float &percent);

    int64_t getCurrentTime();

    void mapto_vertical(Mat img, vector<int> &v_map);

    void mapto_horizon(Mat img, vector<int> &h_map);

// void begin_end_idx( vector<int> vec, int& idx1, int& idx2 );
    void
    begin_end_idx_thrd(vector<int> vec, int &startup, int &idx1, int &idx2, int &thrd1, int &thrd2);

    void find_max(vector<int> &v, int &thrd, int &idx, int order);

    /**
     *
     * @param templateMat
     * @param target_mat
     * @param location 模板坐标
     * @param match_thr 模板匹配阈值
     * @return
     */
    bool match_it(cv::Mat &template_mat, cv::Mat &target_mat, cv::Point &location, float match_thr);

    /**
     *
     * @param templateMat
     * @param target_mat
     * @param location 模板坐标
     * @param res_thr 残差阈值
     * @return
     */
    bool img_res(cv::Mat &template_mat, cv::Mat &target_mat, cv::Point &location, float res_thr);
}