
#include <time.h>
#include <sys/time.h>
#include "utils.h"

#include "cv_util.h"
#include "medusa_log.h"

namespace PUBG {

    void calcu_nonzero_percent(Mat img, float &percent) {
        if (img.channels() > 1) {
            cout << "image channel must be 1,when count non zero" << endl;
            return;
        }
        int nonzero = cv::countNonZero(img);
        int h = img.rows;
        int w = img.cols;
        percent = 1.0 * nonzero / (h * w);
    }

    //直接调用这个函数就行了，返回值最好是int64_t，long long应该也可以
    int64_t getCurrentTime() {
        struct timeval tv;
        gettimeofday(&tv, NULL);    //该函数在sys/time.h头文件中
        return tv.tv_sec * 1000 + tv.tv_usec / 1000;
    }

    /*映射到垂直方向，统计单通道图像的非零个数*/
    void mapto_vertical(Mat img, vector<int> &v_map) {
        //容错检查
        if (img.channels() > 1) {
            cout << "statictic image pixel must be one channel" << endl;
            return;
        }

        //单通道投影到垂直平面
        int w = img.cols;
        int h = img.rows;
        for (int i = 0; i < h; i++) {
            int n = 0;
            for (int j = 0; j < w; j++) {
                if ((int) img.at<uchar>(i, j) > 0)//注意顺序不要弄反
                    n++;

            }
            v_map.push_back(n);
            // cout<<"map to vertical，nonezero num :"<<n<<endl;
        }
    }

    //投影到水平平面
    void mapto_horizon(Mat img, vector<int> &h_map) {
        //容错检查
        if (img.channels() > 1) {
            cout << "statictic image pixel must be one channel" << endl;
            return;
        }

        //单通道投影到水平面
        int w = img.cols;
        int h = img.rows;

        for (int i = 0; i < w; i++) {
            int n = 0;
            for (int j = 0; j < h; j++) {
                if ((int) img.at<uchar>(j, i) > 0)//注意顺序不要弄反
                    n++;

            }
            // cout<<n<<endl;
            h_map.push_back(n);
        }
    }

    //根据累加图，确定其起始索引
    void begin_end_idx(vector<int> vec, int &startup, int &idx1, int &idx2) {
        int len = vec.size();
        // idx1 = int(len*1.0/2) - 1;
        idx1 = startup - 1;
        idx2 = idx1 + 2;

        for (int i = idx1 - 1; i > 0; i--) {
            if (vec[i] > 10) {
                idx1 = i;
            } else
                break;
        }
        for (int i = idx2 + 1; i < len; i++) {
            if (vec[i] > 10) {
                idx2 = i;
            } else
                break;
        }
    }

    //根据累加图，确定其起始索引,不过可以自定义阈值
    void begin_end_idx_thrd(vector<int> vec, int &startup, int &idx1, int &idx2, int &thrd1,
                            int &thrd2) {
        int len = vec.size();
        // idx1 = int(len*1.0/2) - 1;
        idx1 = startup - 1;
        idx2 = idx1 + 2;

        for (int i = idx1 - 1; i > 0; i--) {
            if (vec[i] > thrd1) {
                idx1 = i;
            } else
                break;
        }
        for (int i = idx2 + 1; i < len; i++) {
            if (vec[i] > thrd2) {
                idx2 = i;
            } else
                break;
        }
    }

    void find_max(vector<int> &v, int &thrd, int &idx, int order) {
        //order =0 从小到大检索
        //order = 1 从大到小检索
        if (order == 0) {
            for (int i = 0; i < v.size(); i++) {
                if (v.at(i) > thrd) {
                    idx = i;
                    break;
                }
            }
        } else if (order == 1) {
            for (int i = v.size() - 1; i > 0; i--) {
                if (v.at(i) > thrd) {
                    idx = i;
                    break;
                }
            }

        }
    }

    //根据给定的阈值，是否匹配成功
    bool
    match_it(cv::Mat &template_mat, cv::Mat &target_mat, cv::Point &location, float match_thr) {
        if (template_mat.channels() != 3) {
            return false;
        }

        double minVal = -1;
        double maxVal = -1;
        Point minLoc;
        Point maxLoc;
        Mat rst;

        int type = template_mat.type(), depth = CV_MAT_DEPTH(type);
        if (!(depth == CV_8U || depth == CV_32F) && type == template_mat.type() &&
            template_mat.dims <= 2) {
            ALOGE_M("MatchTemplate::match_it matchTemplate error");
            return false;
        }

        if ((target_mat.size().width >= template_mat.size().width) &&
            (target_mat.size().height >= template_mat.size().height)) {
            matchTemplate(target_mat, template_mat, rst, TM_SQDIFF_NORMED);
            minMaxLoc(rst, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

            location = minLoc;
        } else {
            return false;
        }

//        ALOGV_M("match_it minVal:%lf maxVal:%lf match_thr:%f x:%d y:%d w:%d h:%d", minVal, maxVal,
//                match_thr, minLoc.x, minLoc.y, maxLoc.x,
//                maxLoc.y);

        return (minVal != -1) && (minVal < match_thr);
    }//match_it

    bool img_res(cv::Mat &template_mat, cv::Mat &target_mat, cv::Point &location, float res_thr) {
        if (target_mat.empty()) {
            return false;
        }

        Mat residual;
//        cv::subtract(this->tmpl, roi, residual);

        cv::absdiff(template_mat, target_mat, residual);

        // 判断残差是否成功
        Mat tmpl_gray, tmpl_bina;
        Mat residual_gray, residual_bina;
        cvtColor(template_mat, tmpl_gray, COLOR_BGR2GRAY);
        threshold(tmpl_gray, tmpl_bina, 190, 255, THRESH_BINARY);

        cvtColor(residual, residual_gray, COLOR_BGR2GRAY);
        threshold(residual_gray, residual_bina, 80, 255, THRESH_BINARY);

        int sum1 = countNonZero(tmpl_bina);
        int sum2 = countNonZero(residual_bina);

        double ratio = sum2 * 1.0 / sum1;
//        ALOGV_M("img_res ratio:%lf this->res_thr:%f", ratio, res_thr);
        return ratio < res_thr;
    }//img_res
}//PUBG
