
#include "../swt_seg/platform.h"
#include "entry.hpp"
#include "utils.h"

#include "cv_util.h"

using namespace PUBG;
// #include "classify/label_image_warpper.h"

#include "medusa_log.h"

namespace PUBG {

    enum MapType {
        NOT_SNOW, SNOW
    };

    static inline bool ContoursCmpl(vector<cv::Point> contour1, vector<cv::Point> contour2) {
        return (contour1.size() > contour2.size());
    }

    void detect_black_frame(Mat &frame, int &blk1, int &blk2, int &blk3, int &blk4,
                            bool &black_flag) {
        //blk1,  blk2,  blk3, blk4 对应x1 x2 y1 y2 的顺序

        //检测黑边，有的话去除
        Mat tmp_gray, tmp_binary;
        cvtColor(frame, tmp_gray, COLOR_BGR2GRAY);
        threshold(tmp_gray, tmp_binary, 10, 255, THRESH_BINARY);

        vector<int> h_map, v_map;
        mapto_horizon(tmp_binary, h_map);
        mapto_vertical(tmp_binary, v_map);
        int thrd = 10;
        int startup = int(h_map.size() * 1.0 / 2) - 1;
        begin_end_idx_thrd(h_map, startup, blk1, blk2, thrd, thrd);// x1 x2
        startup = int(v_map.size() * 1.0 / 2) - 1;
        begin_end_idx_thrd(v_map, startup, blk3, blk4, thrd, thrd); //y1 y2

        //合法性检查
        int minx = frame.cols * 1.0 / 8;
        int maxx = frame.cols * 1.0 / 8 * 7;
        int miny = frame.rows * 1.0 / 6;
        int maxy = frame.rows * 1.0 / 6 * 5;
        if (blk1 < minx && blk2 > maxx && blk3 < miny && blk4 > maxy) {
            black_flag = true;

        }
        ALOGV_M("detect_black_frame, %d, %d, %d, %d", blk1, blk2, blk3, blk4);
    }

    void preprocess(Mat &frame, Mat &gray, Mat &edge, int &blk1, int &blk2, int &blk3, int &blk4,
                    int &target_height, int &target_width, int reset) {
        bool show_it = false;
        // imshow( "frame_with_black", frame );

        //黑边检测
        static bool black_flag = false; //是否进行过边沿检测
        if (reset == 1) {
            black_flag = false;
        }

        if (!black_flag) {
//        black_flag = false;
            detect_black_frame(frame, blk1, blk2, blk3, blk4, black_flag);
        }

        if (blk1 < 0 || blk3 < 0) {
            return;
        }

        if (blk2 - blk1 <= 0) {
            return;
        }

        if (blk4 - blk3 <= 0) {
            return;
        }

        if (blk2 > frame.cols) {
            return;
        }

        if (blk4 > frame.rows) {
            return;
        }

        //获取非黑边的roi

        frame = frame(Rect(blk1, blk3, blk2 - blk1, blk4 - blk3));

        if (show_it) imshow("frame_without_black", frame);

        target_height = int(frame.rows * 1);
        target_width = int(frame.cols * 1);
        resize(frame, frame, Size(target_width, target_height));
        cvtColor(frame, gray, COLOR_BGR2GRAY);
//    int canny_low = 50, canny_hight = 150;
//    Canny( gray, edge, canny_low,canny_hight );

        if (show_it) {
            imshow("gray", gray);
            imshow("edge", edge);
        }
    }

//右上角小地图定位
    cv::Rect map_view(Mat &region_mat) {
        bool debug = false;
        bool save_img = false;

        //         imwrite("/sdcard/small_map.png",img);
        //分类地图下方三种不同颜色
        Mat mask = Mat::zeros(region_mat.size(), CV_8UC1);
        for (int r = 0; r < region_mat.rows; r++) {
            for (int c = 0; c < region_mat.cols; c++) {
                //blue
                if (region_mat.at<Vec3b>(r, c)[2] > 19 && region_mat.at<Vec3b>(r, c)[2] < 33 &&
                    region_mat.at<Vec3b>(r, c)[1] > 135 && region_mat.at<Vec3b>(r, c)[1] < 250 &&
                    region_mat.at<Vec3b>(r, c)[0] > 230 && region_mat.at<Vec3b>(r, c)[0] < 256) {
                    mask.at<uchar>(r, c) = 150;
                }
                //black
                if (region_mat.at<Vec3b>(r, c)[2] < 30 && region_mat.at<Vec3b>(r, c)[1] < 30 &&
                    region_mat.at<Vec3b>(r, c)[0] < 30) {
                    mask.at<uchar>(r, c) = 200;
                }
                //white
                if (region_mat.at<Vec3b>(r, c)[2] > 230 && region_mat.at<Vec3b>(r, c)[1] > 230 &&
                    region_mat.at<Vec3b>(r, c)[0] > 230) {
                    mask.at<uchar>(r, c) = 250;
                }
            }
        }
        if (debug) { imshow("mask", mask); }

//        imwrite("/sdcard/small_map_mask.png", mask);
        //small map在roi中的坐标
        int map_x1 = mask.cols;//min
        int map_x2 = 0;
        int map_y1 = 0;
        int map_y2 = 0;
        //一行同时出现三种颜色，才认为是地图底部
        for (int r = 0; r < mask.rows; r++) {
            bool blue = false;
            bool black = false;
            bool white = false;


            for (int c = 0; c < mask.cols; c++) {
                if ((int) mask.at<uchar>(r, c) == 150) {
                    blue = true;
                }
                if ((int) mask.at<uchar>(r, c) == 200) {
                    black = true;
                }
                if ((int) mask.at<uchar>(r, c) == 250) {
                    white = true;
                }
            }

            bool has_small_map = false;
            //sometimes the black slide will disapper
            if ((blue && black && white) || (blue && white)) {
                map_y2 = r;
                for (int c = 1; c < mask.cols - 1; c++) {
                    if ((int) mask.at<uchar>(r, c) == 150) {
                        map_x1 = min(c, map_x1);

                    }

                    if ((int) mask.at<uchar>(r, c) == 250) {
                        map_x2 = max(c, map_x2);
                    }
                }
            } else {
                bool tmp = false;
                has_small_map = has_small_map || tmp;
            }
        }

        //合法性检测 并修正
        int h_tmp = (map_y2 - map_y1);
        int w_tmp = (map_x2 - map_x1);
        if (h_tmp <= 0 || w_tmp <= 0) {
            return cv::Rect(0, 0, 0, 0);

        }
        if (w_tmp > 0) {
            if (h_tmp / (float) w_tmp > 0.8 && h_tmp / (float) w_tmp < 1.0) {
                map_x2 = map_x1 + h_tmp;
            } else if (h_tmp / (float) w_tmp >= 1.0 && h_tmp / (float) w_tmp < 1.2) {
                map_y2 = map_y1 + w_tmp;
            } else {
                return cv::Rect(0, 0, 0, 0);
            }
        }
        if (debug) {
            cout << map_x1 << " " << map_y1 << " " << map_x2 << " " << map_y2 << endl;
            rectangle(mask, Point(map_x1, map_y1), Point(map_x2, map_y2), Scalar(255));
            imshow("mask", mask);
        }

        return cv::Rect(map_x1, map_y1, map_x2 - map_x1, map_y2 - map_y1);
    }

    void
    coord_trans(int &blk1, int &blk2, int &blk3, int &blk4, int &target_width, int &target_height,
                int &x, int &y, int &width, int &height,
                int &x2, int &y2, int &w2, int &h2) {
        //坐标逆变换
        int x1, y1, w1, h1; //地图在去黑边的画面上坐标

        float coefx = 1.0 * (blk2 - blk1) / target_width;
        float coefy = 1.0 * (blk4 - blk3) / target_height;
        x1 = int(coefx * x);
        y1 = int(coefy * y);
        w1 = int(coefx * width);
        h1 = int(coefy * height);

        ALOGV_M("key blk2 - blk1):%d ", blk2 - blk1);
        ALOGV_M("key blk4 - blk3):%d ", blk4 - blk3);
        ALOGV_M("key target_width:%d ", target_width);
        ALOGV_M("key target_height:%d ", target_height);

        w2 = int(w1);
        h2 = int(h1);
        x2 = int(x1) + blk1;
        y2 = int(y1) + blk3;
        //LOGI( "key %d, %d, %d, %d",x2, y2, w2, h2 );
    }

    void coord_log(int &a) {
        ALOGV_M("key-log:%d", a);
    }

    void color_coord(Mat &img, float &left, float &right, float &top, float &bottom, int &map_cnt,
                     int &frame_cnt) {
        //对于PUBG地图，周边蓝色区域的BGR范围为
        //海岛地图 B:[70,92] G:[46,64], R:[14,39]
        // 1地图 B:[67,90] G:[43,63], R:[18,38]
        // 2地图 B:[77,98] G:[48,66], R:[0,23]

        //PUBG地图 上下边界不需要确定，仅仅需要确定左右边界
        bool show_it = false;
        //
        int min = 0;
        int max = 0;

        int tmp_left = 0;
        int tmp_right = 0;
        int tmp_top = 0;
        int tmp_bottom = 0;

        //B channel
        Mat mask;
        min = 67;
        max = 98;
        mask = Mat::zeros(img.rows, img.cols, CV_8UC1);
        for (int i = 0; i < img.rows; i++) {
            for (int j = 0; j < img.cols; j++) {
                if (img.at<Vec3b>(i, j)[0] > min && img.at<Vec3b>(i, j)[0] < max) {
                    mask.at<uchar>(i, j) = 255;
                }
            }
        }
        if (show_it) {
            imshow("B", mask);
        }

        //G channel
        Mat mask1;
        min = 43;
        max = 66;
        mask1 = Mat::zeros(img.rows, img.cols, CV_8UC1);
        for (int i = 0; i < img.rows; i++) {
            for (int j = 0; j < img.cols; j++) {
                if (img.at<Vec3b>(i, j)[1] > min && img.at<Vec3b>(i, j)[1] < max) {
                    mask1.at<uchar>(i, j) = 255;
                }
            }
        }
        if (show_it) {
            imshow("G", mask1);
        }

        //R channel
        Mat mask2;
        min = 0;
        max = 39;
        mask2 = Mat::zeros(img.rows, img.cols, CV_8UC1);
        for (int i = 0; i < img.rows; i++) {
            for (int j = 0; j < img.cols; j++) {
                if (img.at<Vec3b>(i, j)[2] > min && img.at<Vec3b>(i, j)[2] < max) {
                    mask2.at<uchar>(i, j) = 255;
                }
            }
        }
        if (show_it) imshow("R", mask2);

        //B and G and R channel
        Mat mask_BGR;
        multiply(mask, mask1, mask_BGR);
        multiply(mask_BGR, mask2, mask_BGR);
        if (show_it) imshow("mask_BGR", mask_BGR);

        //开运算比运算
        Mat element;
        int s = 5;
        element = getStructuringElement(MORPH_ELLIPSE, Size(s, s));
        morphologyEx(mask_BGR, mask_BGR, MORPH_CLOSE, element);
        if (show_it) imshow("open", mask_BGR);

        //查找边界
        vector<vector<Point>> contours;
        vector<Vec4i> hierarcy;
        findContours(mask_BGR, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        //后处理
        vector<Rect> boundRect(contours.size());  //定义外接矩形集合
        for (int i = 0; i < contours.size(); i++) {
            boundRect[i] = boundingRect(Mat(contours[i]));
        }

        for (int i = 0; i < boundRect.size(); i++) {
            if ((boundRect[i].width * 1.0 / mask_BGR.rows) < 0.8) {

            } else {
                tmp_left = boundRect[i].x;
                tmp_right = boundRect[i].x + boundRect[i].width;
                tmp_top = 0;
                tmp_bottom = mask_BGR.rows;
                //LOGI("key_corrd:%d,%d,%d,%d",tmp_left, tmp_right, tmp_top, tmp_bottom);

                //清理毛刺
                vector<int> h_map;
                mapto_horizon(mask_BGR, h_map);
                for (int i = left; i < right; i++) {
                    if (h_map[i] < 10) {
                        tmp_left = i;
                    } else {
                        break;
                    }
                }
            }
        }

        if (show_it) {
            rectangle(img, Point(left, 0), Point(right, bottom), Scalar(0, 255, 0), 2);
            imshow("map_location", img);
        }

        //合法性检测
        if (tmp_left > 0 && tmp_right > 0) {
            float ratio = (tmp_right - tmp_left) * 1.0 / (tmp_bottom - tmp_top);
//        LOGI("zhyj_ratio:%f",ratio);
            if (ratio > 1.05 || ratio < 0.95) { //一般情况下，在0.95-0.96之间
                tmp_left = 0;
                tmp_right = 0;
                tmp_top = 0;
                tmp_bottom = 0;
            } else {

                if ((tmp_left + tmp_right) / 2 < img.cols / 2) {
                    tmp_left = 0;
                    tmp_right = 0;
                    tmp_top = 0;
                    tmp_bottom = 0;
                } else {
                    map_cnt++;
//                    frame_cnt ++;

                    left = 1.0 * (map_cnt - 1) / map_cnt * left + 1.0 / map_cnt * tmp_left;
                    right = 1.0 * (map_cnt - 1) / map_cnt * right + 1.0 / map_cnt * tmp_right;
                    top = 1.0 * (map_cnt - 1) / map_cnt * top + 1.0 / map_cnt * tmp_top;
                    bottom = 1.0 * (map_cnt - 1) / map_cnt * bottom + 1.0 / map_cnt * tmp_bottom;

                }
            }
        }
    }

    void is_large(Mat &img, float &map_left, float &map_right, int &large) {
        if (map_left <= 0 || map_right <= 0 || map_right == map_left) {
            large = -1;
            return;

        }
        //过滤不是按钮的画面
        int botton_w = int((map_right - map_left) / 10.0);
        if (botton_w <= 0) {
            large = -1;
            return;
        }

        if (img.rows <= 0 || img.cols <= 0) {
            large = -1;
            return;
        }

        if ((int(map_right) + botton_w) > img.cols) {
            large = -1;
            return;
        }

        Mat botton = img(Rect(int(map_right), 0, botton_w, img.rows));
        // imshow("botton",botton);
        Mat botton_gray;
        cvtColor(botton, botton_gray, COLOR_BGR2GRAY);

        Mat botton_bina1;
        threshold(botton_gray, botton_bina1, 50, 255, THRESH_BINARY);
        //imshow( "botton_bina1" ,botton_bina1 );
        float white_per1;
        calcu_nonzero_percent(botton_bina1, white_per1);
        // cout<<white_per1<<endl;


        if (white_per1 > 0.5) {
            //大概率是游戏画面
            large = -1;
        } else {
            //可能是地图画面,含有按钮
            Mat botton_bina2;
            threshold(botton_gray, botton_bina2, 200, 255, THRESH_BINARY_INV);

            float white_per2;
            calcu_nonzero_percent(botton_bina2, white_per2);

            Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();
            vector<KeyPoint> keypoints;
            detector->detect(botton_bina2, keypoints);



            //赋值给botton
            int bt_x = 0;
            int bt_y = 0;
            int bt_r = 0;
            for (int i = 0; i < keypoints.size(); i++) {
                if (keypoints[i].pt.y > int(0.25 * botton_bina2.rows) &&
                    keypoints[i].pt.y < int(0.75 * botton_bina2.rows)) {
                    if (keypoints[i].size > bt_r) {
                        bt_x = keypoints[i].pt.x;
                        bt_y = keypoints[i].pt.y;
                        bt_r = keypoints[i].size;
                    }
                }
            }

            //是否放大
            if (bt_y > int(0.6667 * botton_bina2.rows) && bt_y < int(0.71 * botton_bina2.rows)) {
                large = 0;
                // LOGI("normal");
            } else if (bt_y > int(0.28 * botton_bina2.rows) &&
                       bt_y < int(0.6667 * botton_bina2.rows)) {
                large = 1;
                // LOGI("large");
            } else {
                large = -1;
                //  LOGI("bg");
            }
            // rectangle( botton,Rect(35,485,30,30), Scalar(0,0,255),2 );
            // imshow("keypoints",botton);
        }
    }

    bool is_upload(Mat &img) {
        int height = img.rows;
        int width = img.cols;

        //Mat roi_img = img(Rect(0.75 * width, 0, width / 4, height / 3));
        if (height <= 0 || width <= 0) {
            return false;
        }

        //imwrite("/mnt/sdcard/map.jpeg",img);
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                int b = img.at<Vec3b>(i, j)[0];
                int g = img.at<Vec3b>(i, j)[1];
                int r = img.at<Vec3b>(i, j)[2];
                if ((r >= 0xCB && r <= 0xF9) && (g >= 0x5A && g <= 0x87) &&
                    (b >= 0x0A && b <= 0x28)) {
                    return true;
                }
            }
        }
        return false;
    }

//计算主角的坐标和半径
    void self_loc(Mat &map_view, Point &self_center, float &self_radius, int &has_orange) {
        bool debug = false;
        if (debug) { imshow("map_view", map_view); }
        resize(map_view, map_view, Size(160, 160));

        //分类开
        vector<Mat> masks;
        masks.push_back(Mat::zeros(map_view.rows, map_view.cols, CV_8UC1));//yellow
        masks.push_back(Mat::zeros(map_view.rows, map_view.cols, CV_8UC1));//blue
        masks.push_back(Mat::zeros(map_view.rows, map_view.cols, CV_8UC1));//green
        masks.push_back(Mat::zeros(map_view.rows, map_view.cols, CV_8UC1));//orange
        for (int r = 0; r < map_view.rows; r++) {
            for (int c = 0; c < map_view.cols; c++) {
                //黄色
                if (map_view.at<Vec3b>(r, c)[2] > 180 && map_view.at<Vec3b>(r, c)[2] < 250 &&
                    map_view.at<Vec3b>(r, c)[1] > 163 && map_view.at<Vec3b>(r, c)[1] < 205 &&
                    map_view.at<Vec3b>(r, c)[0] > 40 && map_view.at<Vec3b>(r, c)[0] < 80) {
                    masks[0].at<uchar>(r, c) = 255;
                }
                //蓝色
                if (map_view.at<Vec3b>(r, c)[2] > 30 && map_view.at<Vec3b>(r, c)[2] < 70 &&
                    map_view.at<Vec3b>(r, c)[1] > 110 && map_view.at<Vec3b>(r, c)[1] < 150 &&
                    map_view.at<Vec3b>(r, c)[0] > 150 && map_view.at<Vec3b>(r, c)[0] < 190) {
                    masks[1].at<uchar>(r, c) = 255;
                }
                //绿色
                if (map_view.at<Vec3b>(r, c)[2] > 90 && map_view.at<Vec3b>(r, c)[2] < 140 &&
                    map_view.at<Vec3b>(r, c)[1] > 130 && map_view.at<Vec3b>(r, c)[1] < 178 &&
                    map_view.at<Vec3b>(r, c)[0] > 50 && map_view.at<Vec3b>(r, c)[0] < 84) {
                    masks[2].at<uchar>(r, c) = 255;
                }

                //橙色
                if (map_view.at<Vec3b>(r, c)[0] < 100
                    && map_view.at<Vec3b>(r, c)[0] < map_view.at<Vec3b>(r, c)[1]
                    && map_view.at<Vec3b>(r, c)[1] < 150
                    && (map_view.at<Vec3b>(r, c)[1] - map_view.at<Vec3b>(r, c)[0]) > 30
                    && map_view.at<Vec3b>(r, c)[1] < map_view.at<Vec3b>(r, c)[2]
                    && (map_view.at<Vec3b>(r, c)[2] - map_view.at<Vec3b>(r, c)[1]) > 50
                    && map_view.at<Vec3b>(r, c)[2] > 120) {
                    masks[3].at<uchar>(r, c) = 250;
                }

            }
        }

        //后处理
        if (cv::countNonZero(masks[3] > 2)) {
            has_orange = 1;
        }

        for (int i = 0; i < masks.size(); i++) {
            Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
            morphologyEx(masks[i], masks[i], MORPH_DILATE, element);
        }


        if (debug) { imshow("yellow", masks[0]); }
        if (debug) { imshow("blue", masks[1]); }
        if (debug) { imshow("green", masks[2]); }
        if (debug) { imshow("orange", masks[3]); }

        //确定which color 是主角
        int roi_w = 40;
        int roi_h = 40;
        int roi_x = map_view.cols / 2 - roi_w / 2;
        int roi_y = map_view.rows / 2 - roi_h / 2;

        vector<int> nonezeros;
        int self_ind = -1;
        int self = -1;
        for (int i = 0; i < masks.size(); i++) {

            if (roi_x < 0 || roi_y < 0) {
                continue;
            }

            if (roi_w <= 0) {
                continue;
            }

            if (roi_h <= 0) {
                continue;
            }

            if (roi_x + roi_w > masks[i].cols) {
                continue;
            }

            if (roi_y + roi_h > masks[i].rows) {
                continue;
            }

            Mat roi = masks[i](Rect(roi_x, roi_y, roi_w, roi_h));
            int nonezero = cv::countNonZero(roi);
            nonezeros.push_back(nonezero);
            if (nonezero > 0 && nonezero > self) {
                self = nonezero;
                self_ind = i;
            }
        }

        //计算主角位置 半径
        if (self_ind == -1) {
            self_center = Point(0, 0);
            self_radius = 0.0;
            return;
        }

        if (roi_x < 0 || roi_y < 0) {
            return;
        }

        if (roi_w <= 0) {
            return;
        }

        if (roi_h <= 0) {
            return;
        }

        if (roi_x + roi_w > masks[self_ind].cols) {
            return;
        }

        if (roi_y + roi_h > masks[self_ind].rows) {
            return;
        }
        Mat out = masks[self_ind](Rect(roi_x, roi_y, roi_w, roi_h));
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(out, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point());

        Point2f center;
        float radius;
        for (int i = 0; i < contours.size(); i++) {
            //绘制轮廓的最小外结圆
            minEnclosingCircle(contours[i], center, radius);
            if (radius > self_radius) {
                self_radius = radius;
                self_center = center;
            }
        }

        //he fa xing jian cha
        if (self_radius == 0) {
            self_center = Point(0, 0);
            self_radius = 0.0;
        }


        self_center.x += roi_x;
        self_center.y += roi_y;

        if (debug && self_radius > 0) {
            circle(map_view, self_center, self_radius, Scalar(0, 0, 255), 1);
            // imshow( "map_view_", map_view );
        }
    }


//根据first的值升序排序
    bool cmp1(pair<int, int> a, pair<int, int> b) {
        return a.first < b.first;
    }

//根据second的值升序排序
    bool cmp2(pair<int, int> a, pair<int, int> b) {
        return a.second < b.second;
    }

//根据second的值升序排序
    bool cmp2_float(pair<int, float> a, pair<int, float> b) {
        return a.second < b.second;
    }

    void orient1(Mat &map_view, Point &self_center, float &self_radius, float &self_atan) {
        bool debug = false;
        bool show_it = false;
        if (show_it) imshow("map_view", map_view);

        //极坐标变换
        vector<Mat> cs;
        vector<Mat> cs_polar;
        Mat map_polar;
        split(map_view, cs);
        for (int i = 0; i < 3; i++) {
            Mat tmp;
            Point2f center((float) map_view.cols / 2, (float) map_view.rows / 2);
            double M = map_view.cols / 2;
            linearPolar(cs[i], tmp, center, M, INTER_LINEAR + WARP_FILL_OUTLIERS);

            cs_polar.push_back(tmp);
        }
        merge(cs_polar, map_polar);
        if (show_it) imshow("map_polar", map_polar);


        Mat map_polar_bina = cv::Mat::zeros(map_polar.size(), CV_8UC1);//阈值小
        Mat map_polar_bina1 = cv::Mat::zeros(map_polar.size(), CV_8UC1);//阈值大
        for (int r = 0; r < map_polar.rows; r++) {
            for (int c = 0; c < map_polar.cols; c++) {
                if (map_polar.at<Vec3b>(r, c)[0] > 100 && map_polar.at<Vec3b>(r, c)[1] > 100 &&
                    map_polar.at<Vec3b>(r, c)[2] > 100) {
                    map_polar_bina.at<uchar>(r, c) = 255;
                }
                if (map_polar.at<Vec3b>(r, c)[0] > 200 && map_polar.at<Vec3b>(r, c)[1] > 200 &&
                    map_polar.at<Vec3b>(r, c)[2] > 200) {
                    map_polar_bina1.at<uchar>(r, c) = 255;
                }
            }
        }

        int ss = map_polar_bina.cols / 4;
        map_polar_bina(Range(0, map_polar_bina.rows), Range(0, ss)) = 0;
        map_polar_bina(Range(0, map_polar_bina.rows),
                       Range(map_polar_bina.cols * 3 / 4, map_polar_bina.cols)) = 0;
        if (show_it) imshow("map_polar_bina", map_polar_bina);

        // map_polar_bina1( Range( 0,map_polar_bina1.rows ), Range(0,map_polar_bina1.cols / 4 ) ) = 0 ;
        // map_polar_bina1( Range( 0,map_polar_bina1.rows ), Range( map_polar_bina1.cols / 2, map_polar_bina1.cols  ) ) = 0 ;
        // imshow( "map_polar_bina1", map_polar_bina1 );

        Mat map_polar_bina_trunc = cv::Mat::zeros(map_polar.size(), CV_8UC1);
        for (int r = 0; r < map_polar_bina.rows; r++) {
            bool cut = false;
            for (int c = 0; c < map_polar_bina.cols - 1; c++) {
                if (map_polar_bina.at<uchar>(r, ss) == 0) {
                    break;
                } else if (map_polar_bina.at<uchar>(r, c) > 0 &&
                           map_polar_bina.at<uchar>(r, c + 1) == 0) {
                    cut = false;
                    break;
                } else {
                    map_polar_bina_trunc.at<uchar>(r, c) = map_polar_bina.at<uchar>(r, c);
                }

            }
        }
        if (show_it) imshow("trunc", map_polar_bina_trunc);

        Mat twice = cv::Mat::zeros(Size(map_polar.cols, map_polar.rows * 2), CV_8UC1);

        map_polar_bina_trunc.copyTo(twice(Range(0, twice.rows / 2), Range(0, twice.cols)));
        map_polar_bina_trunc.copyTo(twice(Range(twice.rows / 2, twice.rows), Range(0, twice.cols)));
        if (show_it) imshow("twice", twice);

        //既要考虑宽度，也要考虑长度
        vector<int> cnts;
        mapto_vertical(twice, cnts);
        //查找每一段开始结束位置
        vector<pair<int, int>> start_ends;
        for (int i = 0; i < cnts.size(); i++) {
            static int ind1 = -1;
            static int ind2 = -1;

            static bool begin_flag = false;
            if (!begin_flag && cnts[i] > 0) {
                begin_flag = true;
                ind1 = i;
            } else if (begin_flag && (cnts[i] == 0 || i == cnts.size() - 1)) {
                ind2 = i;
                begin_flag = false;
                pair<int, int> start_end = make_pair(ind1, ind2);
                start_ends.push_back(start_end);
            }
        }

        vector<int> areas;
        int max_area = 0;
        int max_begin = 0;
        int max_end = 0;

        for (int i = 0; i < start_ends.size(); i++) {
            int begin = start_ends[i].first;
            int end = start_ends[i].second;
            int area = 0;
            for (int r = begin; r < end; r++) {
                for (int c = 0; c < twice.cols; c++) {
                    if (twice.at<uchar>(r, c) > 0) {
                        area++;
                    }
                }
            }

            areas.push_back(area);
            if (area > max_area) {
                max_area = area;
                max_begin = begin;
                max_end = end;
            }
        }
        if (max_end - max_begin < 5) {
            return;
        }
        //
        // int max_ind = 0;
        int max_cnt = 0;
        for (int i = max_begin; i < max_end; i++) {
            if (cnts[i] > max_cnt) {
                max_cnt = cnts[i];
                // max_ind = i;
            }

        }
        int n = 0;
        int max_ind = 0;
        for (int i = max_begin; i < max_end; i++) {
            if (cnts[i] > int(max_cnt * 0.8)) {
                max_ind += i;
                n++;
            }
        }
        max_ind = max_ind / n;


        if (show_it) {
            line(twice, Point(0, max_ind), Point(20, max_ind), 255, 2);
            imshow("ttt", twice);
        }


        if (max_ind >= twice.rows / 2) {
            self_atan = (twice.rows - max_ind) * 360 / (twice.rows / 2);
        } else {
            self_atan = (twice.rows / 2 - max_ind) * 360 / (twice.rows / 2);
        }
        if (debug) cout << self_atan << endl;
    }


//计算主角身边三角形的角度
    void
    orient(Mat &map_view, Point &self_center, float &self_radius, Mat &last_bina, Mat &this_bina,
           int &change_flag, float &self_atan) {
        //计算环状物体
        bool show_it = false;
        bool debug = false;

        Mat mask = cv::Mat::zeros(map_view.size(), CV_8UC1);
        circle(mask, self_center, self_radius * 1.5, Scalar(1), -1);//
        circle(mask, self_center, self_radius, Scalar(0), -1);
        if (show_it) { imshow("mask", mask); } //可视化的时候是255

        Mat map_mask;
        vector<Mat> map_cs;
        split(map_view, map_cs);
        for (int i = 0; i < map_cs.size(); i++) {
            multiply(map_cs[i], mask, map_cs[i]);
        }
        merge(map_cs, map_mask);
        if (show_it) imshow("map_mask", map_mask);

        //----------------------
        // Mat tmp_gray, tmp_bina;
        // cvtColor( map_mask, tmp_gray , CV_BGR2GRAY);

        // for( int th = 0; th<255; th++ ){
        // threshold( tmp_gray, tmp_bina, th, 255, THRESH_BINARY );
        // imshow( "bina", tmp_bina );
        // waitKey(100);
        // }
        //-----------------------------

        //二值化
        Mat mask1 = cv::Mat::zeros(map_mask.size(), CV_8UC1);
        for (int r = 0; r < map_mask.rows; r++) {
            for (int c = 0; c < map_mask.cols; c++) {

                if (map_mask.at<Vec3b>(r, c)[2] > 235 && map_mask.at<Vec3b>(r, c)[1] > 235 &&
                    map_mask.at<Vec3b>(r, c)[0] > 235) {
                    mask1.at<uchar>(r, c) = 250;
                }
            }
        }
        if (show_it) imshow("mask1", mask1);

        //根据上次的结果进行预判断
        this_bina = mask1;
        Mat diff;
        absdiff(last_bina, this_bina, diff);
        if (countNonZero(diff) > 5) {
            change_flag = 1;
            last_bina = this_bina;
        } else {
            change_flag = 0;
            return;
        }

        //计算二值化以后的块的数量
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(mask1, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE, Point());

        //计算多种情况下的角度
        if (contours.size() == 0) {//检测不到白色三角
            return;
        } else if (contours.size() == 1) {//只有一块白色物体
            //在中间扣取一小块计算坐标xy的平均值
            int patch_x1 = mask1.cols / 4;
            int patch_x2 = mask1.cols * 3 / 4;
            int patch_y1 = mask1.rows / 4;
            int patch_y2 = mask1.rows * 3 / 4;
            int x_sum = 0;
            int y_sum = 0;
            int n_sum = 0;
            for (int r = patch_y1; r < patch_y2; r++) {
                for (int c = patch_x1; c < patch_x2; c++) {
                    if ((int) mask1.at<uchar>(r, c) == 250) {
                        x_sum += c;
                        y_sum += r;
                        n_sum++;

                    }
                }
            }

            if (n_sum > 0) {
                self_atan = fastAtan2(-1 * (y_sum / n_sum - self_center.y),
                                      x_sum / n_sum - self_center.x);
            }
            if (debug) cout << "self_atan_single block:" << self_atan << endl;

        }//只有一块白色物体
        else if (contours.size() > 1) { //多个白色物体
            //极坐标变换
            Mat polar_img;
            Mat polar_img1;

            double M = 40;
            linearPolar(mask1, polar_img, self_center, M, INTER_LINEAR + WARP_FILL_OUTLIERS);
            Mat element;
            int s = 3;
            element = getStructuringElement(MORPH_ELLIPSE, Size(s, s));
            morphologyEx(polar_img, polar_img, MORPH_ERODE, element);
            if (show_it) imshow("polar", polar_img);

            //去掉中间隔离的block
            polar_img1 = cv::Mat::zeros(polar_img.size(), CV_8UC1);
            for (int r = 0; r < polar_img.rows; r++) {
                for (int c = 0; c < polar_img.cols - 1; c++) {
                    if (polar_img.at<uchar>(r, c) > 0 && polar_img.at<uchar>(r, c + 1) == 0) {
                        break;
                    } else {
                        polar_img1.at<uchar>(r, c) = polar_img.at<uchar>(r, c);
                    }

                }
            }
            if (show_it) imshow("plar1", polar_img1);

            //统计每一行非零的个数
            vector<int> cnts;
            for (int r = 0; r < polar_img1.rows; r++) {
                int cnt = 0;
                for (int c = 0; c < polar_img1.cols; c++) {
                    if (polar_img1.at<uchar>(r, c) == 250) {
                        cnt++;
                    }
                }
                cnts.push_back(cnt);
                if (debug) cout << r << " " << cnt << endl;
            }


            //查找每一段开始结束位置
            vector<pair<int, int>> start_ends;
            for (int i = 0; i < cnts.size(); i++) {
                static int ind1 = -1;
                static int ind2 = -1;

                static bool begin_flag = false;
                if (!begin_flag && cnts[i] > 0) {
                    begin_flag = true;
                    ind1 = i;
                } else if (begin_flag && (cnts[i] == 0 || i == cnts.size() - 1)) {
                    ind2 = i;
                    begin_flag = false;
                    pair<int, int> start_end = make_pair(ind1, ind2);
                    start_ends.push_back(start_end);
                }
            }

            if (debug) {
                for (int i = 0; i < start_ends.size(); i++) {
                    cout << "start:" << start_ends[i].first << " end: " << start_ends[i].second
                         << endl;
                }
            }

            //去除过大和过小的block
            vector<pair<int, int>>::iterator it;
            for (it = start_ends.begin(); it != start_ends.end();) {
                if ((*it).second - (*it).first > 10 || (*it).second - (*it).first < 3) {
                    it = start_ends.erase(it);
                } else {
                    ++it;
                }
            }
            if (start_ends.size() == 0) {
                return;
            }

            if (debug) {
                for (int i = 0; i < start_ends.size(); i++) {
                    cout << start_ends[i].first << " " << start_ends[i].second << endl;
                }
            }

            //特征提取
            vector<vector<float>> feats;
            for (int i = 0; i < start_ends.size(); i++) {
                int begin = start_ends[i].first;
                int end = start_ends[i].second;

                //计算箭头向左还是向右
                Mat roi = mask1(Rect(0, begin, mask1.cols, end - begin));
                vector<int> v_map;
                int left_ds = 0;
                bool left_wide = 0;
                mapto_vertical(roi, v_map);
                for (int j = 0; j < v_map.size() - 1; j++) {
                    if (v_map[j] >= v_map[j + 1])
                        left_ds++;
                }
                if (left_ds * 1.0 / v_map.size() > 0.5) {
                    left_wide = 1;
                }

                //判断
                for (int j = begin; j < end; j++) {
                    vector<float> feat;
                    feat.push_back(end - begin);//0
                    feat.push_back(left_wide); //1
                    feat.push_back(j);// 2

                    int as = 0;
                    int ds = 0;
                    for (int k = begin; k < j; k++) {
                        if (cnts[k] <= cnts[k + 1])
                            as++;
                    }
                    for (int k = j; k < end; k++) {
                        if (cnts[k] >= cnts[k + 1])
                            ds++;
                    }
                    float f = 1.0 * (as + ds) / (end - begin);
                    feat.push_back(f);// 3
                    feat.push_back(3 * left_ds + 0.3 * (end - begin) + 0.7 * f);//4

                    //
                    feats.push_back(feat);
                }

            }

            //查找最大值
            int max_ind = 0;
            float max_score = 0;
            for (int i = 0; i < feats.size(); i++) {
                if (feats[i][4] > max_score) {
                    max_ind = i;
                    max_score = feats[i][4];
                }
            }

            if (max_score > 5.2) {//3*1 + 0.3*6+0.7*4/6=2.2 #最小得分
                if (show_it) {
                    // cout<<"max score correspond:"<<feats[max_ind][1]<<endl; // max score in cnts
                    line(polar_img1, Point(0, feats[max_ind][2]), Point(20, feats[max_ind][2]), 255,
                         2);
                    imshow("index", polar_img1);
                }

                self_atan = 360 * (mask1.rows - feats[max_ind][2]) / mask1.rows;
                cout << "self_atan_multiple  block:" << self_atan << endl;


            } else {
                if (debug) cout << "score is too low, can not detect " << endl;
                return;
            }

        }//多个白色物体

    }//orient





//使用固定宽度来卡住光住
    void
    orient3(Mat &map_view, Mat &last_bina, Mat &this_bina, int &change_flag, float &self_atan) {

        bool debug = false;
        bool show_it = false;
        int guangzhu = 35; // 光柱的宽度
        if (show_it) imshow("map_view", map_view);


        //极坐标变换
        vector<Mat> cs;
        vector<Mat> cs_polar;
        Mat map_polar;
        split(map_view, cs);
        for (int i = 0; i < 3; i++) {
            Mat tmp;
            Point2f center((float) map_view.cols / 2, (float) map_view.rows / 2);
            double M = map_view.cols / 2;
            linearPolar(cs[i], tmp, center, M, INTER_LINEAR + WARP_FILL_OUTLIERS);

            cs_polar.push_back(tmp);
        }
        merge(cs_polar, map_polar);
        if (show_it) imshow("map_polar", map_polar);

        //二值化
        Mat map_polar_bina = cv::Mat::zeros(map_polar.size(), CV_8UC1);//阈值小
        for (int r = 0; r < map_polar.rows; r++) {
            for (int c = 0; c < map_polar.cols; c++) {
                if (map_polar.at<Vec3b>(r, c)[0] > 100 && map_polar.at<Vec3b>(r, c)[1] > 100 &&
                    map_polar.at<Vec3b>(r, c)[2] > 100) {
                    map_polar_bina.at<uchar>(r, c) = 255;
                }
            }
        }
        cout << map_polar_bina.cols << " " << map_polar_bina.rows << endl;

        //使用上次的结果，进行预先判断
        this_bina = map_polar_bina;
        Mat diff;
        absdiff(this_bina, last_bina, diff);
        if (countNonZero(diff) > 80) {
            change_flag = 1;
            last_bina = this_bina;// 更新
        } else {
            change_flag = 0;
            return;
        }

        int ss = map_polar_bina.cols / 4;
        map_polar_bina(Range(0, map_polar_bina.rows), Range(0, ss)) = 0;
        map_polar_bina(Range(0, map_polar_bina.rows),
                       Range(map_polar_bina.cols * 3 / 4, map_polar_bina.cols)) = 0;
        if (show_it) imshow("map_polar_bina", map_polar_bina);

        // map_polar_bina1( Range( 0,map_polar_bina1.rows ), Range(0,map_polar_bina1.cols / 4 ) ) = 0 ;
        // map_polar_bina1( Range( 0,map_polar_bina1.rows ), Range( map_polar_bina1.cols / 2, map_polar_bina1.cols  ) ) = 0 ;
        // imshow( "map_polar_bina1", map_polar_bina1 );

        Mat map_polar_bina_trunc = cv::Mat::zeros(map_polar.size(), CV_8UC1);
        for (int r = 0; r < map_polar_bina.rows; r++) {
            bool cut = false;
            for (int c = 0; c < map_polar_bina.cols - 1; c++) {
                if (map_polar_bina.at<uchar>(r, ss) == 0) {
                    break;
                } else if (map_polar_bina.at<uchar>(r, c) > 0 &&
                           map_polar_bina.at<uchar>(r, c + 1) == 0) {
                    cut = false;
                    break;
                } else {
                    map_polar_bina_trunc.at<uchar>(r, c) = map_polar_bina.at<uchar>(r, c);
                }

            }
        }
        if (show_it) imshow("trunc", map_polar_bina_trunc);

        Mat twice = cv::Mat::zeros(Size(map_polar.cols, map_polar.rows * 2), CV_8UC1);

        map_polar_bina_trunc.copyTo(twice(Range(0, twice.rows / 2), Range(0, twice.cols)));
        map_polar_bina_trunc.copyTo(twice(Range(twice.rows / 2, twice.rows), Range(0, twice.cols)));
        if (show_it) imshow("twice", twice);

        //通过光住的宽度，卡住最佳
        vector<int> v_map;
        mapto_vertical(twice, v_map);
        int max_pix = 0;
        int max_ind = 0;
        for (int i = 0; i < v_map.size() - guangzhu; i++) {

            int seg_pix = 0;
            int j = i;
            for (; j < i + guangzhu; j++) {
                seg_pix += v_map.at(j);
            }
            if (seg_pix > max_pix) {
                max_pix = seg_pix;
                max_ind = j - guangzhu / 2;//违反直觉
            }

        }
        if (show_it) {
            line(twice, Point(0, max_ind), Point(20, max_ind), 255, 2);
            imshow("ttt", twice);

        }
        if (max_ind >= twice.rows / 2) {
            self_atan = (twice.rows - max_ind) * 360 / (twice.rows / 2);
        } else {
            self_atan = (twice.rows / 2 - max_ind) * 360 / (twice.rows / 2);
        }
        if (debug) cout << self_atan << endl;

    }


    void obj_detect(Mat &map, vector<int> &kinds, vector<int> &thetas) {
        bool show_it = false;
        bool debug = false;
        resize(map, map, cv::Size(150, 150));//固定尺寸，以便判断目标种类
        if (show_it) imshow("map", map);

        //获取橙色区域
        Mat mask = cv::Mat::zeros(map.size(), CV_8UC1);
        for (int r = 0; r < map.rows; r++) {
            for (int c = 0; c < map.cols; c++) {
                //橙色
                if (map.at<Vec3b>(r, c)[0] < 100
                    && map.at<Vec3b>(r, c)[0] < map.at<Vec3b>(r, c)[1]
                    && map.at<Vec3b>(r, c)[1] < 150
                    && (map.at<Vec3b>(r, c)[1] - map.at<Vec3b>(r, c)[0]) > 30
                    && map.at<Vec3b>(r, c)[1] < map.at<Vec3b>(r, c)[2]
                    && (map.at<Vec3b>(r, c)[2] - map.at<Vec3b>(r, c)[1]) > 50
                    && map.at<Vec3b>(r, c)[2] > 120) {
                    mask.at<uchar>(r, c) = 250;
                }

            }
        }
//    if( debug )
//    imwrite( "/storage/emulated/0/mask.jpg", mask );


//    //去掉中心可能出现的橙色
//    for( int r = int( 3.0/8*mask.rows ); r <  int( 5.0/8*mask.rows ); r++ ){
//        for( int c = int( 3.0/8*mask.cols ); c <  int( 5.0/8*mask.cols ); c++ ){
//            mask.at<uchar>(r,c) = 0;
//        }
//    }
//    if( show_it ) imshow( "mask", mask );


        //去除橙色队友 和自己的影响
        mask(Rect(55, 55, 40, 40)) = 0;
        Mat mask_bak = mask.clone();

        try {
            vector<vector<Point>> contours;
            vector<Vec4i> hierarcy;
            findContours(mask_bak, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

            vector<RotatedRect> rects;
            for (int i = 0; i < contours.size(); i++) {
                //绘制轮廓的最小外结矩形
                RotatedRect rect = minAreaRect(contours[i]);
                rects.push_back(rect);
            }

            for (int i = 0; i < rects.size(); i++) {
                RotatedRect rect = rects[i];
                if (rect.size.width > 10 && (rect.size.width / rect.size.height) < 1.1) {
                    Rect brect = rect.boundingRect();
                    mask(brect) = 0;
                    // rectangle( mask_bak, brect, (255), 2  );
                }
            }
            // if( show_it ) imshow( "mask_bak__", mask_bak );

            if (show_it) imshow("mask", mask);

        }
        catch (exception &e) { ;
        }

        //极坐标变换
        Mat polar_img;
        Point2f center((float) mask.cols / 2, (float) map.rows / 2);
        double M = mask.cols / 2;
        linearPolar(mask, polar_img, center, M, INTER_LINEAR + WARP_FILL_OUTLIERS);
        if (show_it) imshow("polar", polar_img);

        //开运算比运算，去除小间隙
        Mat element;
        int s = 3;
        element = getStructuringElement(MORPH_ELLIPSE, Size(s, s));
        morphologyEx(polar_img, polar_img, MORPH_OPEN, element);
        if (show_it) imshow("open_close", polar_img);
//    if(debug) imwrite( "/storage/emulated/0/polar_img.jpg", polar_img );

        vector<int> v_map;
        mapto_vertical(polar_img, v_map);

        int begin_ind = -1;
        int end_ind = -1;
        bool begin_flag = false;
        vector<int> begin_end_pair;
        for (int i = 0; i < v_map.size(); i++) {
            if (!begin_flag && v_map[i] > 1) {
                begin_ind = i;
                begin_flag = true;
                begin_end_pair.push_back(begin_ind);
            }
            if (begin_flag && (v_map[i] < 1 || i == v_map.size() - 1)) {
                end_ind = i;
                begin_flag = false;
                begin_end_pair.push_back(end_ind);
            }
        }

        //如果找到了起始坐标
        if (begin_end_pair.size() > 0 && begin_end_pair.size() % 2 == 0) {

            for (int i = 0; i < begin_end_pair.size(); i += 2) {
                begin_ind = begin_end_pair[i];
                end_ind = begin_end_pair[i + 1];

                if (begin_ind < 0 || (end_ind - begin_ind) <= 0 || mask.cols <= 0) {
                    continue;
                }

                if (mask.cols > polar_img.cols) {
                    continue;
                }

                if (end_ind > polar_img.rows) {
                    continue;
                }

                // cout<<begin_ind<<"<--->"<<end_ind<<endl;
                Mat seg = polar_img(Rect(0, begin_ind, mask.cols, end_ind - begin_ind));
                if (show_it) {
                    imshow("seg", seg);
                    waitKey(1000);
                }

                //查找轮廓，目标分类
                vector<vector<Point>> contours1;
                vector<Vec4i> hierarcy1;
                findContours(seg, contours1, hierarcy1, RETR_EXTERNAL, CHAIN_APPROX_NONE);

                int cnt = 0;
                for (int i = 0; i < contours1.size(); i++) {
                    // RotatedRect rect=minAreaRect(contours[i]);
                    // cout<<rect.size.width*rect.size.height <<endl;
                    Rect rect = boundingRect(Mat(contours1[i]));
                    if (rect.height * 2 > seg.rows) {
                        cnt++;
                        if (show_it)rectangle(seg, rect, (255), 2);
                    }

                }

                if (show_it) {
                    imshow("11", seg);
                    // waitKey(3000);
                }

                // cout<<cnt<<endl;
                if (cnt == 1 || cnt == 2 || cnt == 4) {
                    int theta = int((1 - (begin_ind + end_ind) / 2.0 / polar_img.rows) * 360);
                    //cout<<theta<<" bullet" <<endl;
                    thetas.push_back(theta);
                    kinds.push_back(0);

                } else if (cnt == 3) {
                    int theta = int((1 - (begin_ind + end_ind) / 2.0 / polar_img.rows) * 360);
                    //cout<<theta<<" footprint"<<endl;
                    thetas.push_back(theta);
                    kinds.push_back(1);

                }
            }
        }
    }

//
// //方法1 计算地图的外框
// void m1_coord( Mat& frame, Mat& gray, Mat& edge, float& left_h, float& right_h, float& top_h, float& bottom_h, float& left, float& right, float& top, float& bottom, bool& m1_done ){
//     bool show_it = false;

//     //计算文字所在行数，当作地图外框的标记
//     int text_end = text_coord( frame, gray, left, right, bottom );
//     if(show_it){
//         line( gray, Point( left,text_end ), Point( right, text_end ), (255,255,255),1 );
//         imshow( "text_line", gray );
//     }


//     //进行直线检测，获取地图外框坐标
//     int padding = int( (right - left)/8.0 );
//     Mat roi = edge( Rect( left - padding, 0,right-left+2*padding, edge.rows ) );
//     if(show_it) imshow("m1_roi", roi);

//     //检测直线
//     int target_width = roi.cols;
//     vector<Vec4i> lines;

//     int minlen = int(target_width*0.3);
//     int gap = int(target_width*0.2);
//     HoughLinesP(roi, lines, 1, CV_PI / 90, 150, minlen, gap);

//     //依次画出每条线段，由于地图很密集，因此误差很大
//     if (false){
//         Mat gray_c = roi.clone();
//         for (size_t i = 0; i < lines.size(); i++)
//             {
//                 Vec4i l = lines[i];
//                 line(gray_c,Point(l[0], l[1]),Point(l[2], l[3]), Scalar(255, 255, 255), 2);
//             }
//         imshow("hough", gray_c);
//     }

//     //排除可能的误检. 为了防止地图的背景表格混淆，确定大致范围
//     vector<Vec4i> lines_l;
//     vector<Vec4i> lines_r;
//     vector<Vec4i> lines_tp;
//     vector<Vec4i> lines_b;

//     int leftmax = padding;
//     int rightmin = roi.cols - padding;
//     int topmax = top+padding;

//     // for (size_t i = 0; i < lines.size(); i++){
//     //     Vec4i l = lines[i];
//     //     if( abs(l[0] == l[3])<4 ){//垂直

//     //     }else if


//     // }

//     int tmp_l = 0;
//     int tmp_r = 0;
//     int tmp_tp = 0;
//     int tmp_b = 0;

//     for (size_t i = 0; i < lines.size(); i++){
//         Vec4i l = lines[i];
//         if( l[0]>0 && l[0]<leftmax  ){
//             tmp_l = l[0];
//         }
//         else if( l[0]>rightmin && l[0]<roi.cols ){
//             tmp_r = l[0];
//         }

//         if( l[1]>top - padding && l[1]<top ){
//             tmp_tp = l[1];
//         }else if( l[1]> bottom && l[1]< bottom + padding)
//         {
//             tmp_b = l[1];
//         }
//     }

//     //平均值
//     static int ln = 0;
//     static int rn = 0;
//     static int tn = 0;
//     static int bn = 0;
//     if( tmp_l>0 ){
//         ln++;
//         left_h = 1.0/ln*(ln-1)*left_h + 1.0/ln*(tmp_l + left-padding);//加上偏置
//     }
//     if( tmp_r>0 ){
//         rn ++;
//         right_h = 1.0/rn*(rn-1)*right_h + 1.0/rn*(tmp_r + left - padding);//加上偏置
//     }
//     if(tmp_tp>0){
//         tn++;
//         top_h = 1.0/tn*(tn-1)*top_h + 1.0/tn*tmp_tp;
//     }
//     if( tmp_b>0 ){
//         bn++;
//         bottom_h = 1.0/bn*(bn-1)*bottom_h + 1.0/bn*tmp_b;
//     }


//     //画出来
//     if (true){
//         Mat roi_c = roi.clone();
//         if( tmp_l > 0 ) line( roi_c, Point(tmp_l, 0), Point( tmp_l, roi.rows ), (255,255,255), 1 );
//         if( tmp_r > 0 ) line( roi_c, Point(tmp_r, 0), Point( tmp_r, roi.rows ), (255,255,255), 1 );
//         if( tmp_tp > 0 ) line( roi_c, Point(0, tmp_tp), Point( roi.cols, tmp_tp ), (255,255,255), 1 );
//         if( tmp_b > 0) line( roi_c, Point(0, tmp_b), Point( roi.cols, tmp_b ), (255,255,255), 1 );

//         imshow("roi_c", roi_c);
//     }

//     if (true){
//         Mat gray_c = gray.clone();
//         if( left_h> left-padding ) line( gray_c, Point(left_h, 0), Point( left_h, roi.rows ), (255,255,255), 1 );
//         if( right_h> left-padding ) line( gray_c, Point(right_h, 0), Point( right_h, roi.rows ), (255,255,255), 1 );
//         if( top_h > 0 ) line( gray_c, Point(0, top_h), Point( roi.cols, top_h ), (255,255,255), 1 );
//         if( bottom_h > 0) line( gray_c, Point(0, bottom_h), Point( roi.cols, bottom_h ), (255,255,255), 1 );

//         imshow("hough", gray_c);
//     }

//     //flag
//     if( left_h>0 && right_h>0 && top_h>0 && bottom_h>0 ){
//         m1_done = true;
//     }
// }

//准星定位，构建mask
    void shot_loc_mask(Mat roi, int min_len, int max_len, bool &loc_done, Rect &shot_loc,
                       Mat &shot_mask) {

        bool debug = false;
        bool show_it = false;
        bool save_it = false;

        //查找准星边界
        Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
        for (int r = 0; r < roi.rows; r++) {
            for (int c = 0; c < roi.cols; c++) {
                if (roi.at<Vec3b>(r, c)[0] > 180 && roi.at<Vec3b>(r, c)[2] > 180 &&
                    roi.at<Vec3b>(r, c)[2] > 180) {
                    mask.at<uchar>(r, c) = 255;
                }
            }
        }
        if (show_it) imshow("mask", mask);

        //查找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierarcy;
        findContours(mask, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        //进行筛选
        vector<Rect> squares;
        for (int i = 0; i < contours.size(); i++) {
            Rect tmp = boundingRect(Mat(contours[i]));

            //长宽比例合适
            if (tmp.width < min_len || tmp.height < min_len)
                continue;
            if (tmp.width > max_len || tmp.height > max_len)
                continue;
            if (tmp.width * 1.0 / tmp.height > 1.2 || tmp.width * 1.0 / tmp.height < 0.8)
                continue;

            if(!roi_valid(mask,tmp))
                return;
            //排除大面积光斑
            float nonezero;
            calcu_nonzero_percent(mask(tmp), nonezero);
            if (nonezero > 0.2)
                continue;

            //保留圆形
            Mat polar_img;
            Mat tmp_roi = mask(tmp);
            // if(show_it) imshow( "tmp_roi" , tmp_roi );
            Point self_center = Point(int(tmp_roi.cols / 2), int(tmp_roi.rows / 2));
            int M = tmp_roi.rows < tmp_roi.cols ? int(0.48 * tmp_roi.rows) : int(
                    0.48 * tmp_roi.cols);
            linearPolar(tmp_roi, polar_img, self_center, M, INTER_LINEAR + WARP_FILL_OUTLIERS);
            polar_img(Range(0, int(polar_img.rows)), Range(0, polar_img.cols / 2)) = 0;
            // if(show_it) imshow( "polar_img", polar_img );

            // //判断是yuanxing
            vector<int> v_map;
            mapto_vertical(polar_img, v_map);
            int none_z = 0;
            for (int i = 0; i < v_map.size(); i++) {
                if (v_map[i] > 2) {
                    none_z++;
                }
            }
            if (none_z > int(0.95 * polar_img.rows)) {
                loc_done = true;//设定flag

                // for( int r = tmp.y; r < tmp.y+tmp.height; r++ ){
                //     for( int c = tmp.x; c < tmp.x + tmp.width; c++ ){
                //         shot_mask.at<uchar>(r,c) = mask.at<uchar>(r,c);
                //     }
                // }
                shot_loc = tmp;
                shot_mask = tmp_roi.clone();
                if (show_it) imshow("shot_mask", shot_mask);
            }
        }
        return;

    }

    void shot_yellow(Mat &shot_roi, Mat &shot_mask, Mat &shot_yellow_mask) {
        bool show_it = false;
        //颜色判断


        //识别黄色
        Mat shot_hsv;
        cvtColor(shot_roi, shot_hsv, COLOR_BGR2HSV);

        for (int r = 0; r < shot_hsv.rows; r++) {
            for (int c = 0; c < shot_hsv.cols; c++) {
                if (int(shot_mask.at<uchar>(r, c)) > 0) {
                    int h = shot_hsv.at<Vec3b>(r, c)[0];
                    int s = shot_hsv.at<Vec3b>(r, c)[1];
                    int v = shot_hsv.at<Vec3b>(r, c)[2];

                    if (h >= 15 && h <= 30 && s >= 120 && s <= 255 && v >= 150 && v <= 255) {
                        shot_yellow_mask.at<uchar>(r, c) = 255;
                    }

                }


            }
        }

        if (show_it) imshow("shot_yellow_mask", shot_yellow_mask);

        return;

    }

//准星位置检测和状态判断
    void shot_sight(Mat roi, int &min_len, int &max_len, int &yellow_shot, int reset) {
        bool show_it = false;
        bool debug = false;

        static Rect shot_loc = Rect(0, 0, 0, 0);
        static Mat shot_mask = cv::Mat::zeros(roi.size(), CV_8UC1);
        static bool loc_done = false;

        if (reset || !loc_done) {
            shot_loc_mask(roi, min_len, max_len, loc_done, shot_loc, shot_mask);
        }

        //判断颜色
        if (loc_done) {
            if(!roi_valid(roi,shot_loc))
                return;
            Mat shot_roi = roi(shot_loc);
            Mat shot_yellow_mask = cv::Mat::zeros(shot_roi.size(), CV_8UC1);
            if (show_it) imshow("shot_roi", shot_roi);

            // BGR图像中准星位置 圆环的mask  圆环中黄色的mask
            shot_yellow(shot_roi, shot_mask, shot_yellow_mask);
//        imwrite("sdcard/shot_roi.jpg", shot_roi);
//        imwrite("sdcard/shot_mask.jpg", shot_mask);
//        imwrite("sdcard/shot_yellow_mask.jpg", shot_yellow_mask);


            int mask_cnt = countNonZero(shot_mask);
            int yellow_cnt = countNonZero(shot_yellow_mask);

            if (yellow_cnt > 0.11 * mask_cnt) {
                yellow_shot = 1;
                ALOGV_M("yellow:%d", yellow_shot);
                if (debug) cout << "yellow" << endl;
            } else {
                yellow_shot = 0;
            }
        }


        // //保存图片
        // for( int i = 0; i < squares.size(); i++ ){
        //     // rectangle( mask, squares[i], (255), 3  );

        //     int time = -1*getCurrentTime();
        //     string img_name = to_string(time) + ".jpg";
        //     Mat roi = mask( squares[i] );
        //     imwrite( img_name, roi );
        // }

        // imshow( "tmp", mask );


    }//shot_sight

/**========================================以下是新接口========================================**/
    int pubg_shot_sight(cv::Mat &scene, int reset) {
        int c_begin = scene.cols / 4 * 3 + 50;
        int w = scene.cols / 4 - 50;

        int r_begin = scene.rows / 4;
        int h = scene.rows / 4 * 3;

        cv::Rect rect(c_begin, r_begin, w, h);
        if (!roi_valid(scene, rect)) {
            return -1;
        }

        Mat roi = scene(rect);
        if (roi.empty())
            return -1;

        //0表示准星是白色，1表示黄色
        int yellow_shot = -1;
        // 最小高度
        int min_len = int(0.05 * scene.rows);
        //最大高度
        int max_len = int(0.14 * scene.rows);

        PUBG::shot_sight(roi, min_len, max_len, yellow_shot, reset);

        return yellow_shot;
    } // shot_sight(oz::OZPic *m_pic, int reset)

    bool pubg_in_game_icon(cv::Mat &scene) {
        bool game_start = false;

        if (scene.empty())
            return game_start;
        Mat hsv;
        cv::Rect rect = Rect(20, 0, scene.cols / 3.2, scene.rows / 8.2);
        if (!roi_valid(scene, rect)) {
            return false;
        }
        Mat roi = scene(rect);
        if (roi.empty()) {
            return false;
        }
        cvtColor(roi, hsv, COLOR_BGR2HSV);
        Mat mask = cv::Mat(roi.rows, roi.cols, CV_8UC1, cv::Scalar(0));

        double h = 0.0, s = 0.0, v = 0.0;
        for (int i = 0; i < int(roi.rows); i++) {
            for (int j = 0; j < int(roi.cols); j++) {
                h = hsv.at<Vec3b>(i, j)[0];
                s = hsv.at<Vec3b>(i, j)[1];
                v = hsv.at<Vec3b>(i, j)[2];

                if (h > 15 && h <= 30) {
                    if (s >= 150 && s <= 255) {
                        if (v >= 150 && v <= 255) {
                            mask.at<uchar>(i, j) = 255;
                        }
                    }
                }
            }
        }

        Mat elem = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
        dilate(mask, mask, elem);
        elem = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
        erode(mask, mask, elem);

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));

        vector<vector<Point> >::iterator iter = contours.begin();
        //面积筛选
        //*
        for (; iter != contours.end();) {
            double g_dConArea = contourArea(*iter);
            float contourAreaMax = roi.cols * roi.rows * 0.6;//最大面积为输入图像的面积的0.8
            if (g_dConArea > contourAreaMax || g_dConArea < 800) {

                iter = contours.erase(iter);
            } else {
                ++iter;
            }
        }

        //对面积进行排序，并进行分析
        std::sort(contours.begin(), contours.end(), ContoursCmpl);
        if (contours.size() > 0) {
            Rect boxRect = boundingRect(contours[0]);
            if (!boxRect.empty()) {
                float ratio = float(boxRect.width) / float(boxRect.height);   //求出宽高比
                float areaBox = boxRect.width * boxRect.height;
                float area = contourArea(contours[0]);
                float areaP = area / areaBox;
                float areapBox = areaBox / (roi.rows *
                                            roi.cols);//检测框占roi图像面积的占比；PUBG占屏幕通常小于0.05；start的屏幕占比通常大于0.4；在此默认值宽松设为0.12
                if (areaP <= 1.2 && areaP > 0.6) {
                    //1. PUBG && STB
                    if (ratio > 1.05 && ratio < 3.35) {
                        if (areapBox < 0.16) {
                            int f_w = roi.cols * 0.8;
                            int f_h = roi.rows * 0.4;
                            if (boxRect.x < f_w && (float) boxRect.y < f_h) {
                                game_start = true;
                            }
                        }

                    }
                }
            }
        }
        return game_start;
    } // in_game_icon(Mat &in)

    bool pubg_game_hall(cv::Mat &scene, Rect &out) {
        bool game_over = false;

        cv::Rect rect = Rect(0, 0, scene.cols / 3.2, scene.rows / 6.2);
        if (!roi_valid(scene, rect)) return false;

        Mat hsv;
        Mat roi = scene(rect);
        if (roi.empty())
            return false;

        float img_ratio = 0.4;

        cv::Size d_size = cv::Size(roi.size().width * img_ratio, roi.size().height * img_ratio);
        if (d_size.empty()) return false;

        resize(roi, roi,d_size);
        cvtColor(roi, hsv, COLOR_BGR2HSV);
        Mat mask = cv::Mat(roi.rows, roi.cols, CV_8UC1, cv::Scalar(0));

        double h = 0.0, s = 0.0, v = 0.0;
        for (int i = 0; i < int(roi.rows); i++) {
            for (int j = 0; j < int(roi.cols); j++) {
                h = hsv.at<Vec3b>(i, j)[0];
                s = hsv.at<Vec3b>(i, j)[1];
                v = hsv.at<Vec3b>(i, j)[2];

                if (h > 8 && h <= 53) {
                    if (s >= 130 && s <= 255) {
                        if (v >= 130 && v <= 255) {
                            mask.at<uchar>(i, j) = 255;
                        }
                    }
                }
            }
        }

        Mat elem = getStructuringElement(MORPH_ELLIPSE, Size(8, 8));
        dilate(mask, mask, elem);
        elem = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
        erode(mask, mask, elem);

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE,
                     Point(0, 0));

        vector<vector<Point> >::iterator iter = contours.begin();
        //面积筛选
        //*
        for (; iter != contours.end();) {
            double g_dConArea = contourArea(*iter);
            float contourAreaMax =
                    roi.cols * roi.rows * 0.8;  //最大面积为输入图像的面积的0.8
            if (g_dConArea > contourAreaMax || g_dConArea < 800 * img_ratio) {
                iter = contours.erase(iter);
            } else {
                ++iter;
            }
        }

        //对面积进行排序，并进行分析
        std::sort(contours.begin(), contours.end(), ContoursCmpl);
        if (contours.size() > 0) {
            Rect boxRect = boundingRect(contours[0]);
            if (!boxRect.empty()) {
                float ratio = float(boxRect.width) / float(boxRect.height);  //求出宽高比
                float areaBox = boxRect.width * boxRect.height;
                float area = contourArea(contours[0]);
                float areaP = area / areaBox;
                float areapBox =
                        areaBox /
                        (roi.rows *
                         roi.cols);  //检测框占roi图像面积的占比；PUBG占屏幕通常小于0.05；start的屏幕占比通常大于0.4；在此默认值宽松设为0.12
                if (areaP <= 1.2 && areaP > 0.47) {
                    // 1. PUBG && STB
                    if (areapBox > 0.35 && areapBox < 0.8) {
                        if (ratio > 2.36 && ratio < 3.8) {
                            boxRect.x = boxRect.x * (1.0 / img_ratio);
                            boxRect.y = boxRect.y * (1.0 / img_ratio);
                            boxRect.width = boxRect.width * (1.0 / img_ratio);
                            boxRect.height = boxRect.height * (1.0 / img_ratio);
                            out = boxRect;
                            game_over = true;
                        }
                    }
                    // 2.Fortnite
                    //        if (ratio > 0.8 && ratio < 1.2) {
                    //          if ((float)boxRect.x > roi.cols * 0.75 &&
                    //              (float)boxRect.y < roi.rows * 0.5) {
                    //            out = boxRect;
                    //            game_over = true;
                    //          }
                    //        }
                }
            }
        }
        return game_over;
    } // pubg_game_hall(Mat &in, Rect &out)
}
