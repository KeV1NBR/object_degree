#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "opencv2/imgproc.hpp"
#define OPENCV
#include <yolo_v2_class.hpp>

#include "object_degree.h"

using namespace cv;
using namespace std;

bool areaComp(const vector<Point>& lhs, const vector<Point>& rhs) {
    return (contourArea(lhs) < contourArea(rhs)) ? true : false;
}

double degree2Rad(double degree) { return degree * M_PI / 180; }
double rad2Degree(double rad) { return rad * 180 / M_PI; };

Detector_deg::Detector_deg(string cfg, string weight) : Detector(cfg, weight) {}

Detector_deg::~Detector_deg() {}

std::vector<bbox_t_deg> Detector_deg::detectWithDeg(const Mat& color,
                                                    const Mat& depth) {
    vector<bbox_t> darknet_predict;
    vector<bbox_t_deg> predict;

    darknet_predict = detect(color, 0.5);

    unsigned int i = 0;
    for (i = 0; i < predict.size(); i++) {
        predict[i].x = darknet_predict[i].x;
        predict[i].y = darknet_predict[i].y;
        predict[i].w = darknet_predict[i].w;
        predict[i].h = darknet_predict[i].h;
        predict[i].prob = darknet_predict[i].prob;
        predict[i].obj_id = darknet_predict[i].obj_id;
        predict[i].track_id = darknet_predict[i].track_id;
        predict[i].frames_counter = darknet_predict[i].frames_counter;
    }

    for (auto& item : predict) {
        Rect bbox(item.x, item.y, item.w, item.h);
        Mat crop = depth(bbox).clone();
        item.degree = calcDeg(crop);
    }
    return predict;
}

double Detector_deg::calcDeg(Mat& crop) {
    // Convert image to grayscale
    Mat gray;
    cvtColor(crop, gray, COLOR_BGR2GRAY);
    // Convert image to binary
    Mat bw;
    threshold(gray, bw, 50, 255, THRESH_BINARY | THRESH_OTSU);
    // Find all the contours in the thresholded image
    vector<vector<Point> > contours;
    findContours(bw, contours, RETR_LIST, CHAIN_APPROX_NONE);

    vector<Point> pts;

    for (auto c : contours) pts = areaComp(pts, c) ? c : pts;

    // Construct a buffer used by the pca analysis
    int sz = static_cast<int>(pts.size());
    Mat data_pts = Mat(sz, 2, CV_64F);

    for (int i = 0; i < data_pts.rows; i++) {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }

    // Perform PCA analysis
    PCA pca_analysis(data_pts, Mat(), PCA::DATA_AS_ROW);

    // Store the eigenvalues and eigenvectors
    vector<Point2d> eigen_vecs(2);
    vector<double> eigen_val(2);

    for (int i = 0; i < 2; i++) {
        eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                pca_analysis.eigenvectors.at<double>(i, 1));
        eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
    }

    return rad2Degree(
        atan2(eigen_vecs[0].y, eigen_vecs[0].x));  // orientation in radians;
}
