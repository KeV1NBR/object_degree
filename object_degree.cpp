#include<iostream>
#include <opencv2/opencv.hpp>
#define OPENCV
#include <yolo_v2_class.hpp>

#include "object_degree.hpp"

using namespace cv;
using namespace std;

Detector_deg::Detector_deg(string cfg, string weight): Detector(cfg, weight)
{

}

vector<bbox_t_deg> Detector_deg::detectWithDeg(image_t img, float thresh, bool use_mean, int thres1, int thres2, int rho, int theta, int lineThres, int rhoScale, int thetaScale)
{
    vector<bbox_t> darknet_predict;
    vector<bbox_t_deg> predict;
    
    darknet_predict = detect(img, thresh,use_mean);

    int i=0;
    for(auto& item: darknet_predict)
    {
        predict[i].x = darknet_predict[i].x;
        predict[i].y = darknet_predict[i].y;
        predict[i].w = darknet_predict[i].w;
        predict[i].h = darknet_predict[i].h;
        predict[i].prob = darknet_predict[i].prob;
        predict[i].obj_id = darknet_predict[i].obj_id;
        predict[i].track_id = darknet_predict[i].track_id;
        predict[i].frames_counter = darknet_predict[i].frames_counter;
    }

    i=0;
    for(auto& item: darknet_predict)
    {
        Rect bbox(item.x, item.y, item.w, item.h);
        bbox.width = min(640-bbox.x, bbox.width);
        bbox.height = min(480-bbox.y, bbox.height);
        Mat crop = img(bbox).clone();
        Mat gray;
        cvtColor(crop, gray, COLOR_BGR2GRAY);
        Mat canny;
        Mat cannyBGR;
        vector<Vec2f> lines;
        Canny(gray, canny, thres1, thres2);
        HoughLines(canny, lines, (double)rho / rhoScale, (double)theta / thetaScale, lineThres);
        cvtColor(canny, cannyBGR, COLOR_GRAY2BGR);
        drawLines(crop, lines);
        Mat combine = Mat::zeros(crop.rows + 20, crop.cols * 2, CV_8UC3);
        crop.copyTo(combine(Rect(Point(), crop.size())));
        cannyBGR.copyTo(combine(Rect(Point(canny.cols, 0), canny.size())));
        predict[i].degree = calcDeg(lines);
        putText(combine, std::to_string(predict[i].degree), Point(0, crop.rows + 18), 0, 0.5, Scalar(255, 255, 255));

        imshow("Canny" + std::to_string(i), combine);
        i++;
    }
    return predict;
}





double Detector_deg::calcDeg(const std::vector<cv::Vec2f>& lines)
{
    double ret = -999;
    if(lines.size() > 0) 
    {
        ret = 0;
        std::vector<double> degs;
        for(const cv::Vec2f& line: lines) 
        {
            degs.push_back(line[1] * 2.0);
        }
        double s = 0, c = 0;
        for(double& d: degs) 
        {
            s += sin(d);
            c += cos(d);
        }
        s /= degs.size();
        c /= degs.size();
        
        ret = atan2(s, c);
        ret /= 2;
        ret = ret * 180.0 / CV_PI;
    }
    return ret;
}
void Detector_deg::drawLines(cv::Mat &input, const std::vector<cv::Vec2f> &lines)
{
    for(size_t i=0; i<lines.size(); i++)
    {
        float r = lines[i][0];
        float theta = lines[i][1];
        if(theta<CV_PI/4.0 || theta>3*CV_PI/4.0)
        {
            cv::Point pt1(r/cos(theta),0);
            cv::Point pt2((r-input.rows*sin(theta))/cos(theta), input.rows);
            line(input, pt1, pt2, cv::Scalar(255,0,0), 1);
        } 
        else 
        {
            cv::Point pt1(0,r/sin(theta));
            cv::Point pt2(input.cols, (r-input.cols*cos(theta))/sin(theta));
            cv::line(input, pt1, pt2, cv::Scalar(255,0,0), 1);
        }
    }
}

