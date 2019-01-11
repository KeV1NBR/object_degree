#include <iostream>
#include <opencv2/opencv.hpp>
#define OPENCV
#include <yolo_v2_class.hpp>

struct bbox_t_deg {
    unsigned int x, y, w, h;    // (x,y) - top-left corner, (w, h) - width & height of bounded box
    float prob;                    // confidence - probability that the object was found correctly
    unsigned int obj_id;        // class of object - from range [0, classes-1]
    unsigned int track_id;        // tracking id for video (0 - untracked, 1 - inf - tracked object)
    unsigned int frames_counter;// counter of frames on which the object was detected

    double degree;
};

class Detector_deg : public Detector
{
public:
    Detector_deg(std::string cfg, std::string weight);
    ~Detector_deg();
    
	std::vector<bbox_t_deg> detectWithDeg(cv::Mat img, float thresh, bool use_mean, int thres1, int thres2, int rho, int theta, int lineThres, int rhoScale, int thetaScale);


protected:

    double calcDeg(const std::vector<cv::Vec2f>& lines);
    void drawLines(cv::Mat &input, const std::vector<cv::Vec2f> &lines);


};


