#define OPENCV
#include <yolo_v2_class.hpp>

struct bbox_t_deg {
    unsigned int x, y, w, h;    // (x,y) - top-left corner, (w, h) - width & height of bounded box
    float prob;                    // confidence - probability that the object was found correctly
    unsigned int obj_id;        // class of object - from range [0, classes-1]
    unsigned int track_id;        // tracking id for video (0 - untracked, 1 - inf - tracked object)
    unsigned int frames_counter;// counter of frames on which the object was detected

    int degree;
};

class Detector_deg : public Detector
{
public:
    
    std::vector<bbox_t_deg> detectWithDeg(std::string image_filename, float thresh = 0.2, bool use_mean = false);
	std::vector<bbox_t_deg> detectWithDeg(image_t img, float thresh = 0.2, bool use_mean = false);

protected:

    

}


