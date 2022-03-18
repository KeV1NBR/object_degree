#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "object_degree.h"
#include "realsense.h"

using namespace std;
using namespace cv;
using namespace rs2;
using namespace realsense;

void drawBoundingBox(cv::Mat image, bbox_t_deg boundingBox);

int main() {
    RealSense rs;
    Detector_deg detector("", "");

    if (rs.depth_supports(RS2_OPTION_MIN_DISTANCE))
        rs.set_depth_option(RS2_OPTION_MIN_DISTANCE, 200);
    rs.set_align_stream(RS2_STREAM_COLOR);
    rs.set_colorizer_option(RS2_OPTION_COLOR_SCHEME, 3);
    rs.enable_hole_filling_filter(true);
    rs.set_hole_filling_filter_option(RS2_OPTION_HOLES_FILL, 1);

    Mat color;
    Mat depth_image;

    while (true) {
        rs.update();
        rs.retrieve_color_image(color);
        rs.retrieve_depth_image(depth_image);

        std::vector<bbox_t_deg> predict =
            detector.detectWithDeg(color, depth_image);

        for (auto p : predict) {
            drawBoundingBox(color, p);
            drawBoundingBox(depth_image, p);
        }

        imshow("color", color);
        imshow("depth", depth_image);

        if (waitKey(1) == 27) break;
    }

    return 0;
}

/** Draw a bounding box onto a Mat, include drawing it's class name. */
void drawBoundingBox(cv::Mat image, bbox_t_deg boundingBox) {
    cv::Rect rect(boundingBox.x, boundingBox.y, boundingBox.w, boundingBox.h);
    // Random select a color of bounding box
    int r = 50 + ((43 * (boundingBox.obj_id + 1)) % 150);
    int g = 50 + ((97 * (boundingBox.obj_id + 1)) % 150);
    int b = 50 + ((37 * (boundingBox.obj_id + 1)) % 150);
    cv::Scalar color(b, g, r);
    cv::rectangle(image, rect, color, 2);
    cv::putText(image, to_string(boundingBox.degree),
                rect.tl() + cv::Point(0, 20), 0, 0.7, color, 2);
}
