// ******** BLUE SQUARE DETECTION CODE ************* //

// This program was adapted from a ChatGPT version of a blue square detection program, as well
// as other open-source OpenCV examples to detect shapes and colours.
// In particular, ChatGPT was important to address the issue of varying lighting conditions.

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int thresh = 50;

// Cossine of the countour
static double angle(Point pt1, Point pt2, Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        // Convert the ROS message to something we can use with OpenCV
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Define the minimum number of detections required to consider a contour as a square
        const int MIN_DETECTIONS = 10;

        // Variables to track the number of detections and the current detected contour
        int detections = 0;
        std::vector<cv::Point> detectedContour;

        // Convert the frame to the HSV color space
        cv::Mat hsvFrame;
        cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

        // Define the range of blue color in HSV
        cv::Scalar lowerBlue(90, 50, 50);
        cv::Scalar upperBlue(130, 255, 255);

        // Threshold the frame to obtain only blue pixels using adaptive thresholding
        cv::Mat blueMask;
        cv::inRange(hsvFrame, lowerBlue, upperBlue, blueMask);
        cv::adaptiveThreshold(blueMask, blueMask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, -10);
        cv::GaussianBlur(blueMask, blueMask, cv::Size(9, 9), 2, 2);

        // Apply morphological operations to enhance and smooth the mask
        cv::morphologyEx(blueMask, blueMask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
        cv::morphologyEx(blueMask, blueMask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

        // Find contours in the blue mask
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(blueMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Iterate over the contours and filter out non-square contours
        for (const auto& contour : contours)
        {
            double perimeter = cv::arcLength(contour, true);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, 0.04 * perimeter, true);

            // Check if the contour is a square (4 vertices)
            if (approx.size() == 4)
            {
                float width = std::max(approx[1].x - approx[0].x, approx[2].x - approx[3].x);
                float height = std::max(approx[3].y - approx[0].y, approx[2].y - approx[1].y);
                float ratio = width / height;
                cout << height << endl;
                cout << width << endl;

                double area = cv::contourArea(contour);

                if (area > 500)
                {
                    double maxCosine = 0;
                    for (int j = 2; j < 5; j++)
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if (maxCosine < 0.3)
                    {
                        detections++;
                    }

                    if (detections >= MIN_DETECTIONS)
                        {
                            detectedContour = approx;
                            break;
                        }
                }
            }
        }

        if (!detectedContour.empty())
        {
            cv::Rect boundingRect = cv::boundingRect(detectedContour);
            cv::rectangle(frame, boundingRect, cv::Scalar(0, 255, 255), 3);
            cv::putText(frame, "Square", detectedContour[0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        }

        // Display the frame with detected squares
        cv::imshow("Webcam", frame);
        cv::imshow("Bluemask", blueMask);

        // Exit the loop if the 'Esc' key is pressed
        if (cv::waitKey(1) == 27)
            ros::shutdown();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "blue_square_detection_node");
    ros::NodeHandle nh;

    // Subscribe to the ROS topic with the image callback function
    ros::Subscriber sub = nh.subscribe("/webcam/image_raw", 1, imageCallback);

    // Create a window to display the camera feed
    cv::namedWindow("Webcam", cv::WINDOW_NORMAL);

    // Start the ROS main loop
    ros::spin();

    // Close the OpenCV windows
    cv::destroyAllWindows();

    return 0;
}
