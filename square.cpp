// // ******** BLUE SQUARE DETECTION CODE ************* //

// // This program was adapted from a ChatGPT version of a blue square detection program, as well
// // as other open-source OpenCV examples to detect shapes and colours.
// // In particular, ChatGPT was important to address the issue of varying lighting conditions.


// #include <opencv2/opencv.hpp>
// #include "opencv2/core.hpp"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"
// #include <iostream>
// using namespace cv;
// using namespace std;

// int thresh = 50;

// // Calculates cosine of a given angle in the 4 vertices countour

// static double angle( Point pt1, Point pt2, Point pt0 )
// {
//     double dx1 = pt1.x - pt0.x;
//     double dy1 = pt1.y - pt0.y;
//     double dx2 = pt2.x - pt0.x;
//     double dy2 = pt2.y - pt0.y;
//     return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
// }

// int main()
// {

//     //  Mat frame = imread("gazebo.png");

//     // // Check if the image was loaded successfully
//     // if (frame.empty())
//     // {
//     //     cout << "Failed to load the image." << endl;
//     //     return -1;
//     // }


//     // Open the default camera
//     cv::VideoCapture capture("DJI_0442.MP4");

//     // Check if the camera was opened successfully
//     if (!capture.isOpened())
//     {
//         std::cout << "Failed to open the camera." << std::endl;
//         return -1;
//     }

//     // Create a window to display the camera feed
//     cv::namedWindow("Webcam", cv::WINDOW_NORMAL);

//     // Define the minimum number of detections required to consider a contour as a square
//     const int MIN_DETECTIONS = 20;

//     // Variables to track the number of detections and the current detected contour
//     int detections = 0;
//     std::vector<cv::Point> detectedContour;
    

//     while (true)
//     {
        
//         // Capture a frame from the camera
//         cv::Mat frame;
//         capture >> frame;


//         // Declare variables

//         Mat pyr, timg, gray0(frame.size(), CV_8U), gray;

//         // Check if the frame was captured successfully
//         if (frame.empty())
//         {
//             std::cout << "Failed to capture a frame." << std::endl;
//             break;
//         }

//         // Convert the frame to the HSV color space
//         cv::Mat hsvFrame;
//         cv::cvtColor(frame, hsvFrame, cv::COLOR_BGR2HSV);

//         // Define the range of blue color in HSV
//         cv::Scalar lowerBlue(90, 50, 50);
//         cv::Scalar upperBlue(130, 255, 255);

//         // Threshold the frame to obtain only blue pixels using adaptive thresholding
//         cv::Mat blueMask;
//         cv::inRange(hsvFrame, lowerBlue, upperBlue, blueMask);
//         cv::adaptiveThreshold(blueMask, blueMask, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 15, -10);
//         cv::GaussianBlur(blueMask, blueMask, cv::Size(9, 9), 2, 2);

//         // Apply morphological operations to enhance and smooth the mask
//         cv::morphologyEx(blueMask, blueMask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
//         cv::morphologyEx(blueMask, blueMask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

//         // Find contours in the blue mask
//         std::vector<std::vector<cv::Point>> contours;
//         cv::findContours(blueMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//         // Iterate over the contours and filter out non-square contours
//         for (const auto& contour : contours)
//         {
//             double perimeter = cv::arcLength(contour, true);
//             std::vector<cv::Point> approx;
//             cv::approxPolyDP(contour, approx, 0.04 * perimeter, true);

//             // Check if the contour is a square (4 vertices)
//             if (approx.size() == 4)
//             {
//                 float width = std::max(approx[1].x - approx[0].x, approx[2].x - approx[3].x);
//                 float height = std::max(approx[3].y - approx[0].y, approx[2].y - approx[1].y);
//                 float ratio = width / height;
//                 cout << height <<endl;
//                 cout << width <<endl;


//                 // if (ratio >= 0.3 && ratio <= 1.7) // Problems with detecting a square (aspect ration too narrow?)
//                 // {
//                     double area = cv::contourArea(contour);

//                     if(area > 500) {
//                          double maxCosine = 0;
//                         for( int j = 2; j < 5; j++ )
//                         {
//                             // find the maximum cosine of the angle between joint edges
//                             double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
//                             maxCosine = MAX(maxCosine, cosine);
//                         }
//                         // if cosines of all angles are small
//                         // (all angles are ~90 degree) then write quandrange
//                         // vertices to resultant sequence
//                         if( maxCosine < 0.3 ) {
//                          // Increment the detections counter
//                         detections++;

//                         // Store the detected contour if it has been detected enough times
//                         if (detections >= MIN_DETECTIONS)
//                         {
//                             detectedContour = approx;
//                             break;
//                         }
//                         // Draw a bounding rectangle around the square
//                         // cv::Rect boundingRect = cv::boundingRect(approx);
//                         // cv::rectangle(frame, boundingRect, cv::Scalar(0, 255, 255), 3);
//                         // cv::putText(frame, "Square", approx[0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);}
//                         // cv::polylines(frame, approx, true, Scalar(0, 255, 0), 3, LINE_AA);
//                         // Draw a bounding rectangle around the square
//                         // cv::Rect boundingRect = cv::boundingRect(approx);
//                         // cv::rectangle(frame, boundingRect, cv::Scalar(0, 255, 255), 3);
//                         // cv::putText(frame, "Square", approx[0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
//                     }
//                 }
//             }
//         }

//          if (!detectedContour.empty())
//         {
//             cv::Rect boundingRect = cv::boundingRect(detectedContour);
//             cv::rectangle(frame, boundingRect, cv::Scalar(0, 255, 255), 3);
//             cv::putText(frame, "Square", detectedContour[0], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
//         }

//         // Display the frame with detected squares and the blue mask
//         cv::imshow("Webcam", frame);
//         cv::imshow("Bluemask", blueMask);

//         // Exit the loop if the 'Esc' key is pressed
//         if (cv::waitKey(1) == 27)
//             break;
//     }

//     // Release the camera capture and close the windows
//     // capture.release();
//     cv::destroyAllWindows();

//     return 0;
// }

#include <opencv2/opencv.hpp>
#include <iostream>
#include <exiv2/exiv2.hpp>

using namespace cv;
using namespace std;

int thresh = 50;

int NUM_IMAGES = 218;

// Calculates cosine of a given angle in the 4 vertices contour
static double angle(Point pt1, Point pt2, Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void printGPSData(const Exiv2::ExifData& exifData)
{
    // Get the GPS data
    Exiv2::ExifData::const_iterator gpsLatIterator = exifData.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLatitude"));
    Exiv2::ExifData::const_iterator gpsLonIterator = exifData.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLongitude"));
    Exiv2::ExifData::const_iterator gpsLatRefIterator = exifData.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLatitudeRef"));
    Exiv2::ExifData::const_iterator gpsLonRefIterator = exifData.findKey(Exiv2::ExifKey("Exif.GPSInfo.GPSLongitudeRef"));

    bool hasGPSInfo = (gpsLatIterator != exifData.end()) && (gpsLonIterator != exifData.end()) &&
                      (gpsLatRefIterator != exifData.end()) && (gpsLonRefIterator != exifData.end());

    if (hasGPSInfo) {
        Exiv2::Rational gpsLatDegrees = gpsLatIterator->toRational(0);
        Exiv2::Rational gpsLatMinutes = gpsLatIterator->toRational(1);
        Exiv2::Rational gpsLatSeconds = gpsLatIterator->toRational(2);

        Exiv2::Rational gpsLonDegrees = gpsLonIterator->toRational(0);
        Exiv2::Rational gpsLonMinutes = gpsLonIterator->toRational(1);
        Exiv2::Rational gpsLonSeconds = gpsLonIterator->toRational(2);

        std::string gpsLatRef = gpsLatRefIterator->toString();
        std::string gpsLonRef = gpsLonRefIterator->toString();

        double degreesLat = static_cast<double>(gpsLatDegrees.first) + static_cast<double>(gpsLatMinutes.first) / 60.0 + static_cast<double>(gpsLatSeconds.first) / (3600.0 * static_cast<double>(gpsLatSeconds.second));
        double degreesLon = static_cast<double>(gpsLonDegrees.first) + static_cast<double>(gpsLonMinutes.first) / 60.0 + static_cast<double>(gpsLonSeconds.first) / (3600.0 * static_cast<double>(gpsLonSeconds.second));

        if (gpsLonRef == "E") {
            gpsLonRef = "W";
        }

    int precision = 9; // Set the desired number of decimal places
    std::cout << std::fixed << std::setprecision(precision) << "Latitude: " << degreesLat << " (degrees), " << gpsLatRef << std::endl;
    std::cout << std::fixed << std::setprecision(precision) << "Longitude: " << degreesLon << " (degrees), " << gpsLonRef << std::endl;

    } else {
        std::cout << "GPS coordinates not found in the image!" << std::endl;
    }
}

int detect(int num)
{
    // Load the image from file
    std::string filename = "test/image" + std::to_string(num) + ".jpg";
    cv::Mat frame = cv::imread(filename);

    // Check if the image was loaded successfully
    if (frame.empty())
    {
        cout << "Failed to load the image." << endl;
        return -1;
    }

    // Define the minimum number of detections required to consider a contour as a square
    const int MIN_DETECTIONS = 1;

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
    for (const auto &contour : contours)
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
            cout << "Height: " << height << endl;
            cout << "Width: " << width << endl;

            double area = cv::contourArea(contour);

            if (area > 2)
            {
                double maxCosine = 0;
                for (int j = 2; j < 5; j++)
                {
                    // find the maximum cosine of the angle between joint edges
                    double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                    maxCosine = MAX(maxCosine, cosine);
                }
                // if cosines of all angles are small
                // (all angles are ~90 degree) then write quadrangle
                // vertices to resultant sequence
                if (maxCosine < 0.3)
                {
                    // Increment the detections counter
                    detections++;

                    // Store the detected contour if it has been detected enough times
                    if (detections >= MIN_DETECTIONS)
                    {
                        detectedContour = approx;

                        float xSquare = (detectedContour[0].x + detectedContour[1].x + detectedContour[2].x + detectedContour[3].x)/4;
                        float ySquare = (detectedContour[0].y + detectedContour[1].y + detectedContour[2].y + detectedContour[3].y)/4;
                        
                        int xCenter = frame.cols/2; // Get image width
                        int yCenter = frame.rows/2; // Get image height

                        float distanceCenter = sqrt((xCenter - xSquare)*(xCenter - xSquare) + (yCenter - ySquare)*(yCenter - ySquare));

                        float distanceMeter = distanceCenter*6.16/100;
                        std::cout << distanceMeter << std::endl;
                        if(distanceMeter < 20){
                            
                            std::cout << "ok" << std::endl;
                        }
                        
                        try {
                            // Open the image file
                            Exiv2::Image::AutoPtr image = Exiv2::ImageFactory::open(filename);
                            if (!image.get()) {
                                std::cout << "Error: Image not found or unsupported format!" << std::endl;
                                return 0;
                            }

                            // Read the image metadata
                            image->readMetadata();

                            // Get the Exif data
                            Exiv2::ExifData& exifData = image->exifData();
                            if (exifData.empty()) {
                                std::cout << "No Exif data found in the image!" << std::endl;
                                return 0;
                            }

                            // Print GPS data
                            printGPSData(exifData);

                        } catch (const std::exception& ex) {
                            std::cout << "Error: " << ex.what() << std::endl;
                        }

                        break;
                    }

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

    // Display the frame with detected squares and the blue mask
    cv::imshow("Webcam", frame);
    cv::imshow("Bluemask", blueMask);

    // Wait for a key press and then close the windows
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}

int main(){

    for(int i = 0; i < NUM_IMAGES; i++ ){
        detect(i);
    }

    return 0;
}
