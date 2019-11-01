
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>
#include "withrobot_camera.hpp"

#define WIDTH		640
#define HEIGHT		480
#define FPS			30
#define GAIN		100
#define EXPOSURE	150
// If you use openCV 3.x.x , comment out and modify the Makefile Line 34~40
#define OPENCV_4_X_X
#define AE_OFF

using namespace cv;
using namespace std;

enum Viewer { SIDEXSIDE, ANAGLYPH};

static void
StereoView(Mat left, Mat right, Viewer viewType = SIDEXSIDE)
{
    if(viewType == SIDEXSIDE)
    {
        /* Side by Side */
        Mat stereo_image( Size(left.cols*2, left.rows), left.type());

        left.copyTo(stereo_image(Rect(0, 0, left.cols, left.rows)));
        right.copyTo(stereo_image(Rect(left.cols, 0, left.cols, left.rows)));

        imshow("Stereo View", stereo_image);
    }
    else if(viewType == ANAGLYPH)
    {
        namedWindow("Anaglyph", 0);
        /* Anaglyph */
        Mat left_mono(Size(left.cols, left.rows), CV_8UC1);
        Mat right_mono(Size(left.cols, left.rows), CV_8UC1);
        Mat img3d(Size(left.cols, left.rows), CV_8UC3);
        Mat null(Size(left.cols, left.rows), CV_8UC1);;
        null.setTo(127);

#ifdef OPENCV_4_X_X
        cvtColor(left, left_mono, COLOR_BGR2GRAY);
        cvtColor(right, right_mono, COLOR_BGR2GRAY);
#else
        cvtColor(left, left_mono, CV_BGR2GRAY);
        cvtColor(right, right_mono, CV_BGR2GRAY);
#endif
        Mat channels[3] = {left_mono, left_mono, right_mono};

        cv::merge(channels, 3, img3d);
        imshow("Anaglyph", img3d);
    }
}

int main( int argc, char** argv )
{
    bool AE_toggle = false;
    int g =	100;
    int e =	150;
    String toggle = "Off";
    /* camera */
    const char* devPath = "/dev/video0";

    /* Camera Open */
    Withrobot::Camera cap(devPath);
    Withrobot::camera_format camFormat;

    /* Set Camera control */
    cap.set_format(WIDTH, HEIGHT, Withrobot::fourcc_to_pixformat('Y','U','Y','V'), 1, FPS);
    /*
     * get current camera format (image size and frame rate)
     */
    cap.get_current_format(camFormat);
    /*
     * set camera parameter
     */
    cap.set_control("Gain", GAIN);
    cap.set_control("Exposure (Absolute)", EXPOSURE);
    cap.set_control("White Balance Blue Component", 180);
    cap.set_control("White Balance Red Component", 150);
#ifdef AE_OFF
    cap.set_control("Exposure, Auto", 0x1); //AE Off
#else
    cap.set_control("Exposure, Auto", 0x3); //AE On
#endif
    /*
     * Start streaming
     */
    if (!cap.start())
    {
        perror("Failed to start(camera).");
        exit(0);
    }
    else
    {
        cout << "Success to start(camera)" << endl;
        Mat frame(Size(WIDTH, HEIGHT), CV_8UC2);
        Mat stereo_raw[2];
        Mat left_color, right_color;
        for(;;)
        {
            cap.get_frame(frame.data, camFormat.image_size, 1);
            if (frame.empty())
            {
                cerr << "ERROR! blank frame grabbed\n";
                continue;
            }
            split(frame, stereo_raw);
#ifdef OPENCV_4_X_X
            cvtColor(stereo_raw[1], left_color, COLOR_BayerGR2RGB);
            cvtColor(stereo_raw[0], right_color, COLOR_BayerGR2RGB);
#else
            cvtColor(stereo_raw[1], left_color, CV_BayerGR2RGB);
            cvtColor(stereo_raw[0], right_color, CV_BayerGR2RGB);
#endif

            /* stereo view */
            //StereoView(left_color, right_color, SIDEXSIDE);

            /* separate view */
            imshow("left", left_color);
            imshow("right", right_color);

            /* Key Command */
            char key = waitKey(20);
            if (key=='q')
            {
                destroyAllWindows();
                cap.stop();
                printf("Done.\n");
                break;
            }
        }
    }
    destroyAllWindows();
    cap.stop();
    printf("Done.\n");

    return 0;
}
