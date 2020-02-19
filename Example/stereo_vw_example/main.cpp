#include "main.hpp"

using namespace cv;

int main(int argc, char* argv[]) 
{
    nvxio::Application &app = nvxio::Application::get();
    ovxio::printVersionInfo();
    std::string configFile ="/home/nvidia/twkim/stereo_vw_example/stereo_matching_demo_config.ini";
    StereoMatching::StereoMatchingParams params;
    StereoMatching::ImplementationType implementationType = StereoMatching::HIGH_LEVEL_API;
    // You can choice the StereoMatching Level
    // HIGH_LEVEL_API
    // LOW_LEVEL_API
    // LOW_LEVEL_API_PYRAMIDAL
    
    char* cfg_file = "./darknet/yolov3.cfg";
    char* weight_file = "./darknet/yolov3.weights";
    char* meta_file = "./darknet/coco.data";
    float thresh = 0.5;
    float hier_thresh = 0.5;
    image im;
    detection *dets;

    std::string obj_dist = "";
    cv::Mat left_color, right_color, left_gray, right_gray, map11, map12, map21, map22, l_img, r_img, disp, disp16, disp32, ROI;

    int num = 0;
    int w, h, x, y = 0;
    double min, max = 0.0;
    double _depth = 0.0;

    app.init(argc, argv);
    std::string error;

    if (!read(configFile, params, error))
    {
        std::cerr << error;
        return nvxio::Application::APP_EXIT_CODE_INVALID_VALUE;
    }

    // Load DNN and meta file
    // If you failed, re-build Darknet and replace libdarknet.so file
    metadata meta = get_metadata(meta_file);
    network *net = load_network(cfg_file, weight_file, 0);

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
    cap.set_control("Gain", GAIN);
    cap.set_control("Exposure (Absolute)", EXPOSURE);
    cap.set_control("White Balance Blue Component", 180);
    cap.set_control("White Balance Red Component", 150);

    ovxio::ContextGuard context;
    vxDirective(context, VX_DIRECTIVE_ENABLE_PERFORMANCE);
        
    vx_image vx_left_rect = vxCreateImage(context, WIDTH, HEIGHT, VX_DF_IMAGE_RGB);
    NVXIO_CHECK_REFERENCE(vx_left_rect);
    vx_image vx_right_rect = vxCreateImage(context, WIDTH, HEIGHT, VX_DF_IMAGE_RGB);
    NVXIO_CHECK_REFERENCE(vx_right_rect);
    vx_image disparity_rect = vxCreateImage(context, WIDTH, HEIGHT, VX_DF_IMAGE_U8);
    NVXIO_CHECK_REFERENCE(disparity_rect);
    vx_image disparity_16 = vxCreateImage(context, WIDTH, HEIGHT, VX_DF_IMAGE_S16);
    NVXIO_CHECK_REFERENCE(disparity_16);

    std::unique_ptr<StereoMatching> stereo_rect(
    StereoMatching::createStereoMatching(
        context, params,
        implementationType,
        vx_left_rect, vx_right_rect, disparity_rect, disparity_16));
        
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
        std::cout << "Success to start(camera)" << std::endl;
        cv::Mat frame(cv::Size(WIDTH, HEIGHT), CV_8UC2);
        cv::Mat stereo_raw[2];
        double F = readCalib(map11, map12, map21, map22);
        double total_ms = 0.0;
        std::unique_ptr<nvxio::SyncTimer> syncTimer = nvxio::createSyncTimer();
        syncTimer->arm(1. / app.getFPSLimit());
        nvx::Timer totalTimer;
        totalTimer.tic();
        
        for(;;)
        {
            cap.get_frame(frame.data, camFormat.image_size, 1);
            if (frame.empty())
            {
                std::cout << "ERROR! blank frame grabbed" << std::endl;
                continue;
            }

            cv::split(frame, stereo_raw);
            // opencv : RGB, visionworks : BGR
            // cv::cvtColor(stereo_raw[1], left_color, CV_BayerGR2BGR);
            // cv::cvtColor(stereo_raw[0], right_color, CV_BayerGR2BGR);
            cv::cvtColor(stereo_raw[1], left_color, CV_BayerGR2RGB);
            cv::cvtColor(stereo_raw[0], right_color, CV_BayerGR2RGB);

            cv::remap(left_color, l_img, map11, map12, cv::INTER_LINEAR);
            cv::remap(right_color, r_img, map21, map22, cv::INTER_LINEAR);

            vx_image vx_left = nvx_cv::createVXImageFromCVMat(context, l_img);
            vx_image vx_right = nvx_cv::createVXImageFromCVMat(context, r_img);
            nvxuCopyImage(context,vx_left,vx_left_rect);
            nvxuCopyImage(context,vx_right,vx_right_rect);

            stereo_rect->run();

            disp16 = copyVxImageToCvMat(disparity_16);
            disp16.convertTo(disp32, CV_32FC1, 1./16);
         
            im = mat_to_image(l_img);
            network_predict_image(net, im);
            dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, NULL, 0, &num);
            do_nms_obj(dets, num, 80, thresh);
                        
            for(int i =0; i <num; i++)
            {
                for(int j =0; j <80; j++)
                {
                    // We concerned about person
                    if(std::string(meta.names[j]) == "person")
                    {
                        if (dets[i].prob[j] > 0)
                        {
                            w = dets[i].bbox.w;
                            h = dets[i].bbox.h;
                            x = dets[i].bbox.x;
                            y = dets[i].bbox.y;
                            cv::Rect rect(x,y,w*0.1,h*0.1);
                            ROI = disp32(rect);
                            cv::minMaxLoc(ROI, &min, &max);
                            _depth = roundf(0.12 * F / max * 100) / 100;
                            obj_dist = Withrobot::to_string<double>(_depth)+'m';

                            if(_depth < 2)
                            {
                                cv::rectangle(l_img, cv::Point(x-w/2, y-h/2), cv::Point(x+w/2, y+h/2), cv::Scalar(0,0,255), 2, 8, 0);
                                cv::putText(l_img, obj_dist, cv::Point(x-w/2, y-h/2), 1, 1, cv::Scalar(0,0,255), 2);
                            }
                            else if((_depth < 4) && (_depth > 2))
                            {
                                cv::rectangle(l_img, cv::Point(x-w/2, y-h/2), cv::Point(x+w/2, y+h/2), cv::Scalar(0,255,0), 2, 8, 0);
                                cv::putText(l_img, obj_dist, cv::Point(x-w/2, y-h/2), 1, 1, cv::Scalar(0,255,0), 2);
                            }
                            else if (_depth > 4)
                            {
                                cv::rectangle(l_img, cv::Point(x-w/2, y-h/2), cv::Point(x+w/2, y+h/2), cv::Scalar(255,0,0), 2, 8, 0);
                                cv::putText(l_img, obj_dist, cv::Point(x-w/2, y-h/2), 1, 1, cv::Scalar(255,0,0), 2);
                            }
                        }
                    }
                }
            }
            total_ms = totalTimer.toc();
            std::cout << "Display Time : " << total_ms << " ms" << std::endl << std::endl;
            syncTimer->synchronize();

            cv::imshow("disp", l_img);
            cv::waitKey(10);

            free_image(im);
            free_detections(dets, 0);
            vxReleaseImage(&vx_left);
            vxReleaseImage(&vx_right);
            
            total_ms = totalTimer.toc();
            totalTimer.tic();
        }
    }
    free_network(net);
    vxReleaseImage(&vx_left_rect);
    vxReleaseImage(&vx_right_rect);
    vxReleaseImage(&disparity_rect);
    cv::destroyAllWindows();
    cap.stop();
    printf("Done.\n");

    return nvxio::Application::APP_EXIT_CODE_SUCCESS;
}