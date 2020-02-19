#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>

#include <VX/vx.h>
#include <NVX/nvx_timer.hpp>
#include "OVX/FrameSourceOVX.hpp"
#include "NVX/nvx_opencv_interop.hpp"
#include "OVX/RenderOVX.hpp"
#include "NVX/Application.hpp"
#include "OVX/UtilityOVX.hpp"
#include "NVX/ConfigParser.hpp"
#include <NVX/SyncTimer.hpp>

#include "stereo_matching.hpp"
#include "color_disparity_graph.hpp"

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "withrobot_camera.hpp"
#include "darknet.h"
#include <string.h>
#include <math.h>

#define WIDTH		640
#define HEIGHT		480
#define FPS			30
#define GAIN		150
#define EXPOSURE	150

using namespace cv;

image ipl_to_image(IplImage* src)
{
    int h = src->height;
    int w = src->width;
    int c = src->nChannels;
    image im = make_image(w, h, c);
    unsigned char *data = (unsigned char *)src->imageData;
    int step = src->widthStep;
    int i, j, k;

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){
                im.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
            }
        }
    }
    return im;
}

image mat_to_image(Mat m)
{
    IplImage ipl = m;
    image im = ipl_to_image(&ipl);
    rgbgr_image(im);
    return im;
}

static bool read(const std::string &nf, StereoMatching::StereoMatchingParams &config, std::string &message)
{
    std::unique_ptr<nvxio::ConfigParser> parser(nvxio::createConfigParser());
    
    parser->addParameter("min_disparity",
                         nvxio::OptionHandler::integer(
                             &config.min_disparity,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("max_disparity",
                         nvxio::OptionHandler::integer(
                             &config.max_disparity,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("P1",
                         nvxio::OptionHandler::integer(
                             &config.P1,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("P2",
                         nvxio::OptionHandler::integer(
                             &config.P2,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("sad",
                         nvxio::OptionHandler::integer(
                             &config.sad,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(31)));
    parser->addParameter("bt_clip_value",
                         nvxio::OptionHandler::integer(
                             &config.bt_clip_value,
                             nvxio::ranges::atLeast(15) & nvxio::ranges::atMost(95)));
    parser->addParameter("max_diff",
                         nvxio::OptionHandler::integer(
                             &config.max_diff));
    parser->addParameter("uniqueness_ratio",
                         nvxio::OptionHandler::integer(
                             &config.uniqueness_ratio,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(100)));
    parser->addParameter("scanlines_mask",
                         nvxio::OptionHandler::integer(
                             &config.scanlines_mask,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("flags",
                         nvxio::OptionHandler::integer(
                             &config.flags,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(3)));
    parser->addParameter("ct_win_size",
                         nvxio::OptionHandler::integer(
                             &config.ct_win_size,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(5)));
    parser->addParameter("hc_win_size",
                         nvxio::OptionHandler::integer(
                             &config.hc_win_size,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(5)));

    message = parser->parse(nf);
    return message.empty();
}



double readCalib(cv::Mat& _map11, cv::Mat& _map12, cv::Mat& _map21, cv::Mat& _map22)
{
    std::string left_filename = "./calib/left.yaml";
    std::string right_filename = "./calib/right.yaml";
    cv::FileStorage left_fs(left_filename, cv::FileStorage::READ);
    cv::FileStorage right_fs(right_filename, cv::FileStorage::READ);
    std::vector<float> vec;

    vec.clear();
    left_fs["camera_matrix"]["data"] >> vec;
    cv::Mat left_C(vec,true);
    left_C = left_C.reshape(0,3);
    vec.clear();
    left_fs["distortion_coefficients"]["data"] >> vec;
    cv::Mat left_D(vec,true);
    left_D = left_D.reshape(0,1);
    vec.clear();
    left_fs["rectification_matrix"]["data"] >> vec;
    cv::Mat left_R(vec,true);
    left_R = left_R.reshape(0,3);
    vec.clear();
    left_fs["projection_matrix"]["data"] >> vec;
    cv::Mat left_P(vec,true);
    left_P = left_P.reshape(0,3);
    left_C.convertTo(left_C,CV_64F);
    left_D.convertTo(left_D,CV_64F);
    left_R.convertTo(left_R,CV_64F);
    left_P.convertTo(left_P,CV_64F);
    cv::initUndistortRectifyMap(left_C, left_D, left_R, left_P, cv::Size(WIDTH, HEIGHT), CV_32FC1, _map11, _map12);

    vec.clear();
    right_fs["camera_matrix"]["data"] >> vec;
    cv::Mat right_C(vec,true);
    right_C = right_C.reshape(0,3);
    vec.clear();
    right_fs["distortion_coefficients"]["data"] >> vec;
    cv::Mat right_D(vec,true);
    right_D = right_D.reshape(0,1);
    vec.clear();
    right_fs["rectification_matrix"]["data"] >> vec;
    cv::Mat right_R(vec,true);
    right_R = right_R.reshape(0,3);
    vec.clear();
    right_fs["projection_matrix"]["data"] >> vec;
    cv::Mat right_P(vec,true);
    right_P = right_P.reshape(0,3);
    right_C.convertTo(right_C,CV_64F);
    right_D.convertTo(right_D,CV_64F);
    right_R.convertTo(right_R,CV_64F);
    right_P.convertTo(right_P,CV_64F);
    cv::initUndistortRectifyMap(right_C, right_D, right_R, right_P, cv::Size(WIDTH, HEIGHT), CV_32FC1, _map21, _map22);
    vec.clear();
    left_fs.release();
    right_fs.release();
    return left_P.at<double>(0,0);
}

void VX2MatC1(vx_context m_vxCtx, vx_image& vxiSrc, Mat& matDst)
{
	vx_imagepatch_addressing_t dst_addr;
	dst_addr.dim_x = matDst.cols;
	dst_addr.dim_y = matDst.rows;
	dst_addr.stride_x = sizeof(vx_uint8);
	dst_addr.stride_y = matDst.step;
	void* dst_ptrs[] = {matDst.data};
	vx_image vxiDst = vxCreateImageFromHandle(m_vxCtx, VX_DF_IMAGE_U8, &dst_addr, dst_ptrs, VX_IMPORT_TYPE_HOST);

	vx_status status = nvxuCopyImage(m_vxCtx, vxiSrc, vxiDst);

	vxReleaseImage(&vxiDst);
}

cv::Mat copyVxImageToCvMat(vx_image ovxImage)
{
    vx_status status;
    vx_df_image df_image = 0;
    vx_uint32 width, height;
    status = vxQueryImage(ovxImage, VX_IMAGE_FORMAT, &df_image, sizeof(vx_df_image));
    if (status != VX_SUCCESS)
        throw std::runtime_error("Failed to query image");
    status = vxQueryImage(ovxImage, VX_IMAGE_WIDTH, &width, sizeof(vx_uint32));
    if (status != VX_SUCCESS)
        throw std::runtime_error("Failed to query image");
    status = vxQueryImage(ovxImage, VX_IMAGE_HEIGHT, &height, sizeof(vx_uint32));
    if (status != VX_SUCCESS)
        throw std::runtime_error("Failed to query image");

    if (!(width > 0 && height > 0)) throw std::runtime_error("Invalid format");

    int depth;
    switch (df_image)
    {
    case VX_DF_IMAGE_U8:
        depth = CV_8U;
        break;
    case VX_DF_IMAGE_U16:
        depth = CV_16U;
        break;
    case VX_DF_IMAGE_S16:
        depth = CV_16S;
        break;
    case VX_DF_IMAGE_S32:
        depth = CV_32S;
        break;
    default:
        throw std::runtime_error("Invalid format");
        break;
    }

    cv::Mat image(height, width, CV_MAKE_TYPE(depth, 1));

    vx_rectangle_t rect;
    rect.start_x = rect.start_y = 0;
    rect.end_x = width; rect.end_y = height;

    vx_imagepatch_addressing_t addr;
    addr.dim_x = width;
    addr.dim_y = height;
    addr.stride_x = (vx_uint32)image.elemSize();
    addr.stride_y = (vx_uint32)image.step.p[0];
    vx_uint8* matData = image.data;

    status = vxCopyImagePatch(ovxImage, &rect, 0, &addr, matData, VX_READ_ONLY, VX_MEMORY_TYPE_HOST);
    if (status != VX_SUCCESS)
        throw std::runtime_error("Failed to copy image patch");
    return image;
}