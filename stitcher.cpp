#include <stitcher.h>
#include <line.h>


#include <iomanip>
#include <ctime>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>

namespace stitching {

void Stitcher::setSizes(const vector<cv::Mat>&   images)
{
    for (size_t i = 0; i < images.size(); ++i)
    {
        sizes_[i].height=images[i].rows;
        sizes_[i].width=images[i].cols;
    }

}

void Stitcher::setCameras(const vector<Camera> cameras)
{

    std::clock_t c_start = std::clock();

    cameras_.resize(cameras.size());
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras_[i].R = R;

        //Mat t;
        //cameras[i].t.convertTo(t, CV_32F);
        cameras_[i].t=cameras[i].t.clone();

        Mat Q;
        cameras[i].Q.convertTo(Q, CV_64F);
        cameras_[i].Q = Q;

        cameras_[i].focal=cameras[i].focal;
        cameras_[i].aspect=cameras[i].aspect;
        cameras_[i].ppx=cameras[i].ppx;
        cameras_[i].ppy=cameras[i].ppy;
        cameras_[i].cx=cameras[i].cx;
        cameras_[i].cy=cameras[i].cy;


    }

    std::clock_t c_end = std::clock();
    ulong clock=c_end-c_start;

    std::cout << "setCameras " << clock << std::endl;

}

void Stitcher::estimateWarpedImageScale()
{
    std::clock_t c_start = std::clock();

    //vector<double> focals;
    double max_focal=0;
    for (size_t i = 0; i < cameras_.size(); ++i)
    {
        if(max_focal<cameras_[i].focal) {
            max_focal=cameras_[i].focal;
        }

    }
    warped_image_scale_ = max_focal;
  //  warped_image_scale_ = 500;
    std::clock_t c_end = std::clock();
    ulong clock=c_end-c_start;

    std::cout << "estimateWarpedImageScale " << clock << std::endl;
}

void Stitcher::estimateCenter()
{
    std::clock_t c_start = std::clock();

    vector<line3> lines;
    for(size_t i=0; i<cameras_.size();i++)
    {
       lines.push_back(line3(cameras_[i].Q,cameras_[i].t));
    }
//    cv::Point3d x= averagePoint(lines);
//    center_=x;

    cv::Point2d x=averagePoint(commonPoints(lines));
    center_.x=x.x;
    center_.y=x.y;
    center_.z=0;

    std::clock_t c_end = std::clock();
    ulong clock=c_end-c_start;

    std::cout << "estimateCenter " << clock << std::endl;
}

void Stitcher::calcWarps()
{
    std::clock_t c_start = std::clock();
    warpers_.resize(cameras_.size());
    for(size_t i=0;i<warpers_.size();i++)
    {

    std::cout << i << std::endl;

    Mat K;
    cameras_[i].K().convertTo(K, CV_32F);
    Mat T;
    cameras_[i].t.convertTo(T, CV_32F);
    warpers_[i]= warperCreator_->create(float(warped_image_scale_),Point3d(0,0,0),K,cameras_[i].R,cameras_[i].t);

  // warpers_[i]->prepareMap(cv::Size2i(sizes_[i].height,sizes_[i].width));



   }

    std::clock_t c_end = std::clock();
    ulong clock=c_end-c_start;

    std::cout << "calcWarps " << clock << std::endl;
}

Stitcher Stitcher::createDefault(int blenderType)
{
    Stitcher stitcher;
    stitcher.setBlender(Blender::createDefault(blenderType));
    stitcher.setWarper(new SphericalWarperCreator());
    return stitcher;
}

Stitcher::Status Stitcher::composePanorama(InputArray images, OutputArray pano)
{
    vector<Mat> imgs;
    images.getMatVector(imgs);

    Mat &pano_ = pano.getMatRef();

    vector<Point> corners(imgs.size());
    vector<Mat> masks_warped(imgs.size());
    vector<Size> sizes(imgs.size());
    vector<Mat> masks(imgs.size());
    vector<Mat> images_warped(imgs.size());

    // Prepare image masks
    for (size_t i = 0; i < imgs.size(); ++i)
    {
        masks[i].create(imgs[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }


    std::clock_t c_start;
    std::clock_t c_end ;
    ulong clock;

    for (size_t i = 0; i < imgs.size(); ++i)
    {
        c_start = std::clock();
        corners[i] = warpers_[i]->warp(imgs[i],INTER_LINEAR,BORDER_REFLECT, images_warped[i]);
        sizes[i] = images_warped[i].size();


        warpers_[i]->warp(masks[i], INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
        c_end = std::clock();

        clock=c_end-c_start;
        std::cout << "warp: "<< i <<" image "<< clock << std::endl;
    }

//     corners[1].y=-1450;
//     corners[1].y-=30;
//      corners[1].x+=30;
//     corners[2].y-=30;
//     corners[5].y=-1530;
//     corners[4].y=-1450;

    Mat  img_warped_s;

    c_start = std::clock();

    blender_->prepare(corners, sizes);

    c_end = std::clock();
    clock=c_end-c_start;
    std::cout << "blend prepare: " << clock << std::endl;

    for (size_t img_idx = 0; img_idx < imgs.size(); ++img_idx)
    {
        c_start = std::clock();

        images_warped[img_idx].convertTo(img_warped_s, CV_16S);
        blender_->feed(img_warped_s, masks_warped[img_idx], corners[img_idx]);

        c_end = std::clock();

        clock=c_end-c_start;
        std::cout << "blend feed : "<< img_idx <<" image "<< clock << std::endl;
    }

   Mat result, result_mask;

    c_start = std::clock();
    blender_->blend(result, result_mask);
    c_end = std::clock();
    clock=c_end-c_start;

    std::cout << "blend->blend: " << clock << std::endl;

    // Preliminary result is in CV_16SC3 format, but all values are in [0,255] range,
    // so convert it to avoid user confusing
    result.convertTo(pano_, CV_8U);

    return OK;
}

}// namespace stiching
