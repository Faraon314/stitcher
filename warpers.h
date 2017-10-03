#ifndef WARPERS_H
#define WARPERS_H


#include <precomp.h>

namespace stitching {

struct  SphericalProjector
{
    void setCameraParams(const cv::Mat &K = cv::Mat::eye(3, 3, CV_32F),
                             const cv::Mat &R = cv::Mat::eye(3, 3, CV_32F),
                             const cv::Mat &T = cv::Mat::zeros(3, 1, CV_32F));

    float scale;
    float k[9];
    float rinv[9];
    float r_kinv[9];
    float t[3];

    float k_rinv[9];

    float d_t[3];
    float rinv_t[3];

    float k_rinv_t[3];
    float k_t[3];

    cv::Point3d center_;
    void mapForward(float x, float y, float &u, float &v);
    void mapBackward(float u, float v, float &x, float &y);

};


class  SphericalWarper
{
public:

    SphericalProjector projector_;
    SphericalWarper(float scale)
    {
        projector_.scale = scale;
        projector_.center_=cv::Point3d(0,0,0);
    }
    SphericalWarper(float scale,cv::Point3d X)
    {
        projector_.scale = scale;
        projector_.center_=X;
    }
    SphericalWarper(float scale,cv::Point3d X,const cv::Mat &K,
                    const cv::Mat &R, const cv::Mat &T)
    {
        projector_.scale = scale;
        projector_.center_=X;
        projector_.setCameraParams(K,R,T);

    }
    cv::Point warp(const cv::Mat &src,int interp_mode, int border_mode,
                   cv::Mat &dst);
    cv::Rect buildMaps(cv::Size src_size, cv::Mat &xmap, cv::Mat &ymap);

    void prepareMap(cv::Size2i  src_size);

protected:

    void detectResultRoi(cv::Size src_size, cv::Point &dst_tl, cv::Point &dst_br);
    void detectResultRoiByBorder(cv::Size src_size, cv::Point &dst_tl, cv::Point &dst_br);
    cv::Mat xmap, ymap;
    cv::Rect dst_roi;
};


} // namespace stitching


#endif // WARPERS_H
