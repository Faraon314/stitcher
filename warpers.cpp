#include <warpers.h>
#include <iostream>

using namespace std;

namespace stitching {

void SphericalWarper::prepareMap(cv::Size2i src_size)
{
  dst_roi = buildMaps(src_size,xmap, ymap);
}

cv::Point SphericalWarper::warp(const cv::Mat &src,int interp_mode, int border_mode,cv::Mat &dst)
{

    dst_roi = buildMaps(src.size(),xmap, ymap);

    dst.create(dst_roi.height + 1, dst_roi.width + 1, src.type());
    cv::remap(src, dst, xmap, ymap, interp_mode, border_mode);
    //cv::resize(src,dst,cv::Size(dst_roi.height + 1, dst_roi.width + 1));
    return dst_roi.tl();
}

cv::Rect SphericalWarper::buildMaps(cv::Size src_size,cv::Mat &xmap, cv::Mat &ymap)
{
    cv::Point dst_tl, dst_br;
    detectResultRoi(src_size, dst_tl, dst_br);

    xmap.create(dst_br.y - dst_tl.y + 1, dst_br.x - dst_tl.x + 1, CV_32F);
    ymap.create(dst_br.y - dst_tl.y + 1, dst_br.x - dst_tl.x + 1, CV_32F);

    float x, y;
    for (int v = dst_tl.y; v <= dst_br.y; ++v)
    {
        for (int u = dst_tl.x; u <= dst_br.x; ++u)
        {
            projector_.mapBackward(static_cast<float>(u),static_cast<float>(v), x, y);
            xmap.at<float>(v - dst_tl.y, u - dst_tl.x) = x;
            ymap.at<float>(v - dst_tl.y, u - dst_tl.x) = y;
        }
    }

    return cv::Rect(dst_tl, dst_br);
}


void SphericalWarper::detectResultRoi(cv::Size src_size, cv::Point &dst_tl, cv::Point &dst_br)
{
    float tl_uf = std::numeric_limits<float>::max();
    float tl_vf = std::numeric_limits<float>::max();
    float br_uf = -std::numeric_limits<float>::max();
    float br_vf = -std::numeric_limits<float>::max();

    float u, v;
    for (int y = 0; y < src_size.height; ++y)
    {
        for (int x = 0; x < src_size.width; ++x)
        {
            projector_.mapForward(static_cast<float>(x), static_cast<float>(y), u, v);
            tl_uf = std::min(tl_uf, u); tl_vf = std::min(tl_vf, v);
            br_uf = std::max(br_uf, u); br_vf = std::max(br_vf, v);
        }
    }

    projector_.mapForward(static_cast<float>(0), static_cast<float>(0), u, v);
    projector_.mapForward(static_cast<float>(src_size.height-1), static_cast<float>(src_size.width-1), u, v);
    dst_tl.x = static_cast<int>(tl_uf);
    dst_tl.y = static_cast<int>(tl_vf);
    dst_br.x = static_cast<int>(br_uf);
    dst_br.y = static_cast<int>(br_vf);

}

inline
void SphericalProjector::mapForward(float x, float y, float &u, float &v)
{
    float x_ = r_kinv[0] * x + r_kinv[1] * y + r_kinv[2];
    float y_ = r_kinv[3] * x + r_kinv[4] * y + r_kinv[5];
    float z_ = r_kinv[6] * x + r_kinv[7] * y + r_kinv[8];

    u = scale * atan2f(y_, x_);
    v = scale * z_ / sqrtf(x_ * x_ + z_ * z_);

}


inline
void SphericalProjector::mapBackward(float u, float v, float &x, float &y)
{
    u /= scale;
    v /= scale;


    float x_ = cosf(u);
    float y_ = sinf(u);
    float z_ = v;

    float z;
    x = k_rinv[0] * x_ + k_rinv[1] * y_ + k_rinv[2] * z_;
    y = k_rinv[3] * x_ + k_rinv[4] * y_ + k_rinv[5] * z_;
    z = k_rinv[6] * x_ + k_rinv[7] * y_ + k_rinv[8] * z_;

    if (z > 0) { x /= z; y /= z; }
    else x = y = -1;
}


void SphericalProjector::setCameraParams(const cv::Mat &K, const cv::Mat &R, const cv::Mat &T)
{

    cv::Mat_<float> K_(K);
    k[0] = K_(0,0); k[1] = K_(0,1); k[2] = K_(0,2);
    k[3] = K_(1,0); k[4] = K_(1,1); k[5] = K_(1,2);
    k[6] = K_(2,0); k[7] = K_(2,1); k[8] = K_(2,2);

    cv::Mat_<float> Rinv = R.t();
    rinv[0] = Rinv(0,0); rinv[1] = Rinv(0,1); rinv[2] = Rinv(0,2);
    rinv[3] = Rinv(1,0); rinv[4] = Rinv(1,1); rinv[5] = Rinv(1,2);
    rinv[6] = Rinv(2,0); rinv[7] = Rinv(2,1); rinv[8] = Rinv(2,2);

    cv::Mat_<float> R_Kinv = R * K.inv();
    r_kinv[0] = R_Kinv(0,0); r_kinv[1] = R_Kinv(0,1); r_kinv[2] = R_Kinv(0,2);
    r_kinv[3] = R_Kinv(1,0); r_kinv[4] = R_Kinv(1,1); r_kinv[5] = R_Kinv(1,2);
    r_kinv[6] = R_Kinv(2,0); r_kinv[7] = R_Kinv(2,1); r_kinv[8] = R_Kinv(2,2);

    cv::Mat_<float> K_Rinv = K * Rinv;
    k_rinv[0] = K_Rinv(0,0); k_rinv[1] = K_Rinv(0,1); k_rinv[2] = K_Rinv(0,2);
    k_rinv[3] = K_Rinv(1,0); k_rinv[4] = K_Rinv(1,1); k_rinv[5] = K_Rinv(1,2);
    k_rinv[6] = K_Rinv(2,0); k_rinv[7] = K_Rinv(2,1); k_rinv[8] = K_Rinv(2,2);

    cv::Mat_<float> T_(T.reshape(0, 3));
    t[0] = T_(0,0); t[1] = T_(1,0); t[2] = T_(2,0);


//    cv::Mat_<float> Rinv_float;
//    Rinv.convertTo(Rinv_float, CV_64F);

//    cv::Mat_<float> Rinv_t = Rinv_float * T_;
//    rinv_t[0]=Rinv_t(0,0); rinv_t[1]=Rinv_t(1,0); rinv_t[2]=Rinv_t(2,0);

//    k_t[0]=0
//            +k[0]*t[0]+k[1]*t[1]+k[2]*t[2]
//            ;
//    k_t[1]=0
//            +k[3]*t[0]+k[4]*t[1]+k[5]*t[2]
//            ;
//    k_t[0]=0
//            +k[6]*t[0]+k[7]*t[1]+k[8]*t[2]
//            ;


//    cv::Mat_<float> K_Rinv_t=K_Rinv * T_;
//    k_rinv_t[0]=K_Rinv_t(0,0); k_rinv_t[1]=K_Rinv_t(1,0); k_rinv_t[2]=K_Rinv_t(2,0);

  //  std::cout << d_t[0]<<' '<< d_t[1] <<' '<< d_t[2]<< clock << std::endl;

}

} // namespace stitching

