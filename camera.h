#ifndef CAMERA_H
#define CAMERA_H

#include <precomp.h>
namespace stitching {

struct Camera
{
    Camera();
    Camera(const Camera& other);
    const Camera& operator =(const Camera& other);

    //return matrix of intrinsics
    cv::Mat K() const;

    cv::Mat Intrisics;//matrix of intrinsics
    void calcK();

    double focal; // Focal length
    double aspect; // Aspect ratio
    double ppx; // Principal point X
    double ppy; // Principal point Y
    double cx;
    double cy;
    double tangentialX;
    double tangentialY;


    cv::Mat Q;// quaternion (t,x,y,z)
    cv::Mat R; // Rotation
    //return rotation using quaternion
    cv::Mat Rotation();
    //calc rotation using quaternion
    void calcR();

    cv::Mat t; // Translation
};

}// namespace stiching



#endif // CAMERA_H
