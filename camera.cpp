
#include <camera.h>
namespace stitching{

Camera::Camera():focal(1), aspect(1), ppx(0), ppy(0),cx(0),cy(0),
    tangentialX(0),tangentialY(0),
    R(cv::Mat::eye(3, 3, CV_64F)),
    t(cv::Mat::zeros(3, 1, CV_64F)),Q(cv::Mat::zeros(4, 1, CV_64F)),
    Intrisics(cv::Mat::zeros(3, 3, CV_32F)){}

Camera::Camera(const Camera &other) { *this = other; }

const Camera& Camera::operator =(const Camera &other)
{
    focal = other.focal;
    ppx = other.ppx;
    ppy = other.ppy;
    cx = other.cx;
    cy = other.cy;
    tangentialX = other.tangentialX;
    tangentialY = other.tangentialY;

    aspect = other.aspect;

    Q =other.Q.clone();
    R = other.R.clone();
    t = other.t.clone();
    return *this;
}

cv::Mat Camera::K() const
{
    cv::Mat_<float> k = cv::Mat::eye(3, 3, CV_64F);
    k(0,0) = cx; k(0,2) = ppx;
    k(1,1) = cy; k(1,2) = ppy;
    return k;

}

cv::Mat Camera::Rotation()
{

    double t=+Q.at<double>(0,0);
    double x=+Q.at<double>(1,0);
    double y=+Q.at<double>(2,0);
    double z=+Q.at<double>(3,0);


    ////////////////////////////
    /// 1-2*(y^2+z^2)           2*(x*y-z*t)            2*(x*z+y*t)
    ///     2*(x*y+z*t)         1-2*(x^2+z^2)          2*(y*z-x*t)
    ///    2*(x*z-y*t)           2*(y*z+x*t)          1-2*(x^2+y^2)
    ///
    /// /////////////////////////
    cv::Mat_<double> R=cv::Mat::zeros(3,3,CV_64F);
    R(0,0)=1-2*(y*y+z*z);  R(0,1)=2*(x*y-z*t);      R(0,2)=2*(x*z+y*t);
    R(1,0)=2*(x*y+z*t);    R(1,1)=1-2*(x*x+z*z);    R(1,2)=2*(y*z-x*t);
    R(2,0)=2*(x*z-y*t);    R(2,1)=2*(y*z+x*t);      R(2,2)=1-2*(x*x+y*y);
    return R;
}

void Camera::calcR()
{
    R=Rotation().clone();
}

 void Camera::calcK()
 {
    cv::Mat R;
    K().convertTo(R, CV_32F);
    Intrisics=R;
 }

} // namespace stiching

