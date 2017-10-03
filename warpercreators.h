#ifndef WARPERCREATORS_H
#define WARPERCREATORS_H

#include <precomp.h>
#include <warpers.h>

namespace stitching
{


class SphericalWarperCreator
{
public:
    cv::Ptr<SphericalWarper> create(float scale) const {
        return new  SphericalWarper(scale); }
    cv::Ptr<SphericalWarper> create(float scale,cv::Point3d X) const {
        return new SphericalWarper(scale,X); }
    cv::Ptr<SphericalWarper> create(float scale,cv::Point3d X,const cv::Mat &K,
                                    const cv::Mat &R, const cv::Mat &T) const {
        return new SphericalWarper(scale,X,K,R,T); }

};


} // namespace stiching


#endif // WARPERCREATOR_H
