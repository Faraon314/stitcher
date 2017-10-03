#ifndef STITCHER_H
#define STITCHER_H

#include <precomp.h>
#include <warpercreators.h>
#include <camera.h>
#include <blenders.h>

using namespace std;

namespace stitching {

class  Stitcher
{
public:
    enum { ORIG_RESOL = -1 };
    enum Status { OK, ERR_NEED_MORE_IMGS };

    // Creates stitcher with default parameters
    static Stitcher createDefault(int blenderType);

    void setCameras(const std::vector<Camera> cameras);
    void setSizes(const vector<cv::Mat>& images);
    void estimateWarpedImageScale();
    void estimateCenter();
    void calcWarps();


    cv::Ptr<SphericalWarperCreator> warper() { return warperCreator_; }
        const cv::Ptr<SphericalWarperCreator> warper() const { return warperCreator_; }
        void setWarper(cv::Ptr<SphericalWarperCreator> warper) { warperCreator_ = warper; }

    cv::Ptr<Blender> blender() { return blender_; }
    const cv::Ptr<Blender> blender() const { return blender_; }
    void setBlender(cv::Ptr<Blender> b) { blender_ = b; }

    Status composePanorama(cv::InputArray images,cv::OutputArray pano);

    vector<Camera> cameras() const { return cameras_; }
    cv::Point3d center()const {return center_; }

private:
    Stitcher() {}
    vector<cv::Size2i> sizes_;
    cv::Ptr<SphericalWarperCreator> warperCreator_;
    vector< cv::Ptr<SphericalWarper>> warpers_;

    cv::Ptr<Blender> blender_;
    vector<Camera> cameras_;
    double warped_image_scale_;
    cv::Point3d center_;
};
}// namespace stiching

#endif // STITCHER_H
