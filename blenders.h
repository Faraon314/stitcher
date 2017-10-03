#ifndef BLENDERS_H
#define BLENDERS_H

#include <precomp.h>


namespace stitching {

class Blender
{
public:
    virtual ~Blender() {}

    enum { NO, FEATHER, MULTI_BAND };
    static cv::Ptr<Blender> createDefault(int type, bool try_gpu = false);

    void prepare(const std::vector<cv::Point> &corners, const std::vector<cv::Size> &Sizes);
    virtual void prepare(cv::Rect dst_roi);
    virtual void feed(const cv::Mat &img, const cv::Mat &mask, cv::Point tl);
    virtual void blend(cv::Mat &dst, cv::Mat &dst_mask);

protected:
    cv::Mat dst_, dst_mask_;
    cv::Rect dst_roi_;
};

class  FeatherBlender : public Blender
{
public:
    FeatherBlender(float sharpness = 0.02f);

    float sharpness() const { return sharpness_; }
    void setSharpness(float val) { sharpness_ = val; }

    void prepare(cv::Rect dst_roi);
    void feed(const cv::Mat &img, const cv::Mat &mask, cv::Point tl);
    void blend(cv::Mat &dst, cv::Mat &dst_mask);

    // Creates weight maps for fixed set of source images by their masks and top-left corners.
    // Final image can be obtained by simple weighting of the source images.
    cv::Rect createWeightMaps(const std::vector<cv::Mat> &masks, const std::vector<cv::Point> &corners,
                          std::vector<cv::Mat> &weight_maps);

private:
    float sharpness_;
    cv::Mat weight_map_;
    cv::Mat dst_weight_map_;
};

inline FeatherBlender::FeatherBlender(float _sharpness) { setSharpness(_sharpness); }


class  MultiBandBlender : public Blender
{
public:
    MultiBandBlender(int try_gpu = false, int num_bands = 5, int weight_type = CV_32F);

    int numBands() const { return actual_num_bands_; }
    void setNumBands(int val) { actual_num_bands_ = val; }

    void prepare(cv::Rect dst_roi);
    void feed(const cv::Mat &img, const cv::Mat &mask, cv::Point tl);
    void blend(cv::Mat &dst, cv::Mat &dst_mask);

private:
    int actual_num_bands_, num_bands_;
    std::vector<cv::Mat> dst_pyr_laplace_;
    std::vector<cv::Mat> dst_band_weights_;
    cv::Rect dst_roi_final_;
    bool can_use_gpu_;
    int weight_type_; //CV_32F or CV_16S
};


//////////////////////////////////////////////////////////////////////////////
// Auxiliary functions

void  normalizeUsingWeightMap(const cv::Mat& weight, cv::Mat& src);

void  createWeightMap(const cv::Mat& mask, float sharpness, cv::Mat& weight);

void  createLaplacePyr(const cv::Mat &img, int num_levels, std::vector<cv::Mat>& pyr);


// Restores source image
void  restoreImageFromLaplacePyr(std::vector<cv::Mat>& pyr);




} // namespace stiching


#endif // BLENDERS_H
