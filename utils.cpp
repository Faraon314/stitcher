#include <precomp.h>


cv::Rect resultRoi(const std::vector<cv::Point> &corners, const std::vector<cv::Size> &sizes)
{
    CV_Assert(sizes.size() == corners.size());
    cv::Point tl(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
    cv::Point br(std::numeric_limits<int>::min(), std::numeric_limits<int>::min());
    for (size_t i = 0; i < corners.size(); ++i)
    {
        tl.x = std::min(tl.x, corners[i].x);
        tl.y = std::min(tl.y, corners[i].y);
        br.x = std::max(br.x, corners[i].x + sizes[i].width);
        br.y = std::max(br.y, corners[i].y + sizes[i].height);
    }
    return cv::Rect(tl, br);
}


cv::Rect resultRoi(const std::vector<cv::Point> &corners, const std::vector<cv::Mat> &images)
{
    std::vector<cv::Size> sizes(images.size());
    for (size_t i = 0; i < images.size(); ++i)
        sizes[i] = images[i].size();
    return resultRoi(corners, sizes);
}



