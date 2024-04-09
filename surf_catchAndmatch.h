#include "sgm_types.h"
#include <vector>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

class SLAM
{
public:
    SLAM();
    ~SLAM();

    enum MatchMethod
    {
        orb = 0,
        surf = 1,
        sift = 2
    };
    struct SLAMOption
    {
        MatchMethod match_method;
    };

public:
    bool Initialize(const sint32 &width, const sint32 &height, const SLAMOption &option);

    bool Match(const uint8 *img_left, const uint8 *img_right, float32 *disp_left, std::vector<std::pair<int, int>> left_matches);

    bool Reset(const sint32 &width, const sint32 &height, const SLAMOption &option);
private:
    void orb_fearures();

    void surf_fearures();

    void sift_fearures();

    void Matcher_bruteForce();

    void Release();

    void ComputeGray();

private:
    /** \brief SLAM特征点匹配参数 */
    SLAMOption option_;

    /** \brief 影像宽 */
    sint32 width_;

    /** \brief 影像高 */
    sint32 height_;

    /** \brief 左影像数据 */
    const uint8 *img_left_;

    /** \brief 右影像数据 */
    const uint8 *img_right_;

    cv::Mat gray_left_;
    cv::Mat gray_right_;

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;

    cv::Mat descriptions1;
    cv::Mat descriptions2;

    std::vector<std::pair<int, int>> left_matches_;
    std::vector<std::pair<int, int>> right_matches_;

    /** \brief 左影像视差图 */
    float32 *disp_left_;

    /** \brief 是否初始化标志 */
    bool is_Initialized_;
};