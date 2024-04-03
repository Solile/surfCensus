#include <chrono>

#include "surf_catchAndmatch.h"
#include "sgm_util.h"

SLAM::SLAM() : width_(0), height_(0), img_left_(nullptr), img_right_(nullptr), is_Initialized_(false){};
SLAM::~SLAM()
{
    Release();
    is_Initialized_ = false;
};

void SLAM::Release()
{
    // 释放内存
    SAFE_DELETE(disp_left_);
};

bool SLAM::Initialize(const sint32 &width, const sint32 &height, const SLAMOption &option)
{
    //... 赋值
    // 影像尺寸
    width_ = width;
    height_ = height;
    // SLAM方式参数
    option_ = option;

    if (width == 0 || height == 0)
    {
        return false;
    }

    //... 开辟内存空间
    const sint32 img_size = width * height;

    // 匹配代价(初始/聚合)
    gray_left_.create(height, width,CV_8UC1);
    gray_right_.create(height, width, CV_8UC1);

    // 视差图
    disp_left_ = new float32[img_size]();

    is_Initialized_ = disp_left_;

    return is_Initialized_;
};

bool SLAM::Reset(const sint32 &width, const sint32 &height, const SLAMOption &option)
{
    // 释放内存
    Release();

    // 重置初始化标记
    is_Initialized_ = false;

    // 初始化
    return Initialize(width, height, option);
};

bool SLAM::Match(const uint8 *img_left, const uint8 *img_right, float32 *disp_left, std::vector<std::pair<int, int>> left_matches)
{
    if (!is_Initialized_)
    {
        return false;
    }
    if (img_left == nullptr || img_right == nullptr)
    {
        return false;
    }

    img_left_ = img_left;
    img_right_ = img_right;

    // 灰度数据（左右影像）
    //cv::cvtColor(*img_left_, gray_left_, cv::COLOR_BGR2GRAY);
    //cv::cvtColor(*img_right_, gray_right_, cv::COLOR_BGR2GRAY);
    ComputeGray();
    //cv::imshow("Disparity Mat Color", gray_left_);

    //cv::waitKey(0);

    if (option_.match_method == orb)
    {
        orb_fearures();
    }
    else if (option_.match_method == surf)
    {
        surf_fearures();
    }
    else
    {
        sift_fearures();
    }
    Matcher_bruteForce();

    // 输出视差图
    memcpy(disp_left, disp_left_, width_ * height_ * sizeof(float32));
    memcpy(left_matches.data(), left_matches_.data(), left_matches_.size());

    return true;
};

void SLAM::ComputeGray()
{
    for (sint32 y = 0; y < height_; y++) {
        for (sint32 x = 0; x < width_; x++) {
            const auto b = img_left_[y * width_ + x];
            const auto g = img_left_[y * width_ + x + 1];
            const auto r = img_left_[y * width_ + x + 2];
            gray_left_.at<uint8>(y, x) = uint8(r * 0.299 + g * 0.587 + b * 0.114);
        }
    }
    for (sint32 y = 0; y < height_; y++) {
        for (sint32 x = 0; x < width_; x++) {
            const auto b = img_right_[y * width_ + x];
            const auto g = img_right_[y * width_ + x + 1];
            const auto r = img_right_[y * width_ + x + 2];
            gray_right_.at<uint8>(y, x) = uint8(r * 0.299 + g * 0.587 + b * 0.114);
        }
    }  
}

void SLAM::orb_fearures()
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2f);
    orb->detect(gray_left_, keypoints1);
    orb->compute(gray_left_, keypoints1, descriptions1);
    orb->detect(gray_right_, keypoints2);
    orb->compute(gray_right_, keypoints2, descriptions2);
};
void SLAM::sift_fearures()
{
    cv::Ptr<cv::SiftFeatureDetector> sift = cv::SiftFeatureDetector::create();
    sift->detect(gray_left_, keypoints1);
    sift->compute(gray_left_, keypoints1, descriptions1);
    sift->detect(gray_right_, keypoints2);
    sift->compute(gray_right_, keypoints2, descriptions2);
};
void SLAM::surf_fearures()
{
    // minHessian的值一般在400~800之间。minHessian是一个阈值，它决定了哪些值是你接受的关键点。minHessian值越高，得到的关键点越少，但是关键点也就更好，反之，minHessian值越低，得到的关键点越多，关键点质量越差。
    int minHessian = 600;
    cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(minHessian);
    surf->detect(gray_left_, keypoints1, cv::Mat());
    surf->compute(gray_left_, keypoints1, descriptions1);
    surf->detect(gray_right_, keypoints2, cv::Mat());
    surf->compute(gray_right_, keypoints2, descriptions2);
};

void SLAM::Matcher_bruteForce()
{
    std::vector<cv::DMatch> matches;                      // 定义存放匹配结果的变量
    cv::BFMatcher matcher(cv::NORM_HAMMING);              // 定义特征点匹配的类，使用汉明距离
    matcher.match(descriptions1, descriptions2, matches); // 进行特征点匹配

    std::ostringstream ss;
    ss << "matches =" << matches.size() << std::endl; // 匹配成功特征点数目

    // 匹配点对筛选
    auto min_max = minmax_element(matches.begin(), matches.end(),
                                  [](const cv::DMatch &m1, const cv::DMatch &m2)
                                  { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    // 输出所有匹配结果中最大韩明距离和最小汉明距离
    ss << "min_dist=" << min_dist << std::endl;
    ss << "max_dist=" << max_dist << std::endl;

    // 将汉明距离较大的匹配点对删除
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance <= std::max(2 * min_dist, 20.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    // KNN 匹配
    std::vector<std::vector<cv::DMatch>> knn_matches;
    const float ratio_thresh = 0.7f;
    std::vector<cv::DMatch> knn_good_matches;
    matcher.knnMatch(descriptions1, descriptions2, knn_matches, 2);
    for (auto& knn_matche : knn_matches)
    {
        if (knn_matche[0].distance < ratio_thresh * knn_matche[1].distance)
        {
            knn_good_matches.push_back(knn_matche[0]);
        }
    }

    std::cout << "good_match =" << good_matches.size() << std::endl; // 剩余特征点数目
    std::cout << "Knn_good_match:" << good_matches.size() << std::endl;

    std::vector<int> left_keypoints(good_matches.size()), right_keypoints(good_matches.size());
    std::vector<float> matchDepth(good_matches.size());
    for (size_t i = 0; i < good_matches.size(); i++)
    {
        left_keypoints[i] = good_matches[i].queryIdx;
        right_keypoints[i] = good_matches[i].trainIdx;
        matchDepth[i] = good_matches[i].distance;
    }

    //std::vector<std::pair<int, int>> left_matches, right_matches;
    auto left_matches = left_matches_;
    auto right_matches = right_matches_;
    for (size_t i = 0; i < good_matches.size(); i++)
    {
        left_matches.emplace_back(keypoints1[left_keypoints[i]].pt.x, keypoints1[left_keypoints[i]].pt.y);
        right_matches.emplace_back(keypoints2[right_keypoints[i]].pt.x, keypoints2[right_keypoints[i]].pt.y);
    }
    sint32 height = height_;
    sint32 width = width_;
    const auto disparity = disp_left_;
    for (sint32 i = 0; i < height; i++)
    {
        for (sint32 j = 0; j < width; j++)
        {
            disparity[i * width + j] = Invalid_Float;
        }
    }
    for (sint32 k = 0; k < good_matches.size(); k++)
    {
        disparity[left_matches[k].first * width + left_matches[k].second] = matchDepth[k];
    }
    //// KNN 匹配
    //std::vector<std::vector<cv::DMatch>> knn_matches;
    //const float ratio_thresh = 0.7f;
    //std::vector<cv::DMatch> knn_good_matches;
    //matcher.knnMatch(descriptions1, descriptions2, knn_matches, 2);
    //for (auto &knn_matche : knn_matches)
    //{
    //    if (knn_matche[0].distance < ratio_thresh * knn_matche[1].distance)
    //    {
    //        knn_good_matches.push_back(knn_matche[0]);
    //    }
    //}

    //std::cout << "Knn匹配上的线条数:" << knn_good_matches.size() << std::endl;
    // match_two_image(img_1, img_2, keypoints_1, keypoints_2, descriptors_1, descriptors_2);

    // 绘制匹配结果
    cv::Mat imgMatches, imgHammingMatch;
    drawMatches(gray_left_, keypoints1, gray_left_, keypoints2, matches, imgMatches);
    drawMatches(gray_right_, keypoints1, gray_right_, keypoints2, good_matches, imgHammingMatch);

    // 显示结果
    cv::imwrite("D:\\dealpic\\orb_outimg.png", imgMatches);              // 未筛选结果
    cv::imwrite("D:\\dealpic\\orb_Hamming_outimg.png", imgHammingMatch); // 最小汉明距离筛选
};

// void Matcher_bruteForce(cv::Mat img1, cv::Mat img2, std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2,
//                         cv::Mat &descriptions1, cv::Mat &descriptions2)
// {
//     std::vector<cv::DMatch> matches;                      // 定义存放匹配结果的变量
//     cv::BFMatcher matcher(cv::NORM_HAMMING);              // 定义特征点匹配的类，使用汉明距离
//     matcher.match(descriptions1, descriptions2, matches); // 进行特征点匹配

//     // cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
//     // matcher->match(descriptions1,descriptions2,matches);//进行特征点匹配
//     std::ostringstream ss;
//     ss << "matches =" << matches.size() << std::endl; // 匹配成功特征点数目

//     // 匹配点对筛选
//     auto min_max = minmax_element(matches.begin(), matches.end(),
//                                   [](const cv::DMatch &m1, const cv::DMatch &m2)
//                                   { return m1.distance < m2.distance; });
//     double min_dist = min_max.first->distance;
//     double max_dist = min_max.second->distance;

//     // 输出所有匹配结果中最大韩明距离和最小汉明距离
//     ss << "min_dist=" << min_dist << std::endl;
//     ss << "max_dist=" << max_dist << std::endl;

//     // 将汉明距离较大的匹配点对删除
//     std::vector<cv::DMatch> good_matches;
//     for (int i = 0; i < matches.size(); i++)
//     {
//         if (matches[i].distance <= std::max(2 * min_dist, 20.0))
//         {
//             good_matches.push_back(matches[i]);
//         }
//     }
//     ss << "good_match =" << good_matches.size() << std::endl; // 剩余特征点数目
//     std::vector<int> left_keypoints(good_matches.size()), right_keypoints(good_matches.size());
//     std::vector<float> matchDepth(good_matches.size());
//     for (size_t i = 0; i < good_matches.size(); i++)
//     {
//         left_keypoints[i] = good_matches[i].queryIdx;
//         right_keypoints[i] = good_matches[i].trainIdx;
//         matchDepth[i] = good_matches[i].distance;
//     }

//     std::vector<std::pair<int, int>> left_matches, right_matches;
//     for (size_t i = 0; i < good_matches.size(); i++)
//     {
//         left_matches.emplace_back(keypoints1[left_keypoints[i]].pt.x, keypoints1[left_keypoints[i]].pt.y);
//         right_matches.emplace_back(keypoints2[right_keypoints[i]].pt.x, keypoints2[right_keypoints[i]].pt.y);
//         // left_matches[i].first = keypoints1[left_keypoints[i]].pt.x;
//         // left_matches[i].second = keypoints1[left_keypoints[i]].pt.y;
//         // right_matches[i].first = keypoints2[right_keypoints[i]].pt.x;
//         // right_matches[i].second = keypoints2[right_keypoints[i]].pt.y;
//     }

//     // KNN 匹配
//     std::vector<std::vector<cv::DMatch>> knn_matches;
//     const float ratio_thresh = 0.7f;
//     std::vector<cv::DMatch> good_matches;
//     matcher.knnMatch(descriptions1, descriptions2, knn_matches, 2);
//     for (auto &knn_matche : knn_matches)
//     {
//         if (knn_matche[0].distance < ratio_thresh * knn_matche[1].distance)
//         {
//             good_matches.push_back(knn_matche[0]);
//         }
//     }

//     std::cout << "Knn匹配上的线条数:" << good_matches.size() << std::endl;
//     // match_two_image(img_1, img_2, keypoints_1, keypoints_2, descriptors_1, descriptors_2);

//     // 绘制匹配结果
//     cv::Mat imgMatches, imgHammingMatch;
//     drawMatches(img1, keypoints1, img2, keypoints2, matches, imgMatches);
//     drawMatches(img1, keypoints1, img2, keypoints2, good_matches, imgHammingMatch);

//     // 显示结果
//     cv::imwrite("D:\\dealpic\\3.5-1\\orb\\orb_outimg.png", imgMatches);              // 未筛选结果
//     cv::imwrite("D:\\dealpic\\3.5-1\\orb\\orb_Hamming_outimg.png", imgHammingMatch); // 最小汉明距离筛选
// };
// void orb_fearures(const cv::Mat &gray, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptions)
// {
//     cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2f);
//     orb->detect(gray, keypoints);
//     orb->compute(gray, keypoints, descriptions);

//     // cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
//     // cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
//     // detector->detect(gray,keypoints);
//     // descriptor->compute(gray,keypoints,descriptions);
// };
// void sift_fearures(const cv::Mat &gray, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptions)
// {
//     cv::Ptr<cv::SiftFeatureDetector> sift = cv::SiftFeatureDetector::create();
//     sift->detect(gray, keypoints);
//     sift->compute(gray, keypoints, descriptions);
// };
// void surf_fearures(const cv::Mat &gray, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptions)
// {
//     // minHessian的值一般在400~800之间。minHessian是一个阈值，它决定了哪些值是你接受的关键点。minHessian值越高，得到的关键点越少，但是关键点也就更好，反之，minHessian值越低，得到的关键点越多，关键点质量越差。
//     int minHessian = 600;
//     cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(minHessian);
//     surf->detect(gray, keypoints, cv::Mat());
//     surf->compute(gray, keypoints, descriptions);
// };

// int main()
// {

//     // 1. 读取图片
//     const cv::Mat image1 = cv::imread("D:\\KindsOfProj\\双目实验\\0314L\\water thing\\3-4.bmp", 0); // Load as grayscale
//     const cv::Mat image2 = cv::imread("D:\\KindsOfProj\\双目实验\\0314L\\water thing\\3-4.bmp", 0); // Load as grayscale
//     assert(image1.data != nullptr && image2.data != nullptr);

//     std::vector<cv::KeyPoint> keypoints1;
//     std::vector<cv::KeyPoint> keypoints2;
//     cv::Mat descriptors1, descriptors2;
//     if (option_.match_method == orb)
//     {
//         orb_fearures(image1, keypoints1, descriptors1);
//         orb_fearures(image2, keypoints2, descriptors2);
//     }
//     else if (match_method == surf)
//     {
//         surf_fearures(image1, keypoints1, descriptors1);
//         surf_fearures(image2, keypoints2, descriptors2);
//     }
//     else
//     {
//         sift_fearures(image1, keypoints1, descriptors1);
//         sift_fearures(image2, keypoints2, descriptors2);
//     }

//     cv::Mat outImg1, outImg2;
//     drawKeypoints(image1, keypoints1, outImg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
//     drawKeypoints(image2, keypoints2, outImg2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

//     Matcher_bruteForce(image1, image2, keypoints1, keypoints2, descriptors1, descriptors2);

//     return 0;
// }