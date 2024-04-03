#include "SemiGlobalMatching.h"
#include "surf_catchAndmatch.h"
#include <opencv2/opencv.hpp>
#include <iostream>

/**
 * \brief
 * \param argv 3
 * \param argc argc[1]:左影像路径 argc[2]:右影像路径 argc[3]:视差图路径
 * \return
 */
int main(int argv, char **argc)
{
    // if (argv < 3)
    // {
    //     std::cout << "Too few parameters, please specify at least left and right image paths..." << std::endl;
    //     // std::cout <<"参数过少，请至少指定左右影像路径！！" << std::endl;
    //     return -1;
    // }

    //... 读取影像
	//std::string path_left = "D://KindsOfProj//database//cones//imL.png";
	//std::string path_right = "D://KindsOfProj//database//cones//imR.png";
    std::string path_left = "D://C_project//proj_1//Project1//L.jpg";
    std::string path_right = "D://C_project//proj_1//Project1//R.jpg";

    cv::Mat img_left_color = cv::imread(path_left, cv::IMREAD_COLOR);
    cv::Mat img_left = cv::imread(path_left, cv::IMREAD_GRAYSCALE);
    cv::Mat img_right = cv::imread(path_right, cv::IMREAD_GRAYSCALE);

    if (img_left.data == nullptr || img_right.data == nullptr)
    {
        // std::cout << "读取影像失败！！" << std::endl;
        std::cerr << "Error for imread the images..." << std::endl;
    }
    if (img_left.rows != img_right.rows || img_left.cols != img_right.cols)
    {
        // std::cout << "左右影像尺寸不一致！" << std::endl;
        std::cout << "The left image is not the same size as the right one..." << std::endl;
        return -1;
    }

    // ··· SGM匹配
    const sint32 width = static_cast<uint32>(img_left.cols);
    const sint32 height = static_cast<uint32>(img_right.rows);

    SLAM::SLAMOption slamOption;
    slamOption.match_method = SLAM::orb;

    SemiGlobalMatching::SGMOption sgm_option;
    //聚合路径数
    sgm_option.num_paths = 8;
    //候选视差范围
    sgm_option.min_disparity = 0;
    sgm_option.max_disparity = 64;
    // census窗口类型
    sgm_option.census_size == SemiGlobalMatching::Census5x5;
    // 一致性检查
    sgm_option.is_check_lr = true;
    sgm_option.lrcheck_thres = 1.0f;
    // 唯一性约束
    sgm_option.is_check_unique = true;
    sgm_option.uniqueness_ration = 0.99f;
    // 剔除小连通区
    sgm_option.is_remove_speckles = true;
    sgm_option.min_speckle_area = 0;
    //惩罚值P1、P2
    sgm_option.p1 = 40;
    sgm_option.p2_int = 80;
    //视差图填充（科研填充/工程甭填充）
    sgm_option.is_fill_holes = false;

    SLAM pikaQ;
    std::vector<std::pair<int, int>> left_matches;

    if (!pikaQ.Initialize(width, height, slamOption))
    {
        // std::cout << "SGM初始化失败!" << std::endl;
        std::cerr << "Error for SGM Initialization..." << std::endl;
        return -2;
    }
    auto disparity_match = new float32[uint32(width * height)]();

    if (!pikaQ.Match(img_left.data, img_right.data, disparity_match, left_matches))
    {
        // std::cout << "SGM匹配失败!" << std::endl;
        std::cerr << "Error for SGM Matching..." << std::endl;
        return -2;
    }

    SemiGlobalMatching sgm;

    // 初始化
    if (!sgm.Initialize(width, height, sgm_option))
    {
        // std::cout << "SGM初始化失败!" << std::endl;
        std::cerr << "Error for SGM Initialization..." << std::endl;
        return -2;
    }

    // 匹配
    auto disparity = new float32[uint32(width * height)]();
    if (!sgm.Match(img_left.data, img_right.data, disparity_match, left_matches, disparity))
    {
        // std::cout << "SGM匹配失败!" << std::endl;
        std::cerr << "Error for SGM Matching..." << std::endl;
        return -2;
    }

    // 显示视差图
    cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
    float32 min_disp = width;
    float32 max_disp = -width;
    for (sint32 i = 0; i < height; i++)
    {
        for (sint32 j = 0; j < width; j++)
        {
            const float32 disp = disparity[i * width + j];
            if (disp != Invalid_Float)
            {
                min_disp = std::min(min_disp,disp);
                max_disp = std::max(max_disp,disp);
            }
        }
    }
    for (uint32 i = 0; i < height; i++)
    {
        for (uint32 j = 0; j < width; j++)
        {
            const float32 disp = disparity[i * width + j];
            if (disp == Invalid_Float)
            {
                disp_mat.data[i * width + j] = 0;
            }
            else
            {
                disp_mat.data[i * width + j] = 2 * static_cast<uchar>(disp);
                // disp_mat.data[i * width + j] = 2 * static_cast<uchar>((disp-min_disp)/(max_disp - min_disp) * 255);
            }
        }
    }
    // try
    //{
    //     cv::imwrite(argc[3], disp_mat);
    // }
    // catch(cv::Exception&e){
    //     std::cerr<<e.what()<<std::endl;        //cerr ios错误输出流
    // }
    cv::namedWindow("Disparity Mat", cv::WINDOW_NORMAL);
    cv::imshow("Disparity Mat", disp_mat);
    cv::Mat disp_color;
    applyColorMap(disp_mat,disp_color,cv::COLORMAP_JET);
    cv::namedWindow("Disparity Mat Color", cv::WINDOW_NORMAL);
    cv::imshow("Disparity Mat Color", disp_color);

    std::string disp_map_path = "D://KindsOfProj//database//cones//imL.png"; disp_map_path += ".d.png";
    std::string disp_color_map_path = "D://KindsOfProj//database//cones//imL.png"; disp_color_map_path += ".c.png";
    cv::imwrite(disp_map_path, disp_mat);
    cv::imwrite(disp_color_map_path, disp_color);

    
    cv::waitKey(0);

    delete[] disparity;
    disparity = nullptr;

    return 0;
}