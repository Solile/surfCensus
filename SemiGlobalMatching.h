#pragma once

#include "sgm_types.h"
#include <vector>

/**
 * \brief SemiGlobalMatching类（General implementation of Semi-Global Matching）
*/
class SemiGlobalMatching
{
public:
    SemiGlobalMatching();
    ~SemiGlobalMatching();

    /** \brief Census窗口尺寸类型*/
    enum CensusSize{
        Census5x5 = 0,
        Census9x7 = 1,
        Censusdx2d = 2
    };

    /** \brief SGM参数结构体 */
    struct SGMOption
    {
        uint8       num_paths;          // 聚合路径数
        sint32      min_disparity;      // 最大视差
        sint32      max_disparity;      // 最小视差

        CensusSize  census_size;        // census窗口尺寸

        bool        is_check_unique;    // 是否检查唯一性
        float32     uniqueness_ration;  // 唯一性约束阈值（最小代价-次最小代价）/最小代价 > 阈值 为有效像素

        bool        is_check_lr;        // 是否检查左右一致性
        float32     lrcheck_thres;      // 左右一致性约束阈值

        bool        is_remove_speckles; //是否去除小的连通区
        int         min_speckle_area;   //最小的连通区面积

        bool        is_fill_holes;      // 是否填充视差空洞

        // P1, P2
        // P2 = P2_int / (Ip-Iq)
        sint32      p1;                 // 惩罚项参数P1
        sint32      p2_int;             // 惩罚项参数P2

        SGMOption() : num_paths(8), min_disparity(0), max_disparity(64), census_size(Census5x5),
            is_check_unique(true), uniqueness_ration(0.95f),
            is_check_lr(true), lrcheck_thres(1.0f),
            is_remove_speckles(true), min_speckle_area(20),
            is_fill_holes(true),
            p1(10), p2_int(150) { }
    };

public:
    /**
     * \brief 类的初始化，完成一些内存的预分配，参数的预设置等
     * \param width         输入，核线像对影像宽
     * \param height        输入，核线像对影像高
     * \param option        输入，SemiGlobalMatching参数
     */
    bool Initialize(const sint32 &width, const sint32 &height, const SGMOption &option);

    /**
     * \brief 执行匹配
     * \param img_left      输入，左影像数据指针
     * \param img_right     输入，右影响数据指针
     * \param disp_left     输出，左影像视差图指针，预先分配影响等尺寸的内存空间
     */
    bool Match(const uint8* img_left, const uint8* img_right, float32 *disp_match,std::vector<std::pair<int, int>> left_matches_, float32* disp_left);

    /**
     * \brief 重设 -> 修改影像尺寸
     * \param width         输入，核线像对影像宽
     * \param height        输入，核线像对影像高
     * \param option        输入，SemiGlobalMatching参数
     */
    bool Reset(const sint32 &width, const sint32 &height, const SGMOption &option);

private:
    /** \brief Census变换 */
    void CensusTransform() const;

    /** \brief 代价运算 */
    void ComputeCost() const;

    /** \brief 代价聚合 */
    void CostAggregation() const;

    /** \brief 视差计算 */
    void ComputeDisparity() const;

    /** \brief 核对视差值 */
    void CompareDisparity() const;

    /** \brief 视差计算 */
    void ComputeDisparityRight() const;

    /** \brief 一致性检查 */
    void LRCheck();

    /** \brief 视差图填充 */
    void FillHolesDispMap();

    /** \brief 内存释放	 */
    void Release();

private:
    /** \brief SGM参数 */
    SGMOption option_;

    /** \brief 影像宽 */
    sint32 width_;

    /** \brief 影像高 */
    sint32 height_;

    /** \brief 左影像数据 */
    const uint8 *img_left_;

    /** \brief 右影像数据 */
    const uint8 *img_right_;

    /** \brief 左影像census值 */
    void *census_left_;

    /** \brief 右影响census值 */
    void *census_right_;

    /** \brief 初始匹配代价 */
    uint8 *cost_init_;

    /** \brief 聚合匹配代价 */
    uint16 *cost_aggr_;

    // ↘ ↓ ↙   5  3  7
    // →    ←	 1     2
    // ↗ ↑ ↖   8  4  6
    /** \brief 聚合匹配代价-方向1	*/
    uint8 *cost_aggr_1_;
    /** \brief 聚合匹配代价-方向2	*/
    uint8 *cost_aggr_2_;
    /** \brief 聚合匹配代价-方向3	*/
    uint8 *cost_aggr_3_;
    /** \brief 聚合匹配代价-方向4	*/
    uint8 *cost_aggr_4_;
    /** \brief 聚合匹配代价-方向5	*/
    uint8 *cost_aggr_5_;
    /** \brief 聚合匹配代价-方向6	*/
    uint8 *cost_aggr_6_;
    /** \brief 聚合匹配代价-方向7	*/
    uint8 *cost_aggr_7_;
    /** \brief 聚合匹配代价-方向8	*/
    uint8 *cost_aggr_8_;

    /** \brief 左影像视差图 */
    float32 *disp_left_;

    /** \brief 右影像视差图 */
    float32 *disp_right_;

    /** \brief 是否初始化标志 */
    bool is_Initialized_;

    /** \brief 遮挡区像素集	*/
    std::vector<std::pair<sint32, sint32>> occlusions_;

    /** \brief 误匹配区像素集	*/
    std::vector<std::pair<sint32, sint32>> mismatches_;

    float32 *disp_match_;
    std::vector<std::pair<int, int>> left_matches_;
};