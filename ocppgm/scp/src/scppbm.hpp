#include <cstdint>
#include <vector>
#include <Eigen/Dense>
#include <concepts>

namespace E = Eigen;
using uint8 = std::uint8_t;
using uint16 = std::uint16_t;

/**
 * @brief SCP 参数配置
 * @details
 * 所有成员变量都使用静态内存分配，无需动态分配。
 */
class ScpParas{
public:
    // forbid any constructor: refenrece copy or remove
    ScpParas(const ScpParas&) = delete;
    ScpParas& operator=(const ScpParas&) = delete;
    ScpParas(ScpParas&&) = delete;
    ScpParas& operator=(ScpParas&&) = delete;

    ScpParas();
    ~ScpParas();

    /**
     * @brief 时间网格节点数 (Number of temporal grid nodes)
     */
    uint16_t N = 10;
    /**
     * @brief 子区间积分数量 (Number of subinterval integration)
     */
    uint16_t Nsub = 10;
    /**
     * @brief SCP 外循环最大迭代次数 (Maximum outer loop number)
     */
    uint16_t itrScpMax = 30;
    /**
     * @brief 凸规划内循环最大迭代次数 (Maximum internal loop number)
     */
    uint16_t itrPgmMax = 50;
    /**
     * @brief 指定默认求解器类型, 0 LSOCP, 1 QSOCP (Specify default solver type)
     */
    uint16_t PgmType = 0;

    double epsl_abs = 1e-2; // 绝对收敛容差 (Absolute convergence tolerance)
    double epsl_rel = 3e-3; // 相对收敛容差 (Relative convergence tolerance)
    double feas_tol = 1e-2; // 动力学可行性绝对容差 (Dynamic feasibility absolute tolerance)
    double dftmin = 1e-2;   // 用于更新信赖域权重的最小缺陷值 (The minimum defect used to update trust region weights)
    double q_exit = 1e-2;   // 停止标准的范数 (Stopping criterion norm)
};


template<typename TrjPbmType>
class SCPPbm{
public:
    // forbid any constructor: refenrece copy or remove
    SCPPbm(const SCPPbm&) = delete;
    SCPPbm& operator=(const SCPPbm&) = delete;
    SCPPbm(SCPPbm&&) = delete;
    SCPPbm& operator=(SCPPbm&&) = delete;

    // 构造函数需要接收上游问题的引用或指针
    explicit SCPPbm(const TrjPbm& trjpbm);
    ~SCPPbm();

    // downstream friend class

    // public data members
    ScpParas scpprs = ScpParas();
    // problem-specific parameters: timeNodes-grid
    E::VectorXd tNodes = E::VectorXd::LinSpaced(scpprs.N, 0.0, 1.0);

    /**
     * @brief The standard structure for PTR's parsed discrete OPC
     */
    // Updated reference Trajectory
    std::vector<E::VectorXd> xref = std::vector<E::VectorXd>(scpprs.N, E::VectorXd::Zero(6));
    std::vector<E::VectorXd> uref = std::vector<E::VectorXd>(scpprs.N, E::VectorXd::Zero(3));

};

template<typename TrjPbmType, typename ScpPbmType>
class IdcsLnrConePgm {

};

template<typename TrjPbmType, typename ScpPbmType>
class IdcsLnrConePgm {

};


template <typename TrjPbmType, typename ScpPbmType>
class SubPbm {
public:

};