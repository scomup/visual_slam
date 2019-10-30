

#ifndef VISUAL_SLAM_CORE_REPROJECTION_ERROR_
#define VISUAL_SLAM_CORE_REPROJECTION_ERROR_

#include <cmath>
#include "ceres/ceres.h"
#include "Eigen/Core"
#include "src/transform/rigid_transform.h"

namespace visual_slam
{
namespace core
{

class ReprojectionError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ReprojectionError(const Eigen::Matrix2f &information,
                      const Eigen::Matrix3f &K)
        : sqrt_information_(information.llt().matrixL()),
          K_(K) {}
    template <typename T>
    bool operator()(const T *const point3d,
                    const T *const uv,
                    const T *const translation, const T *const rotation,
                    T *const residual) const
    {
        //std::cout<<(double)translation<<std::endl;
        //Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residual);
        //const Eigen::Map<const Eigen::Matrix<T, 3, 1>> p(point3d); 
        
        const Eigen::Matrix<T, 3, 1> Pw(point3d[0], point3d[1],point3d[2]);

        const transform::Rigid3<T> Tcw(
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation),
            Eigen::Quaternion<T>(rotation[0], rotation[1], rotation[2],
                                 rotation[3]));
        //const transform::Rigid3<T> Twc = Tcw.inverse();

        
        const Eigen::Matrix<T, 3, 1> Pc = Tcw * Pw;
        const Eigen::Matrix<T, 3, 1> reprojection_point = K_.cast<T>() * Pc;
        T z = reprojection_point.z();

        residual[0] = reprojection_point(0)/z - uv[0];
        residual[1] = reprojection_point(1)/z - uv[1];
        

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Matrix2f &information, const Eigen::Matrix3f K)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 2, 3, 4>(new ReprojectionError(information, K)));
    }

private:
    const Eigen::Matrix2f sqrt_information_;
    const Eigen::Matrix3f K_;
}; // namespace core

} // namespace core
} // namespace visual_slam

#endif // CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
