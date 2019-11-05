

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

        const Eigen::Matrix<T, 3, 1> Pw(point3d[0], point3d[1], point3d[2]);

        const transform::Rigid3<T> Tcw(
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation),
            Eigen::Quaternion<T>(rotation[0], rotation[1], rotation[2],
                                 rotation[3]));
        //const transform::Rigid3<T> Twc = Tcw.inverse();

        const Eigen::Matrix<T, 3, 1> Pc = Tcw * Pw;
        const Eigen::Matrix<T, 3, 1> image_point(Pc.x() / Pc.y(), -Pc.z() / Pc.y(), (T)1);
        const Eigen::Matrix<T, 3, 1> reprojection_point = K_.cast<T>() * image_point;
        T z = reprojection_point.z();

        residual[0] = reprojection_point(0) / z - uv[0];
        residual[1] = reprojection_point(1) / z - uv[1];

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

class PoseError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseError(const float wt,
              const float wr)
        : wt_(wt),
          wr_(wr) {}
    template <typename T>
    bool operator()(const T *const translation0, const T *const rotation0,
                    const T *const translation1, const T *const rotation1,
                    T *const residual) const
    {

        const transform::Rigid3<T> Tcw0(
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation0),
            Eigen::Quaternion<T>(rotation0[0], rotation0[1], rotation0[2],
                                 rotation0[3]));
        const transform::Rigid3<T> Tcw1(
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation1),
            Eigen::Quaternion<T>(rotation1[0], rotation1[1], rotation1[2],
                                 rotation1[3]));
        const Eigen::Quaternion<T> r = Tcw0.rotation() * Tcw1.rotation();

        T diff_translation = (Tcw0.translation() - Tcw1.translation()).norm();
        T diff_rotation = (T)2 * ceres::acos(r.w());

        residual[0] = diff_translation * T(wt_);
        residual[1] = diff_rotation * T(wr_);

        return true;
    }

    static ceres::CostFunction *Create(const float wt, const float wr)
    {
        return (new ceres::AutoDiffCostFunction<PoseError, 2, 3, 4, 3, 4>(new PoseError(wt, wr)));
    }

private:
    const float wr_;
    const float wt_;
}; // namespace core
} // namespace core
} // namespace visual_slam

#endif // CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
