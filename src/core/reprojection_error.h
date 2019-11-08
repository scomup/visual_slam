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

        const Eigen::AngleAxis<T> roll_angle(rotation[0],  Eigen::Matrix<T, 3, 1>::UnitX());
        const Eigen::AngleAxis<T> pitch_angle(rotation[1], Eigen::Matrix<T, 3, 1>::UnitY());
        const Eigen::AngleAxis<T> yaw_angle(rotation[2], Eigen::Matrix<T, 3, 1>::UnitZ());
        Eigen::Quaternion<T> q =  yaw_angle * pitch_angle * roll_angle;
        const Eigen::Matrix<T, 3, 1> Pw(point3d[0], point3d[1],point3d[2]);

        const transform::Rigid3<T> Tcw(
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation), q);

        
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
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 2, 3, 3>(new ReprojectionError(information, K)));
    }

private:
    const Eigen::Matrix2f sqrt_information_;
    const Eigen::Matrix3f K_;
}; 


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

        const Eigen::AngleAxis<T> roll_angle0(rotation0[0],  Eigen::Matrix<T, 3, 1>::UnitX());
        const Eigen::AngleAxis<T> pitch_angle0(rotation0[1], Eigen::Matrix<T, 3, 1>::UnitY());
        const Eigen::AngleAxis<T> yaw_angle0(rotation0[2], Eigen::Matrix<T, 3, 1>::UnitZ());
        const Eigen::Quaternion<T> q0 =  yaw_angle0 * pitch_angle0 * roll_angle0;

        const transform::Rigid3<T> Tcw0(
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation0),q0);

        const Eigen::AngleAxis<T> roll_angle1(rotation1[0],  Eigen::Matrix<T, 3, 1>::UnitX());
        const Eigen::AngleAxis<T> pitch_angle1(rotation1[1], Eigen::Matrix<T, 3, 1>::UnitY());
        const Eigen::AngleAxis<T> yaw_angle1(rotation1[2], Eigen::Matrix<T, 3, 1>::UnitZ());
        const Eigen::Quaternion<T> q1 =  yaw_angle1 * pitch_angle1 * roll_angle1;

        const transform::Rigid3<T> Tcw1(
            Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation1),q1);
        const Eigen::Quaternion<T> diff_rotation = Tcw0.rotation() * Tcw1.rotation();
        const Eigen::Matrix<T, 3, 1> diff_translation = Tcw0.translation() - Tcw1.translation();
        const Eigen::Matrix<T, 3, 1> rpy = diff_rotation.toRotationMatrix().eulerAngles(0, 1, 2);

        residual[0] = diff_translation.x() * T(wt_);
        residual[1] = diff_translation.y() * T(wt_);
        residual[2] = diff_translation.z() * T(wt_);
        residual[3] = rpy.x() * T(wr_);
        residual[4] = rpy.y() * T(wr_);
        residual[5] = rpy.z() * T(wr_);

        return true;
    }

    static ceres::CostFunction *Create(const float wt, const float wr)
    {
        return (new ceres::AutoDiffCostFunction<PoseError, 6, 3, 3, 3, 3>(new PoseError(wt, wr)));
    }

private:
    const float wr_;
    const float wt_;
}; 
} // namespace core
} // namespace visual_slam

#endif // CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
