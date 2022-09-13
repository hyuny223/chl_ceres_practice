#pragma once

#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "opencv2/opencv.hpp"

double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};

struct SnavelyReprojectionError
{
    SnavelyReprojectionError(double observed_x, double observed_y)
        : observed_x(observed_x), observed_y(observed_y) {}

    template <typename T>
    bool operator()(const T *const R,
                    const T *const t,
                    const T *const p,
                    T *residuals) const
    {
        T origin_x = T(p[0]);
        T origin_y = T(p[1]);
        T origin_z = T(p[2]);
        T cam_x = R[0] * origin_x + R[1] * origin_y + R[2] * origin_z + t[0];
        T cam_y = R[3] * origin_x + R[4] * origin_y + R[5] * origin_z + t[1];
        T cam_z = R[6] * origin_x + R[7] * origin_y + R[8] * origin_z + t[2];
        T proj_x = (cam_x * k[0] + cam_z * k[2]) / cam_z;
        T proj_y = (cam_y * k[4] + cam_z * k[5]) / cam_z;

        T diff_x = proj_x - T(observed_x);
        T diff_y = proj_y - T(observed_y);

        residuals[0] = diff_x * diff_x;
        residuals[1] = diff_y * diff_y;

        return true;
    }

    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y)
    {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 1, 9, 3, 3>(
            new SnavelyReprojectionError(observed_x, observed_y)));
    }

    double observed_x;
    double observed_y;
};

auto optimization(const cv::Mat &r_mat,
                  const cv::Mat &t_mat,
                  const std::vector<cv::Point2d> &point_2d,
                  const std::vector<cv::Point3d> &point_3d)
{
    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;
    using ceres::Problem;
    using ceres::Solve;
    using ceres::Solver;

    double *R = new double[9];
    double *t = new double[3];
    double *p = new double[3];

    R[0] = r_mat.ptr<double>(0)[0];
    R[1] = r_mat.ptr<double>(0)[1];
    R[2] = r_mat.ptr<double>(0)[2];
    R[3] = r_mat.ptr<double>(1)[0];
    R[4] = r_mat.ptr<double>(1)[1];
    R[5] = r_mat.ptr<double>(1)[2];
    R[6] = r_mat.ptr<double>(2)[0];
    R[7] = r_mat.ptr<double>(2)[1];
    R[8] = r_mat.ptr<double>(2)[2];

    t[0] = t_mat.ptr<double>(0)[0];
    t[1] = t_mat.ptr<double>(1)[0];
    t[3] = t_mat.ptr<double>(2)[0];

    ceres::Problem problem;
    for (int i = 0; i < point_2d.size(); ++i)
    {
        p[0] = point_3d.at(i).x;
        p[1] = point_3d.at(i).y;
        p[2] = point_3d.at(i).z;
        ceres::CostFunction *cost_function =
            SnavelyReprojectionError::Create(
                point_2d.at(i).x,
                point_2d.at(i).y);
        problem.AddResidualBlock(cost_function,
                                 new ceres::CauchyLoss(0.01),
                                 R,
                                 t,
                                 p);
    }

    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = 100;
    options.update_state_every_iteration = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    cv::Mat_<double> new_r(3, 3), new_t(3, 1);
    new_r << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
    new_t << t[0], t[1], t[2];

    return std::tuple{new_r, new_t};
}
