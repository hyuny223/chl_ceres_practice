#pragma once

#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include "opencv2/opencv.hpp"

struct SnavelyReprojectionError
{
    SnavelyReprojectionError(double observed_x, double observed_y)
        : observed_x(observed_x), observed_y(observed_y) {}

    template <typename T>
    bool operator()(const T *const camera,
                    const T *const point,
                    T *residuals) const
    {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);
        // camera[3,4,5] are the translation.
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // Apply second and fourth order radial distortion.
        const T &l1 = camera[7];
        const T &l2 = camera[8];
        T r2 = xp * xp + yp * yp;
        T distortion = 1.0 + r2 * (l1 + l2 * r2);

        // Compute final projected point position.
        const T &focal = camera[6];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const double observed_x,
                                       const double observed_y)
    {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
            new SnavelyReprojectionError(observed_x, observed_y)));
    }

    double observed_x;
    double observed_y;
};

auto optimization(const cv::Mat &r_mat, const cv::Mat &t_mat, const std::vector<cv::Point2d> &point2, const std::vector<cv::Point3d> &point_3d)
{
    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;
    using ceres::Problem;
    using ceres::Solve;
    using ceres::Solver;

    cv::Mat rodrigues;
    cv::Rodrigues(r_mat, rodrigues);

    double *camera = new double[9];

    camera[0] = rodrigues.ptr<double>(0)[0];
    camera[1] = rodrigues.ptr<double>(1)[0];
    camera[2] = rodrigues.ptr<double>(2)[0];
    camera[3] = t_mat.ptr<double>(0)[0];
    camera[4] = t_mat.ptr<double>(1)[0];
    camera[5] = t_mat.ptr<double>(2)[0];
    camera[6] = 718.856;
    camera[7] = 0;
    camera[8] = 0;

    double *point = new double[3];
    ceres::Problem problem;
    for (int i = 0; i < point2.size(); ++i)
    {
        point[0] = point_3d[i].x;
        point[1] = point_3d[i].y;
        point[2] = point_3d[i].z;
        ceres::CostFunction *cost_function =
            SnavelyReprojectionError::Create(
                point2[i].x,
                point2[i].y);
        problem.AddResidualBlock(cost_function,
                                 new ceres::CauchyLoss(1.0),
                                 camera,
                                 point);
    }

    ceres::Solver::Options options;
    // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = 100;
    options.update_state_every_iteration = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout.precision(20);
    std::cout << summary.BriefReport() << "\n";

    cv::Mat_<double> new_r(3, 3), new_t(3, 1), new_rod(3, 1);
    new_rod << camera[0], camera[1], camera[2];
    new_t << camera[3], camera[4], camera[5];
    cv::Rodrigues(new_rod, new_r);

    return std::tuple{new_r, new_t};
}
