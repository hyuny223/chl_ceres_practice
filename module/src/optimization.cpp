#include <iostream>

#include "ceres/ceres.h"
#include "optimization.hpp"

auto optimization(const cv::Mat& r_mat, const cv::Mat& t_mat, const std::vector<cv::Point2d>& point2, const std::vector<cv::Point3d>& point_3d)
{
    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;
    using ceres::Problem;
    using ceres::Solver;
    using ceres::Solve;

    cv::Mat rodrigues;
    cv::Rodrigues(r_mat, rodrigues);

    double* camera = new double[9];

    camera[0] = rodrigues.ptr<double>(0)[0];
    camera[1] = rodrigues.ptr<double>(1)[0];
    camera[2] = rodrigues.ptr<double>(2)[0];
    camera[3] = t_mat.ptr<double>(0)[0];
    camera[4] = t_mat.ptr<double>(1)[0];
    camera[5] = t_mat.ptr<double>(2)[0];
    camera[6] = 718.856;
    camera[7] = 0;
    camera[8] = 0;

    double* point = new double[3];
    ceres::Problem problem;
    for (int i = 0; i < point2.size(); ++i)
    {
        point[0] = point_3d[i].x;
        point[1] = point_3d[i].y;
        point[2] = point_3d[i].z;
        ceres::CostFunction* cost_function =
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

    cv::Mat_<double> new_r(3,3), new_t(3,1), new_rod(3,1);
    new_rod << camera[0], camera[1], camera[2];
    new_t << camera[3], camera[4], camera[5];
    cv::Rodrigues(new_rod, new_r);

    return std::tuple{new_r, new_t};
}
