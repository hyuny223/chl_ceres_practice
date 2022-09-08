#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "tuple"
#include "opencv2/opencv.hpp"
#include "filesystem"

double k[] = {718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1};



auto detect(const auto& image1, const auto image2)
{
    cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();

    std::vector<cv::KeyPoint> point1, point2;

    cv::Mat desc1, desc2;

    cv::Mat mask;
    detector->detectAndCompute(image1, mask, point1, desc1);
    detector->detectAndCompute(image2, mask, point2, desc2);

    return std::tuple{point1, desc1, point2, desc2};
}
auto match(const auto& image1, const auto& point1, const auto& desc1, const auto& image2, const auto& point2, const auto desc2)
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2);

    std::vector<cv::DMatch> matches;
    matcher->match(desc1, desc2, matches);

    // std::vector<std::vector<cv::DMatch>> matches;
    // matcher->knnMatch(Frame_L->getDescriptors(), Frame_R->getDescriptors(), matches, 2);
    // spdlog::info("- knnMatch complete");

    auto min_max = std::minmax_element(matches.begin(), matches.end(),
                                [](const cv::DMatch& m1, const cv::DMatch& m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    std::vector<cv::DMatch> good_matches;

    for (int i = 0; i < matches.size(); ++i)
    {
        if (matches[i].distance <= std::max(2 * min_dist, 70.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    // std::vector<cv::DMatch> goodMatches;
    // for (auto m : matches)
    // {
    //     if (m[0].distance / m[1].distance < 0.5)
    //     {
    //         goodMatches.push_back(m[0]);
    //     }
    // }

    std::vector<cv::Point2d> good1, good2;
    good1.reserve(point1.size());
    good2.reserve(point2.size());


    for (size_t i = 0; i < good_matches.size(); i++)
    {
        good1.push_back(point1[good_matches[i].queryIdx].pt);
        good2.push_back(point2[good_matches[i].trainIdx].pt);
    }

    cv::Mat dst;
    cv::drawMatches(image1, point1, image2, point2, good_matches, dst);

    cv::imshow("dst",dst);
    cv::waitKey(0);

    return std::tuple{good1, good2};
}
auto computeEssentialMatrix(const auto& good1, const auto& good2)
{
    cv::Mat_<double> K(3,3);
    K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

    cv::Mat e_mat = cv::findEssentialMat(good1, good2, K);

    cv::Mat r_mat, t_mat;

    cv::recoverPose(e_mat, good1, good2, K, r_mat, t_mat);
    return std::tuple{r_mat, t_mat};

}
auto pixel2cam(const auto& point)
{
    cv::Mat_<double> K(3,3);
    K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

    cv::Point2d pt((point.x - K.ptr<double>(0)[2]) / K.ptr<double>(0)[0], (point.y - K.ptr<double>(1)[2]) / K.ptr<double>(1)[1]);
    return pt;
}
auto computeTriangulation(const cv::Mat& r_mat, const cv::Mat& t_mat, const auto& good1, const auto& good2)
{
    cv::Mat_<double> K(3,3);
    K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

    cv::Mat_<double> T_mat1(3,4), T_mat2(3,4);
    T_mat1 << 1,0,0,0, 0,1,0,0, 0,0,1,0;
    T_mat2 << r_mat.ptr<double>(0)[0], r_mat.ptr<double>(0)[1], r_mat.ptr<double>(0)[2], t_mat.ptr<double>(0)[0],
              r_mat.ptr<double>(1)[0], r_mat.ptr<double>(1)[1], r_mat.ptr<double>(1)[2], t_mat.ptr<double>(1)[0],
              r_mat.ptr<double>(2)[0], r_mat.ptr<double>(2)[1], r_mat.ptr<double>(2)[2], t_mat.ptr<double>(2)[0];

    std::vector<cv::Point3d> point_3d;
    point_3d.reserve(good1.size());

    std::vector<cv::Point2d> point1, point2;
    point1.reserve(good1.size());
    point2.reserve(good2.size());

    for(int i = 0; i < good1.size(); ++i)
    {
        point1.emplace_back(pixel2cam(good1[i]));
        point2.emplace_back(pixel2cam(good2[i]));
    }

    cv::Mat point_4d;
    cv::triangulatePoints(T_mat1, T_mat2, point1, point2, point_4d);

    for (int i = 0; i < point_4d.cols; i++) 
    {
        cv::Mat x = point_4d.col(i);
        x /= x.ptr<double>(3)[0];

        cv::Point3d p(x.ptr<double>(0)[0], x.ptr<double>(1)[0], x.ptr<double>(2)[0]);
        point_3d.emplace_back(p);
    }
    return point_3d;
}
struct SnavelyReprojectionError
{
    SnavelyReprojectionError(double observed_x, double observed_y)
    : observed_x(observed_x), observed_y(observed_y) {}

    template <typename T>
    bool operator()(const T* const camera,
                    const T* const point,
                    T* residuals) const
    {
        // camera[0,1,2] are the angle-axis rotation.
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);
        // camera[3,4,5] are the translation.
        p[0] += camera[3]; p[1] += camera[4]; p[2] += camera[5];

        // Compute the center of distortion. The sign change comes from
        // the camera model that Noah Snavely's Bundler assumes, whereby
        // the camera coordinate system has a negative z axis.
        T xp = - p[0] / p[2];
        T yp = - p[1] / p[2];

        // Apply second and fourth order radial distortion.
        const T& l1 = camera[7];
        const T& l2 = camera[8];
        T r2 = xp*xp + yp*yp;
        T distortion = 1.0 + r2  * (l1 + l2  * r2);

        // Compute final projected point position.
        const T& focal = camera[6];
        T predicted_x = focal * distortion * xp;
        T predicted_y = focal * distortion * yp;

        // The error is the difference between the predicted and observed position.
        // std::cout << "px, py : \n" << predicted_x << std::endl;
        // std::cout <<predicted_y << std::endl;
        // std::cout << "bx, by : \n" << T(observed_x) << std::endl;
        // std::cout << T(observed_y) << std::endl;
        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction* Create(const double observed_x,
                                        const double observed_y)
    {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
                    new SnavelyReprojectionError(observed_x, observed_y)));
    }

    double observed_x;
    double observed_y;
};
auto optimization(const cv::Mat& r_mat, const cv::Mat& t_mat, const std::vector<cv::Point2d>& point2, const std::vector<cv::Point3d>& point_3d)
{
    using ceres::AutoDiffCostFunction;
    using ceres::CostFunction;
    using ceres::Problem;
    using ceres::Solver;
    using ceres::Solve;

    cv::Mat rodrigues;
    cv::Rodrigues(r_mat, rodrigues);

    std::cout << rodrigues << std::endl;

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


    ceres::Problem problem;
    for (int i = 0; i < point2.size(); ++i)
    {
        double* point = new double[3];
        point[0] = point_3d[i].x;
        point[1] = point_3d[i].y;
        point[2] = point_3d[i].z;
        ceres::CostFunction* cost_function =
            SnavelyReprojectionError::Create(
                point2[i].x,
                point2[i].y);
        problem.AddResidualBlock(cost_function,
                                new ceres::CauchyLoss(0.1),
                                camera,
                                point);
    }

    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.max_num_iterations = 2000;
    options.update_state_every_iteration = true;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout.precision(10);
    std::cout << summary.BriefReport() << "\n";

    cv::Mat_<double> new_r(3,3), new_t(3,1), new_rod(3,1);
    new_rod << camera[0], camera[1], camera[2];
    new_t << camera[3], camera[4], camera[5];
    cv::Rodrigues(new_rod, new_r);

    return std::tuple{new_r, new_t};
}
auto projection(const cv::Mat& r_mat, const cv::Mat& t_mat, const auto& point_3d)
{
    cv::Mat_<double> K(3,3);
    K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;
    std::vector<cv::Point2d> projected;
    cv::Mat dist_coeff;
    cv::projectPoints(point_3d, r_mat, t_mat, K, dist_coeff, projected);

    return projected;
}
auto visualization(const cv::Mat& image2, const auto& good2, const auto& projected)
{
    cv::Mat dst;
    cv::cvtColor(image2, dst, cv::COLOR_GRAY2BGR);
    for(int i = 0; i < good2.size(); ++i)
    {
        cv::circle(dst, good2[i], 10, cv::Scalar(0,0,255),3);
    }
    for(int i = 0; i < projected.size(); ++i)
    {
        cv::circle(dst, projected[i], 2, cv::Scalar(0,255,0),3);
    }

    cv::imshow("result", dst);
    while(1)
    {
        if(cv::waitKey(0) == 27)
        {
            break;
        }
    }
}

int main()
{
    cv::Mat image1 = cv::imread("../resources/before.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image2 = cv::imread("../resources/after.png", cv::IMREAD_GRAYSCALE);

    auto [point1, desc1, point2, desc2] = detect(image1, image2);
    auto [good1, good2] = match(image1,point1,desc1, image2,point2,desc2);
    auto [r_mat, t_mat] = computeEssentialMatrix(good1, good2);
    auto point_3d = computeTriangulation(r_mat, t_mat, good1, good2);
    auto tmp = projection(r_mat, t_mat, point_3d);
    visualization(image2, good2, tmp);
    auto [new_r, new_t] = optimization(r_mat, t_mat, good2, point_3d);
    auto projected = projection(new_r, new_t, point_3d);
    visualization(image2, good2, projected);
}
