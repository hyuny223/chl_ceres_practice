#pragma once

#include <tuple>
#include <iostream>

#include "opencv2/opencv.hpp"

auto dot(const cv::Mat &a, const cv::Mat &b)
{
    assert(a.cols == b.rows);
    cv::Mat_<double> ans(a.rows, b.cols);

    for (int lr = 0; lr < a.rows; ++lr)
    {
        for (int rc = 0; rc < b.cols; ++rc)
        {
            double tmp{0};
            for (int rr = 0; rr < b.rows; ++rr)
            {
                tmp += a.ptr<double>(lr)[rr] * b.ptr<double>(rc)[rr];
            }
            ans.ptr<double>(lr)[rc] = tmp;
        }
    }
    return ans;
}

auto cat(const cv::Mat &r, const cv::Mat &t)
{
    cv::Mat_<double> T(4, 4);
    cv::hconcat(r, t, T); // [R t]를 만들어 주는 과정

    cv::Mat_<double> b(1, 4);
    b << 0, 0, 0, 1;

    cv::vconcat(T, b, T); // [[R t], [0 1]]를 만들어주는 과정
    return T;
}

auto split(const cv::Mat &T)
{
    cv::Rect roi_r(0, 0, 3, 3);
    cv::Rect roi_t(3, 0, 1, 3);

    cv::Mat r = T(roi_r);
    cv::Mat t = T(roi_t);

    return std::tuple{r, t};
}

auto detect(const auto &frame)
{
    cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();

    std::vector<cv::KeyPoint> point;

    cv::Mat desc;

    cv::Mat mask;
    detector->detectAndCompute(frame->getImage(), mask, point, desc);

    // std::cout << desc.size() << std::endl; // row : 900 col : 128

    return std::tuple{point, desc};
}

auto match(const auto &frame1, const auto &point1, const auto &desc1, const auto &frame2, const auto &point2, const auto desc2, int view)
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2);

    std::vector<cv::DMatch> matches;
    matcher->match(desc1, desc2, matches);

    auto min_max = std::minmax_element(matches.begin(), matches.end(),
                                       [](const cv::DMatch &m1, const cv::DMatch &m2)
                                       { return m1.distance < m2.distance; });
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

    std::vector<cv::Point2d> good1, good2;
    std::vector<int> good1_idx_in_desc, good2_idx_in_desc;
    good1.reserve(point1.size());
    good2.reserve(point2.size());
    good1_idx_in_desc.reserve(point2.size());
    good2_idx_in_desc.reserve(point2.size());

    for (size_t i = 0; i < good_matches.size(); i++)
    {
        // std::printf("(left, right) = (%d, %d)\n",good_matches[i].queryIdx, good_matches[i].trainIdx);
        good1.push_back(point1[good_matches[i].queryIdx].pt);
        good2.push_back(point2[good_matches[i].trainIdx].pt);

        good1_idx_in_desc.push_back(good_matches[i].queryIdx);
        good2_idx_in_desc.push_back(good_matches[i].trainIdx);
    }
    if (view)
    {
        cv::Mat dst;
        cv::drawMatches(frame1->getImage(), point1, frame2->getImage(), point2, good_matches, dst);

        cv::imshow("dst", dst);
        cv::waitKey(0);
    }

    return std::tuple{good1, good1_idx_in_desc, good2, good2_idx_in_desc};
}
auto computeEssentialMatrix(const auto &good1, const auto &good2)
{
    cv::Mat_<double> K(3, 3);
    K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

    cv::Mat e_mat = cv::findEssentialMat(good1, good2, K);

    cv::Mat r_mat, t_mat;

    cv::recoverPose(e_mat, good1, good2, K, r_mat, t_mat);
    return std::tuple{r_mat, t_mat};
}
auto pixel2cam(const auto &point)
{
    cv::Mat_<double> K(3, 3);
    K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

    cv::Point2d pt((point.x - K.ptr<double>(0)[2]) / K.ptr<double>(0)[0], (point.y - K.ptr<double>(1)[2]) / K.ptr<double>(1)[1]);
    return pt;
}

auto computeTriangulation(const cv::Mat &r_mat, const cv::Mat &t_mat, const auto &good1, const auto &good2)
{
    cv::Mat_<double> K(3, 3);
    K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

    cv::Mat_<double> T_mat1(3, 4), T_mat2(3, 4);
    T_mat1 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
    T_mat2 << r_mat.ptr<double>(0)[0], r_mat.ptr<double>(0)[1], r_mat.ptr<double>(0)[2], t_mat.ptr<double>(0)[0],
        r_mat.ptr<double>(1)[0], r_mat.ptr<double>(1)[1], r_mat.ptr<double>(1)[2], t_mat.ptr<double>(1)[0],
        r_mat.ptr<double>(2)[0], r_mat.ptr<double>(2)[1], r_mat.ptr<double>(2)[2], t_mat.ptr<double>(2)[0];

    std::vector<cv::Point3d> point_3d;
    point_3d.reserve(good1.size());

    std::vector<cv::Point2d> point1, point2;
    point1.reserve(good1.size());
    point2.reserve(good2.size());

    for (int i = 0; i < good1.size(); ++i)
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

auto projection(const cv::Mat &r_mat, const cv::Mat &t_mat, const auto &point_3d)
{
    cv::Mat rodrigues;
    cv::Rodrigues(r_mat, rodrigues);
    cv::Mat_<double> K(3, 3);
    K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;
    std::vector<cv::Point2d> projected;
    cv::Mat dist_coeff;
    cv::projectPoints(point_3d, rodrigues, t_mat, K, dist_coeff, projected);

    return projected;
}

auto computePnP(const auto &point_3d, const auto &point2)
{
    cv::Mat_<double> K(3, 3);
    K << 718.856, 0, 607.1928, 0, 718.856, 185.2157, 0, 0, 1;

    cv::Mat_<double> dist_coeffs(4, 1);
    dist_coeffs << 0, 0, 0, 0;

    cv::Mat rodrigues, t_mat, r_mat;
    cv::solvePnPRansac(point_3d, point2, K, dist_coeffs, rodrigues, t_mat);
    cv::Rodrigues(rodrigues, r_mat);

    return std::tuple{r_mat, t_mat};
}

auto visualization(const cv::Mat &image2, const auto &good2, const auto &projected)
{
    cv::Mat dst;
    cv::cvtColor(image2, dst, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < good2.size(); ++i)
    {
        cv::circle(dst, good2[i], 5, cv::Scalar(0, 0, 255), 3);
        cv::circle(dst, projected[i], 2, cv::Scalar(0, 255, 0), 3);
    }
    cv::imshow("result", dst);
    while (1)
    {
        if (cv::waitKey(0) == 27)
        {
            break;
        }
    }
}

auto match3d2d(const auto &frame,
       const auto &idx,
       const auto &good)
{
    std::vector<cv::Point3d> point_3d;
    std::vector<cv::Point2d> point_2d;

    auto map = frame->getFramePointMap();
    for (int i = 0; i < idx.size(); ++i)
    {
        if (map.contains(idx.at(i)))
        {
            point_3d.push_back(map[idx.at(i)].second->m_point_3d);
            point_2d.push_back(good.at(i));
        }
    }

    return std::tuple{point_3d, point_2d}; // 이전 프레임에서의 3d값과 현재 프레임에서의 2d 값
}

auto filterOutlier(const std::vector<cv::Point2d> &origin, const std::vector<cv::Point2d> &proj, const std::vector<cv::Point3d> &p3)
{
    std::vector<cv::Point2d> new_o, new_p;
    std::vector<cv::Point3d> new_3d;
    for(int i = 0; i < proj.size(); ++i)
    {
        if(proj.at(i).x >= 0 && proj.at(i).y >= 0)
        {
            new_o.push_back(origin.at(i));
            new_p.push_back(proj.at(i));
            new_3d.push_back(p3.at(i));
        }
    }
    return std::tuple{new_o, new_p, new_3d};
}
