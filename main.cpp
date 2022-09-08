
#include "tuple"
#include "opencv2/opencv.hpp"

#include "compute.hpp"
#include "optimization.hpp"

int main()
{
    cv::Mat image1 = cv::imread("../resources/begin.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image2 = cv::imread("../resources/mid.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image3 = cv::imread("../resources/end.png", cv::IMREAD_GRAYSCALE);

    auto [point1, desc1, point2, desc2] = detect(image1, image2);
    auto [good1, good2] = match(image1,point1,desc1, image2,point2,desc2, 0);
    auto [r_mat12, t_mat12] = computeEssentialMatrix(good1, good2);
    auto T12 = cat(r_mat12, t_mat12);
    auto point_3d = computeTriangulation(r_mat12, t_mat12, good1, good2);

    auto [point3, desc3, point4, desc4] = detect(image2, image3);
    auto [good3, good4] = match(image2,point3,desc3, image3,point4,desc4, 0);
    // auto [r_mat34, t_mat34] = computeEssentialMatrix(good1, good2);
    // auto T34 = cat(r_mat34, t_mat34);
    // auto T14 = dot(T34, T12);
    // auto [r, t] = split(T14);
    auto [r_mat14, t_mat14] = computePnP(point_3d, good4);
    // auto tmp = projection(r, t, point_3d);
    // visualization(image3, good4, tmp);
    // auto [tmp1, tmp2] = match(image1, point1, desc1, image3, point4, desc4);

    // auto tmp = projection(r_mat, t_mat, point_3d);
    // visualization(image2, good2, tmp);
    // auto [new_r, new_t] = optimization(r, t, good4, point_3d);
    // auto projected = projection(new_r, new_t, point_3d);
    // visualization(image3, good4, projected);
}


/*
디스크립터가 어떤 형식으로 저장되는가.
*/
