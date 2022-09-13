
#include "tuple"
#include "opencv2/opencv.hpp"

#include "compute.hpp"
#include "optimization.hpp"
#include "type.hpp"

int main()
{
    cv::Mat image1 = cv::imread("../resources/begin.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image2 = cv::imread("../resources/mid.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image3 = cv::imread("../resources/end.png", cv::IMREAD_GRAYSCALE);

    // cv::GaussianBlur(image1, image1, cv::Size(3, 3), 1);
    // cv::GaussianBlur(image2, image2, cv::Size(3, 3), 1);
    // cv::GaussianBlur(image3, image3, cv::Size(3, 3), 1);
    int view = 0;

    type::Frame *frame1 = new type::Frame(image1, 0);
    type::Frame *frame2 = new type::Frame(image2, frame1);
    type::Frame *frame3 = new type::Frame(image3, frame2);

    auto [point1, desc1] = detect(frame1);
    auto [point2, desc2] = detect(frame2);

    auto [good12, good12_idx_in_desc, good21, good21_idx_in_desc] = match(frame1, point1, desc1, frame2, point2, desc2, view);
    auto [r_mat12, t_mat12] = computeEssentialMatrix(good12, good21);
    auto T12 = cat(r_mat12, t_mat12);
    auto point_3d = computeTriangulation(r_mat12, t_mat12, good12, good21);
    makeFramePointInFrame(frame1, good12_idx_in_desc, good12, frame2, good12_idx_in_desc, good21, point_3d);

    auto [point3, desc3] = detect(frame3);
    auto [good23, good23_idx_in_desc, good32, good32_idx_in_desc] = match(frame2, point2, desc2, frame3, point3, desc3, view);
    auto [tmp_3d, point_2d] = match3d2d(frame2, good23_idx_in_desc, good32);

    auto [r_mat13, t_mat13] = computePnP(tmp_3d, point_2d);
    auto projected = projection(r_mat13, t_mat13, tmp_3d);
    for(int i = 0; i < projected.size(); ++i)
    {
        std::printf("origin : (%f, %f)\n",good32.at(i).x, good32.at(i).y);
        std::printf("projec : (%f, %f)\n",projected.at(i).x, projected.at(i).y);
        std::cout << "=================\n";
    }
    visualization(image3, good32, projected);
    auto [new_r, new_t] = optimization(r_mat13, t_mat13, point_2d, tmp_3d);
    auto op_projected = projection(new_r, new_t, tmp_3d);
    for(int i = 0; i < projected.size(); ++i)
    {
        std::printf("origin : (%f, %f)\n",good32.at(i).x, good32.at(i).y);
        std::printf("projec : (%f, %f)o\n",op_projected.at(i).x, op_projected.at(i).y);
        std::cout << "=================\n";
    }
    visualization(image3, good32, op_projected);




    // auto T13 = cat(r_mat13, t_mat13);
    // auto point23_3d = computeTriangulation(r_mat23, t_mat23, good23, good32);
    // makeFramePointInFrame(frame2, good23_idx_in_desc, good23, frame3, good32_idx_in_desc, good32, point23_3d);
}
