
#include "tuple"
#include "opencv2/opencv.hpp"

#include "compute.hpp"
#include "optimization.hpp"
#include "type.hpp"

int view = 0;

int main()
{
    cv::Mat image1 = cv::imread("../resources/begin.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image2 = cv::imread("../resources/mid.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image3 = cv::imread("../resources/end.png", cv::IMREAD_GRAYSCALE);

    // cv::GaussianBlur(image1, image1, cv::Size(3, 3), 1);
    // cv::GaussianBlur(image2, image2, cv::Size(3, 3), 1);
    // cv::GaussianBlur(image3, image3, cv::Size(3, 3), 1);

    type::Frame *frame1 = new type::Frame(image1, 0);
    type::Frame *frame2 = new type::Frame(image2, frame1);
    type::Frame *frame3 = new type::Frame(image3, frame2);

    auto [point1, desc1] = detect(frame1);
    auto [point2, desc2] = detect(frame2);

    auto [good12, good12_idx_in_desc, good21, good21_idx_in_desc] = match(frame1, point1, desc1, frame2, point2, desc2, view);
    auto [r_mat12, t_mat12] = computeEssentialMatrix(good12, good21); // pose initialize, rodrigues
    auto T12 = cat(r_mat12, t_mat12);
    auto point_3d = computeTriangulation(r_mat12, t_mat12, good12, good21); // 삼각 측량으로 3D 포인트 복원
    makeFramePointInFrame(frame1, good12_idx_in_desc, good12, frame2, good12_idx_in_desc, good21, point_3d); // FramePoint 형성(edge만들어 줌)

    auto [point3, desc3] = detect(frame3);
    auto [good23, good23_idx_in_desc, good32, good32_idx_in_desc] = match(frame2, point2, desc2, frame3, point3, desc3, view);
    auto [tmp_3d, point_2d] = match3d2d(frame2, good23_idx_in_desc, good32); // 두번째 이미지에서 세번째 이미지와 매칭되는 것 중 3d값이 존재하는 것 뽑기

    auto [r_mat13, t_mat13] = computePnP(tmp_3d, point_2d); // 3d값과 3번 이미지의 2d 값을 이용하여 pnp 수행
    auto projected = projection(r_mat13, t_mat13, tmp_3d); // pnp로 도출된 R,t를 이용하여 projection 수행
    auto [new_o, new_p, new_3d] = filterOutlier(point_2d, projected, tmp_3d); // projection값 중 음수 값을 제거하는 필터 함수 
    for(int i = 0; i < new_o.size(); ++i)
    {
        std::printf("origin : (%f, %f)\n",new_o.at(i).x, new_o.at(i).y);
        std::printf("projec : (%f, %f)\n",new_p.at(i).x, new_p.at(i).y);
        std::cout << "=================\n";
    }
    visualization(image3, point_2d, projected);
    auto [new_r, new_t] = optimization(r_mat13, t_mat13, new_o, new_3d); // 2d와 3d로 solvepnp를 통해 계산된 R,t값을 최적화
    auto op_projected = projection(new_r, new_t, new_3d); // 최적화된 R, t 값으로 projection 진행
    for(int i = 0; i < new_o.size(); ++i)
    {
        std::printf("origin : (%f, %f)\n",new_o.at(i).x, new_o.at(i).y);
        std::printf("projec : (%f, %f)o\n",op_projected.at(i).x, op_projected.at(i).y);
        std::cout << "=================\n";
    }
    visualization(image3, point_2d, op_projected);


    // auto projected = projection(r_mat12, t_mat12, point_3d); // pnp로 도출된 R,t를 이용하여 projection 수행
    // for(int i = 0; i < good21.size(); ++i)
    // {
    //     std::printf("origin : (%f, %f)\n",good21.at(i).x, good21.at(i).y);
    //     std::printf("projec : (%f, %f)\n",projected.at(i).x, projected.at(i).y);
    //     std::cout << "=================\n";
    // }
    // visualization(image3, good21, projected);
    // auto [new_r, new_t] = optimization(r_mat12, t_mat12, good21, point_3d); // 2d와 3d로 solvepnp를 통해 계산된 R,t값을 최적화
    // auto op_projected = projection(new_r, new_t, point_3d); // 최적화된 R, t 값으로 projection 진행
    // for(int i = 0; i < good21.size(); ++i)
    // {
    //     std::printf("origin : (%f, %f)\n",good21.at(i).x, good21.at(i).y);
    //     std::printf("projec : (%f, %f)o\n",op_projected.at(i).x, op_projected.at(i).y);
    //     std::cout << "=================\n";
    // }
    // visualization(image3, good21, op_projected);

    // auto T13 = cat(r_mat13, t_mat13);
    // auto point23_3d = computeTriangulation(r_mat23, t_mat23, good23, good32);
    // makeFramePointInFrame(frame2, good23_idx_in_desc, good23, frame3, good32_idx_in_desc, good32, point23_3d);
}
