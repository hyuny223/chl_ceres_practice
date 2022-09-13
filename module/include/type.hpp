#pragma once

#include <iostream>
#include <unordered_map>

#include "opencv2/opencv.hpp"

// typedef uint64_t Identifier;

namespace type
{
    class Frame;

    class FramePoint
    {
    public:
        // ds construct a new framepoint, owned by the provided Frame
        FramePoint(const int &idx,
                   const cv::Point2d &good,
                   const cv::Point3d &point_3d,
                   Frame *frame);
        ~FramePoint();
        void setPrevious(FramePoint *previous);
        const FramePoint *origin() const { return m_origin; }
        FramePoint *origin() { return m_origin; }
        void setOrigin(FramePoint *origin) { m_origin = origin; }

    public:
        // unique identifier for a framepoint (exists once in memory)
        //  const Identifier m_identifier;

        // connections to temporal and ownership elements
        FramePoint *m_previous = nullptr; // FramePoint in the previous image
        FramePoint *m_next = nullptr;     // FramePoint in the next image (updated as soon as previous is called)
        FramePoint *m_origin = nullptr;   // FramePoint in the image where it was first detected (track start)
        Frame *m_frame = nullptr;         // Frame to which the point belongs

        // triangulation information (set by StereoFramePointGenerator)
        // const real _disparity_pixels;
        int m_idx;
        cv::Point2d m_good;
        cv::Point3d m_point_3d;
        bool inlier; // pose opt. inlier status

        // PointCoordinates _image_coordinates_left;
        int row, col, id{0};
    };

    /*
    1번과 2번에서 키포인트가 검출되고 매칭이 이루어진다.
    매칭점이 프레임포인트가 된다. 그러면 1번 2번 모두 프레임 포인트가 되어야 할 것.
    1번을 프레임 포인트로 만들 때에는 이전 프레임 포인트를 null로 주어야할 듯?
    2번 프레임을 프레임 포인트로 만들 때에는 1번 매칭 프레임 포인트를 주어야할 듯.

    예시 : 1(23)과 (12)3가 키포인트가 된다면, 1번 이미지는 23이 프레임포인트가 된다. 1번에서 2번과 3번을 프레임포인트로 만드는데 previous는 null로 줌
    2번 이미지에서는 (12)가 프레임 포인트가 된다. 이전의 2,3번을 previous로 준다.

    2번과 3번도 같은 과정을 거친다. 중요한 점은, 2번과 3번에서 매칭점이 나타났는데, 어떻게 1번과 연결시킬 것인가가 문제이다. 디스크립터 비교를 해야하나..?
    */

    /*
    match의 queryIdx와 trainIdx를 저장할 수 있는 녀석 이것과 2D좌표와 3D 좌표를 함께 연결해주어야 한다.
    따로 저장해야하나 함께 묵어야 하나.. 함께 묶는다면 tuple로 묶으면 됨.
    */
    class Frame // good_matches임
    {
    protected:
        cv::Mat m_image;
        std::unordered_map<int, std::pair<int, type::FramePoint *>> mm_frame_point; // right, gt / framepoint
        std::vector<FramePoint *> mv_frame_point;

    public:
        Frame(const cv::Mat &image, Frame *previous);
        FramePoint *createFramePoint(const int &idx,
                                     const int &target, // next
                                     const cv::Point2d &good,
                                     const cv::Point3d &point_3d,
                                     FramePoint *prev = 0);
        cv::Mat &getImage();
        std::vector<FramePoint *> &getFramePointVector();
        std::unordered_map<int, std::pair<int, type::FramePoint *>> &getFramePointMap();
        void setPrevious(Frame *previous);
        const Frame *origin() const { return m_origin; }
        Frame *origin() { return m_origin; }
        void setOrigin(Frame *origin) { m_origin = origin; }

    public:
        std::size_t size = 0;
        Frame *m_previous = nullptr;
        Frame *m_next = nullptr;
        Frame *m_origin = nullptr;
    };
}

void makeFramePointInFrame(type::Frame *frame1,
                           const std::vector<int> &idx1,
                           const std::vector<cv::Point2d> &good1,
                           type::Frame *frame2,
                           const std::vector<int> &idx2,
                           const std::vector<cv::Point2d> &good2,
                           const std::vector<cv::Point3d> &point_3d);


auto prevCheck(const std::unordered_map<int, std::pair<int, type::FramePoint *>> &vec, const int &idx)
{
    for (const auto &v : vec)
    {
        if (v.second.first == idx)
        {
            return std::tuple{true, v};
        }
    }
    for (const auto &v : vec)
    {
        return std::tuple{false, v};
    }
}
