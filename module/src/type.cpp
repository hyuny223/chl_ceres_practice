#include <iostream>
#include <unordered_map>

#include "opencv2/opencv.hpp"

#include "type.hpp"

typedef uint64_t Identifider;

namespace type
{
	// 우측 기준으로만 진행하는건가??
	FramePoint::FramePoint(const int &idx,
						   const cv::Point2d &good,
						   const cv::Point3d &point_3d,
						   Frame *frame)
		: m_idx(idx), m_good(good), m_point_3d(point_3d), m_frame(frame) {}

	void FramePoint::setPrevious(FramePoint *previous)
	{
		// ds update linked list
		previous->m_next = this;
		m_previous = previous;

		setOrigin(previous->origin());
	}

	Frame::Frame(const cv::Mat &image, Frame *previous)
		: m_image(image)
	{
		if (previous)
		{
			// ds connect the framepoints
			this->setPrevious(previous);
		}
		else
		{
			// ds this point has no predecessor
			this->setOrigin(previous);
		}
	}
	void Frame::setPrevious(Frame *previous)
	{
		// ds update linked list
		previous->m_next = this;
		m_previous = previous;

		setOrigin(previous->origin());
	}

	FramePoint *Frame::createFramePoint(const int &idx,	   // curr
										const int &target, // next
										const cv::Point2d &good,
										const cv::Point3d &point_3d,
										FramePoint *prev)
	{

		FramePoint *frame_point = new FramePoint(idx,
												 good,
												 point_3d,
												 this);
		if (prev)
		{
			// ds connect the framepoints
			frame_point->setPrevious(prev);
		}
		else
		{
			// ds this point has no predecessor
			frame_point->setOrigin(frame_point);
		}

		mm_frame_point[idx] = std::make_pair(target, frame_point);
		// mv_frame_point.push_back(frame_point);
		++size;

		return frame_point;
	}
	cv::Mat &Frame::getImage()
	{
		return m_image;
	}
	std::vector<FramePoint *> &Frame::getFramePointVector()
	{
		return mv_frame_point;
	}
	std::unordered_map<int, std::pair<int, type::FramePoint *>> &Frame::getFramePointMap()
	{
		return mm_frame_point;
	}

};

#if 0
void makeFramePointInFrame(type::Frame *frame1,
						   const std::vector<int> &idx1,
						   const std::vector<cv::Point2d> &good1,
						   type::Frame *frame2,
						   const std::vector<int> &idx2,
						   const std::vector<cv::Point2d> &good2,
						   const std::vector<cv::Point3d> &point_3d)
{
	type::FramePoint *prev_point;
	if (frame1->m_previous) // 이전 프레임이 있다면
	{
		std::vector<type::FramePoint *> &curr1 = frame1->getFramePointVector();
		auto size1 = curr1.size();
		for (int i = 0; i < idx1.size(); ++i)
		{
			prev_point = 0;
			auto [flag, index] = prevCheck(curr1, idx1[i]);

			if (i < size1 && flag)
			{
				prev_point = curr1.at(index)->m_previous;
			}
			else
			{
				frame1->createFramePoint(idx1[i], good1[i], point_3d[i], prev_point);
			}
		}
	}
	else
	{
		for (int i = 0; i < idx1.size(); ++i)
		{
			prev_point = 0;
			frame1->createFramePoint(idx1[i], good1[i], point_3d[i], prev_point);
		}
	}

	if (frame2->m_previous) // 이전 프레임이 있다면
	{
		std::vector<type::FramePoint *> &curr2 = frame2->getFramePointVector();
		auto size2 = curr2.size();
		for (int i = 0; i < idx2.size(); ++i)
		{
			prev_point = 0;
			auto [flag, index] = prevCheck(curr2, idx2[i]);
			if (i < size2 && flag)
			{
				prev_point = curr2.at(index)->m_previous;
			}
			frame2->createFramePoint(idx2[i], good2[i], point_3d[i], prev_point);
		}
	}
	else
	{
		for (int i = 0; i < idx2.size(); ++i)
		{
			prev_point = 0;
			frame2->createFramePoint(idx2[i], good2[i], point_3d[i], prev_point);
		}
	}
}

#elif 1

void makeFramePointInFrame(type::Frame *frame1,
						   const std::vector<int> &idx1,
						   const std::vector<cv::Point2d> &good1,
						   type::Frame *frame2,
						   const std::vector<int> &idx2,
						   const std::vector<cv::Point2d> &good2,
						   const std::vector<cv::Point3d> &point_3d)
{
	type::FramePoint *prev_point;
	auto &curr1 = frame1->getFramePointMap();

	if (frame1->m_previous)
	{
		for (int i = 0; i < idx1.size(); ++i)
		{
			if (curr1.contains(idx1[i]))
			{
				curr1[idx1[i]] = std::make_pair(idx2[i], curr1[idx1[i]].second);
			}
			else
			{
				frame1->createFramePoint(idx1[i], idx2[i], good1[i], point_3d[i], 0);
			}
		}
	}
	else
	{
		for (int i = 0; i < idx1.size(); ++i)
		{
			frame1->createFramePoint(idx1[i], idx2[i], good1[i], point_3d[i], 0);
		}
	}
	for (int i = 0; i < idx1.size(); ++i)
	{
		auto [flag, key] = prevCheck(curr1, idx2[i]);
		if (flag)
		{
			prev_point = key.second.second;
		}
		else
		{
			prev_point = 0;
		}
		frame2->createFramePoint(idx2[i], -1, good1[i], point_3d[i], prev_point);
	}
}

#endif
