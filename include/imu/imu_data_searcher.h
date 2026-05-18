
#ifndef FUNNY_LIDAR_SLAM_IMU_DATA_SEARCHER_H
#define FUNNY_LIDAR_SLAM_IMU_DATA_SEARCHER_H

#include "common/data_searcher.h"

#include <iterator>

class IMUDataSearcher final : public DataSearcher<IMUData> {
public:
    explicit IMUDataSearcher(size_t data_size) : DataSearcher<IMUData>(data_size) {}

    ~IMUDataSearcher() = default;

    std::vector<IMUData> GetDataSegment(TimeStampUs timestamp_left,
                                        TimeStampUs timestamp_right) {
        std::lock_guard<std::mutex> lg(mutex_deque_);

        if (timestamp_left >= timestamp_right || data_deque_.size() < 2) {
            return {};
        }

        if (data_deque_.front().timestamp_ > timestamp_left ||
            data_deque_.back().timestamp_ < timestamp_right) {
            return {};
        }

        const auto interpolate = [](const IMUData& data_l,
                                    const IMUData& data_r,
                                    TimeStampUs timestamp) {
            if (data_l.timestamp_ == data_r.timestamp_) {
                return data_r;
            }

            IMUData data;
            data.linear_acceleration_ = MotionInterpolator::InterpolateVectorLerp(
                    data_l.linear_acceleration_, data_r.linear_acceleration_,
                    data_l.timestamp_, data_r.timestamp_, timestamp
            );

            data.angular_velocity_ = MotionInterpolator::InterpolateVectorLerp(
                    data_l.angular_velocity_, data_r.angular_velocity_,
                    data_l.timestamp_, data_r.timestamp_, timestamp
            );

            data.orientation_ = MotionInterpolator::InterpolateQuaternionSlerp(
                    data_l.orientation_, data_r.orientation_,
                    data_l.timestamp_, data_r.timestamp_, timestamp
            );

            data.timestamp_ = timestamp;
            return data;
        };

        const auto data_at = [&](TimeStampUs timestamp) {
            if (timestamp == data_deque_.front().timestamp_) {
                return data_deque_.front();
            }
            if (timestamp == data_deque_.back().timestamp_) {
                return data_deque_.back();
            }

            for (auto iter = std::next(data_deque_.begin());
                 iter != data_deque_.end(); ++iter) {
                if (iter->timestamp_ == timestamp) {
                    return *iter;
                }
                if (iter->timestamp_ > timestamp) {
                    return interpolate(*std::prev(iter), *iter, timestamp);
                }
            }

            return data_deque_.back();
        };

        std::vector<IMUData> data_segment;
        data_segment.reserve(data_deque_.size() + 2);

        const IMUData left_data = data_at(timestamp_left);
        const IMUData right_data = data_at(timestamp_right);
        data_segment.push_back(left_data);

        TimeStampUs last_timestamp = left_data.timestamp_;
        for (const auto& data : data_deque_) {
            if (data.timestamp_ <= timestamp_left ||
                data.timestamp_ >= timestamp_right ||
                data.timestamp_ <= last_timestamp) {
                continue;
            }
            data_segment.push_back(data);
            last_timestamp = data.timestamp_;
        }

        if (right_data.timestamp_ > last_timestamp) {
            data_segment.push_back(right_data);
        }

        return data_segment;
    }
};

#endif //FUNNY_LIDAR_SLAM_IMU_DATA_SEARCHER_H
