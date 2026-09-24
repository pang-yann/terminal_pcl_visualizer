/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *  Copyright (c) 2026, Nathan Shankar.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Nathan Shankar nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include "terminal_pcl_visualizer/terminal_pcl_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <algorithm>
#include <cmath>
#include <exception>

namespace terminal_pcl_visualizer {

TerminalPCLNode::TerminalPCLNode(const rclcpp::NodeOptions & options)
: Node("terminal_pcl_visualizer", options) {
    this->declare_parameter("topic", "/points");
    this->declare_parameter("max_points", 20000);

    std::string topic = this->get_parameter("topic").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            this->callback(msg);
        });

    data_ = std::make_shared<CloudData>();
    data_->frame_id = "waiting...";
}

std::shared_ptr<CloudData> TerminalPCLNode::get_data() {
    std::lock_guard<std::mutex> lock(mtx_);
    return data_;
}

void TerminalPCLNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto next = std::make_shared<CloudData>();
    next->frame_id = msg->header.frame_id;

    const auto max_points = this->get_parameter("max_points").as_int();
    if (max_points <= 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Ignoring point cloud because max_points must be positive (got %lld)",
            static_cast<long long>(max_points));
        return;
    }

    const size_t max_p = static_cast<size_t>(max_points);
    size_t total_p = static_cast<size_t>(msg->width) * msg->height;

    const size_t minimum_row_size = static_cast<size_t>(msg->width) * msg->point_step;
    const size_t expected_data_size = static_cast<size_t>(msg->row_step) * msg->height;
    if (msg->row_step < minimum_row_size || msg->data.size() < expected_data_size) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Ignoring malformed PointCloud2: width=%u height=%u point_step=%u row_step=%u data_size=%zu (need row_step >= %zu and data_size >= %zu)",
            msg->width, msg->height, msg->point_step, msg->row_step, msg->data.size(),
            minimum_row_size, expected_data_size);
        return;
    }
    
    try {
        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

        size_t step = std::max<size_t>(1, total_p / max_p);
        double sx = 0, sy = 0, sz = 0;

        for (size_t i = 0; i < total_p; i += step) {
            if (!(it_x != it_x.end())) break;
            float x = *it_x, y = *it_y, z = *it_z;
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                next->points.push_back({x, y, z});
                sx += x; sy += y; sz += z;
                if (next->points.size() == 1) {
                    next->min_x = next->max_x = x;
                    next->min_y = next->max_y = y;
                    next->min_z = next->max_z = z;
                } else {
                    next->min_x = std::min(next->min_x, x); next->max_x = std::max(next->max_x, x);
                    next->min_y = std::min(next->min_y, y); next->max_y = std::max(next->max_y, y);
                    next->min_z = std::min(next->min_z, z); next->max_z = std::max(next->max_z, z);
                }
            }
            for (size_t s = 0; s < step && it_x != it_x.end(); ++s) { ++it_x; ++it_y; ++it_z; }
            if (next->points.size() >= max_p) break;
        }
        if (!next->points.empty()) {
            next->cx = sx / next->points.size();
            next->cy = sy / next->points.size();
            next->cz = sz / next->points.size();
        } else if (total_p > 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "PointCloud2 has %zu points but none contain finite x/y/z values", total_p);
        }
    } catch (const std::exception & error) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Failed to read PointCloud2 x/y/z fields: %s", error.what());
        return;
    } catch (...) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Failed to read PointCloud2 x/y/z fields due to an unknown error");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mtx_);
        data_ = std::move(next);
    }
}

} // namespace terminal_pcl_visualizer
