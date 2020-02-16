#ifndef NAV_UTILS_PATHS_H
#define NAV_UTILS_PATHS_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace nav_utils
{

template<typename Path>
void downsample(const nav_msgs::Path &msg, Path &out, int skip, float unit)
{
    typedef typename Path::_positions_type::value_type Point;
    typedef typename Point::_x_type T;
    out.header = msg.header;
    out.unit = unit;
    out.positions.reserve(msg.poses.size() / (1 + skip) + 1);
    if (!msg.poses.empty())
    {
        for (size_t i = 0; i < msg.poses.size() - 1; i += 1 + skip) {
            Point p;
            p.x = T(msg.poses[i].pose.position.x / unit);
            p.y = T(msg.poses[i].pose.position.y / unit);
            p.z = T(msg.poses[i].pose.position.z / unit);
            out.positions.push_back(p);
        }
    }
    // Get the last position we missed above if there is any.
    if (!msg.poses.empty())
    {
        Point p;
        p.x = T(msg.poses.back().pose.position.x / unit);
        p.y = T(msg.poses.back().pose.position.y / unit);
        p.z = T(msg.poses.back().pose.position.z / unit);
        out.positions.push_back(p);
    }
}

template<typename Path>
void upsample(const Path &msg, nav_msgs::Path &out)
{
    out.header = msg.header;
    out.poses.reserve(msg.positions.size());
    for (const auto& p: msg.positions)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = double(p.x) * msg.unit;
        pose.pose.position.y = double(p.y) * msg.unit;
        pose.pose.position.z = double(p.z) * msg.unit;
        pose.pose.orientation.w = 1.0;
        out.poses.push_back(pose);
    }
}

}
#endif //NAV_UTILS_PATHS_H