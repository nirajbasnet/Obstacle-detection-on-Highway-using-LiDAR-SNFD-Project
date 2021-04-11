//
// Created by niraj on 3/28/21.
//

#ifndef RANSAC3D_H
#define RANSAC3D_H

#include <pcl/common/common.h>
#include <unordered_set>

template<typename PointT>
std::unordered_set<int>
Segment3DPlane(typename pcl::PointCloud<PointT>::Ptr cloud, const int maxIterations, const float distanceThreshold) {

    std::unordered_set<int> inliers_plane;
    std::unordered_set<int> inliers_iteration;
    srand(time(NULL));

    const int num_points = cloud->points.size();

    for (int i = 0; i < maxIterations; i++) {
        inliers_iteration.clear();
        // Sample three random indices to get corresponding points
        std::unordered_set<int> sampling_indices_set;
        while (sampling_indices_set.size() < 3) {
            sampling_indices_set.insert(std::rand() % num_points);
        }
        // Getting the points from point cloud data
        std::unordered_set<int>::iterator it = sampling_indices_set.begin();
        const auto &pt1 = cloud->points[*it++];
        const auto &pt2 = cloud->points[*it++];
        const auto &pt3 = cloud->points[*it];

        // Calculate the coefficients for the plane in 3D
        // A = (y2 - y1)(z3 - z1) - (z2 - z1)(y3 - y1)
        // B = (z2 - z1)(x3 - x1) - (x2 - x1)(z3 - z1)
        // C = (x2 − x1)(y3 − y1) − (y2 − y1)(x3 − x1)
        // D = -(A*x1 + B*y1 + C*z1)

        double A = (pt2.y - pt1.y) * (pt3.z - pt1.z) - (pt2.z - pt1.z) * (pt3.y - pt1.y);
        double B = (pt2.z - pt1.z) * (pt3.x - pt1.x) - (pt2.x - pt1.x) * (pt3.z - pt1.z);
        double C = (pt2.x - pt1.x) * (pt3.y - pt1.y) - (pt2.y - pt1.y) * (pt3.x - pt1.x);
        double D = -(A * pt1.x + B * pt1.y + C * pt1.z);

        // Find inliers
        for (int j = 0; j < num_points; j++) {
            auto &pt = cloud->points[j];
            double dist = std::abs(A * pt.x + B * pt.y + C * pt.z + D) / std::sqrt(A * A + B * B + C * C);
            if (dist <= distanceThreshold)
                inliers_iteration.insert(j);
        }

        // Update indices of fitted plane that has most inliers
        if (inliers_iteration.size() > inliers_plane.size())
            inliers_plane = inliers_iteration;
    }
    // Return indices of fitted plane that has most inliers
    return inliers_plane;

}

#endif
