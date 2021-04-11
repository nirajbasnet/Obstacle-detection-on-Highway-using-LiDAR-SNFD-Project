//
// Created by niraj on 4/4/21.
//

#include "kdtree.h"

void clusterRecursiveHelper(const int id,
                            const std::vector<std::vector<float>> &points,
                            KdTree *tree,
                            std::vector<int> &cluster,
                            std::vector<bool> &processed,
                            const float distanceTol) {
    const std::vector<float> point = points[id];
    // Mark point as processed
    processed[id] = true;
    cluster.push_back(id);

    // Find nearby ids
    std::vector<int> nearby_ids = tree->search(point, distanceTol);

    // Iterate over each nearby id
    for (int query_id : nearby_ids) {
        if (!processed[query_id])
            clusterRecursiveHelper(query_id, points, tree, cluster, processed, distanceTol);
    }
}

std::vector<std::vector<int>>
EuclideanClusteringRecursive(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol) {
    //Recursive implementation of Euclidean clustering
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (size_t i = 0; i < points.size(); i++) {
        if (processed[i])
            continue;
        std::vector<int> cluster;
        clusterRecursiveHelper(i, points, tree, cluster, processed, distanceTol);
        clusters.push_back(cluster);
    }
    return clusters;
}


std::vector<std::vector<int>>
EuclideanClustering(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol) {
    //Iterative implementation of Euclidean clustering
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);
    std::stack<int> pointsStack;
    for (int i = 0; i < points.size(); i++) {
        if (processed[i])
            continue;
        std::vector<int> cluster;
        pointsStack.push(i);
        while (!pointsStack.empty()) {
            int point_id = pointsStack.top();
            pointsStack.pop();
            //Points may be added more than once, so checking to avoid duplicate processing
            if (processed[point_id])
                continue;

            const std::vector<float> point = points[point_id];
            processed[point_id] = true;
            cluster.push_back(point_id);

            //Find near points indices
            std::vector<int> near_idxs = tree->search(point, distanceTol);

            // Iterate over each near point idx
            for (int id : near_idxs) {
                if (!processed[id])
                    pointsStack.push(id);
            }
        }
        clusters.push_back(cluster);
    }
    return clusters;
}