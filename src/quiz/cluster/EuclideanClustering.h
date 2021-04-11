//
// Created by niraj on 3/28/21.
//

#ifndef EUCLIDEANCLUSTERING_H
#define EUCLIDEANCLUSTERING_H

#include "kdtree.h"

//Helper for recursive version
void clusterRecursiveHelper(const int id,
                            const std::vector<std::vector<float>> &points,
                            KdTree *tree,
                            std::vector<int> &cluster,
                            std::vector<bool> &processed,
                            const float distanceTol);

//Recursive implementation of Euclidean clustering
std::vector<std::vector<int>> EuclideanClusteringRecursive(
        const std::vector<std::vector<float>> &points,
        KdTree *tree,
        float distanceTol);

//Iterative implementation of Euclidean clustering
std::vector<std::vector<int>> EuclideanClustering(
        const std::vector<std::vector<float>> &points,
        KdTree *tree,
        float distanceTol);

#endif
