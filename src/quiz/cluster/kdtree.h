/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H
#define KDTREE_H

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node {
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId)
            : point(arr), id(setId), left(NULL), right(NULL) {}

    ~Node() {
        delete left;
        delete right;
    }
};

struct KdTree {
    Node *root;

    KdTree() : root(NULL) {}

    ~KdTree() {
        delete root;
        root = NULL;
    }

    void insert(std::vector<float> point, int id) {
        // Base case if there are no nodes in the tree
        if (!root) {
            root = new Node(point, id);
            return;
        }

        Node *node = root;
        int depth = 0;  // Depth of tree during traversal

        while (node) {
            // Figure out which coordinate(dimension) we want to compare with
            int idx = depth % point.size();
            float coord_tree = node->point[idx];
            float coord_data = point[idx];

            // If the new element is less than the node at particular depth, then traverse left
            if (coord_data < coord_tree) {
                // Insert the node if leaf node reached on the left
                if (!node->left) {
                    node->left = new Node(point, id);
                    break;
                }
                // If not leaf node, traverse to the left
                node = node->left;
            } else if (coord_data > coord_tree) {
                // If the incoming coordinate is greater than the representative coordinate of the node, then traverse right
                if (!node->right) {
                    // Insert the node if leaf node reached on the right
                    node->right = new Node(point, id);
                    break;
                }

                // If not leaf node, traverse to the right
                node = node->right;
            }
            // Increment depth
            depth++;
        }
    }

    // Recursive helper function to help traverse through the tree
    void searchRecursiveHelper(std::vector<int> &idxs, const std::vector<float> &target, float &distanceTol, Node *node,
                               int depth) {
        // Return if at a leaf node
        if (!node)
            return;

        // Obtain the coordinate we want
        int idx = depth % target.size();
        float coord_node = node->point[idx];
        float coord_target = target[idx];

        // Check to see if all coordinates are within the tolerance of bounding box
        bool in_box = true;
        for (int i = 0; i < target.size(); i++) {
            in_box &= (std::abs(node->point[idx] - target[idx]) <= distanceTol);
        }
        if (in_box) {
            // If within bounding box, then calculate the actual physical distance
            double dist = 0.0;
            //Iterating over all the dimensions to find true euclidean distance
            for (int i = 0; i < target.size(); i++)
                dist += (node->point[i] - target[i]) * (node->point[i] - target[i]);

            if (dist <= (distanceTol * distanceTol))
                idxs.push_back(node->id);
        }

        // Check the boundaries within distanceTol for the coordinate to search for and go left and/or right if we need to
        if ((coord_target - distanceTol) < coord_node) {
            searchRecursiveHelper(idxs, target, distanceTol, node->left, depth + 1);
        }
        if ((coord_target + distanceTol) > coord_node) {
            searchRecursiveHelper(idxs, target, distanceTol, node->right, depth + 1);
        }
    }

    // Return a list of point indices in the tree that are within distanceTol of target
    std::vector<int> search(std::vector<float> target, float distanceTol) {
        std::vector<int> idxs;
        searchRecursiveHelper(idxs, target, distanceTol, root, 0);
        return idxs;
    }
};

#endif



