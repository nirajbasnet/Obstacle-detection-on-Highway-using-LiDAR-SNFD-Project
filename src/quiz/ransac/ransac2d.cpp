/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliers_plane;
	srand(time(NULL));
	std::unordered_set<int> inliers_iteration;

  // Number of points
  const int num_points = cloud->points.size();

  for (int i = 0; i < maxIterations; i++) {
    // Randomly sample subset and fit line
    inliers_iteration.clear();
    const int index1 = std::rand() % num_points;
    const int index2 = std::rand() % num_points;

    if (index1 == index2) {
      i--;
      continue;
    }

    const auto& pt1 = cloud->points[index1];
    const auto& pt2 = cloud->points[index2];

    // Ax + By + C = 0
    // Also, y = mx + b
    // Rewriting above equation, -mx + y - b = 0
    // Commparing we get, A = -m, B = 1, C = -b
    const double slope = (pt2.y - pt1.y) / (pt2.x - pt1.x);

    // Intercept term:
    // y = mx + b
    // b = y - m*x
    const double intercept = pt1.y - slope * pt1.x;

    // Measure distance between every point and fitted line and if distance is smaller than threshold, mark it as inlier
    for (int j = 0; j < num_points; j++) {
      const auto& pt = cloud->points[j];
      // d = |Ax + By + C| / sqrt(A^2 + B^2)
      // d = |-slope*x + y - intercept| / sqrt(slope*slope + 1)
      const double d = std::abs(-slope * pt.x + pt.y - intercept) /
                       std::sqrt(slope * slope + 1.0);
      if (d <= distanceTol) 
	   inliers_iteration.insert(j); 
    }

    // Update and return indices of fitted line that has most inliers
    if (inliers_iteration.size() > inliers_plane.size()) {
      inliers_plane = inliers_iteration;
    }
  }
  return inliers_plane;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
