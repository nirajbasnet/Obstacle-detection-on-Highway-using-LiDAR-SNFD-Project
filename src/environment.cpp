// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr = lidar->scan();
    // renderRays(viewer,lidar->position,input_cloud_ptr);
    // renderPointCloud(viewer,input_cloud_ptr,"lidar_cloud");
   
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> point_clouds_processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_cloud = point_clouds_processor.SegmentPlane(input_cloud_ptr,100, 0.2);
    // renderPointCloud(viewer,segment_cloud.first,"ObstacleCloud",Color(1,0,0));
    renderPointCloud(viewer,segment_cloud.second,"PlaneCloud",Color(0,1,0));


    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacle_clusters = point_clouds_processor.Clustering(segment_cloud.first,1.0,3,30);
    std::vector<Color> color_data = {Color(1,0,0),Color(1,1,0),Color(1,0,1),Color(0,0,1)};
    int cluster_id = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr obst_cluster : obstacle_clusters){
        renderPointCloud(viewer, obst_cluster, "ObstacleCloud"+std::to_string(cluster_id),color_data[cluster_id%color_data.size()]);
        //Generating bounding box for the obstacle cluster and rendering it
        Box box = point_clouds_processor.BoundingBox(obst_cluster);
        renderBox(viewer,box,cluster_id);

        cluster_id++;
    }

    
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud,0.1 , Eigen::Vector4f (-20, -5.25, -3, 0), Eigen::Vector4f ( 20, 7, 10, 1));

//    renderPointCloud(viewer,inputCloud,"inputCloud");
//    renderPointCloud(viewer,filteredCloud,"filteredCloud");

    //Segmenting point cloud for getting road plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessorI->SegmentPlane(filteredCloud,100, 0.2);
    renderPointCloud(viewer,segment_cloud.second,"PlaneCloud",Color(0,1,0));

    //Segmenting point cloud to get obstacle clusters
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacle_clusters = pointProcessorI->Clustering(segment_cloud.first,0.2,100,2000);
    std::vector<Color> color_data = {Color(1,0,0),Color(1,1,0),Color(1,0,1),Color(0,0,1)};
    int cluster_id = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr obst_cluster : obstacle_clusters){
        renderPointCloud(viewer, obst_cluster, "ObstacleCloud"+std::to_string(cluster_id),color_data[cluster_id%color_data.size()]);
        //Generating bounding box for the obstacle cluster and rendering it
        Box box = pointProcessorI->BoundingBox(obst_cluster);
        renderBox(viewer,box,cluster_id);

        cluster_id++;
    }


}

void renderCarBox(pcl::visualization::PCLVisualizer::Ptr& viewer,const int& id){
    Box box;
    box.x_min = -1.4;
    box.y_min = -1.3;
    box.z_min = -1.5;
    box.x_max = 2.4;
    box.y_max = 1.3;
    box.z_max = 0;
    renderBox(viewer,box,id);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud,0.1 , Eigen::Vector4f (-20, -5.25, -3, 0), Eigen::Vector4f ( 20, 7, 10, 1));

//    renderPointCloud(viewer,inputCloud,"inputCloud");
//    renderPointCloud(viewer,filteredCloud,"filteredCloud");

    //Segmenting point cloud for getting road plane
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud = pointProcessorI->SegmentPlane(filteredCloud,100, 0.2);
    renderPointCloud(viewer,segment_cloud.second,"PlaneCloud",Color(0,1,0));

    //Segmenting point cloud to get obstacle clusters
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacle_clusters = pointProcessorI->Clustering(segment_cloud.first,0.2,100,2000);
    std::vector<Color> color_data = {Color(1,0,0),Color(1,1,0),Color(1,0,1),Color(0,0,1)};
    int cluster_id = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr obst_cluster : obstacle_clusters){
        renderPointCloud(viewer, obst_cluster, "ObstacleCloud"+std::to_string(cluster_id),color_data[cluster_id%color_data.size()]);
        //Generating bounding box for the obstacle cluster and rendering it
        Box box = pointProcessorI->BoundingBox(obst_cluster);
        renderBox(viewer,box,cluster_id);

        cluster_id++;
    }

    renderCarBox(viewer,cluster_id);
}



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
//    simpleHighway(viewer);
//    cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}