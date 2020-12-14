/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// #include "quiz/ransac/ransac2d.cpp"

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
    
    Lidar* lidar = new Lidar(cars, 0);
    // TODO:: Create lidar sensor 

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    // renderRays(viewer, Vect3(0,0,0), cloud);
    // renderPointCloud(viewer,cloud, "cloud");
    // TODO:: Create point processor

    ProcessPointClouds<pcl::PointXYZ> *pointProcessor(new ProcessPointClouds<pcl::PointXYZ>);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = pointProcessor->SegmentPlane(cloud,100,0.2);
    // renderPointCloud(viewer,segResult.first,"plane_cloud",Color(1,0,0));
    // renderPointCloud(viewer,segResult.second,"obstacle_cloud",Color(0,1,0));
  
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor->Clustering(segResult.second,1.0,3,30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for (auto cluster : clusters)
    {
        std::cout<<"cluster size";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);        
        ++clusterId;
    }

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pointProcessorI->FilterCloud(inputCloud,0.3,Eigen::Vector4f(-10,-6,-10,1), Eigen::Vector4f(40,6,10,1));    
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = pointProcessorI->SegmentPlane(cloud,100,0.3);

	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = pointProcessorI->SegmentPlaneRansac(cloud,100,0.3);
	
    std::cout<<"plane cloud size "<<segResult.second->points.size()<<std::endl;
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->ClusteringKdTree(segResult.first,0.6,1,400);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(segResult.first,0.6,20,300);

    renderPointCloud(viewer,segResult.second,"plane_cloud",Color(0,1,0));
    renderPointCloud(viewer,segResult.first,"obstacle_cloud",Color(1,0,0));
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(0,0,1)};
    cout << "num valid clusters : " << clusters.size() << endl;
    for (auto cluster : clusters)
    {
	        // std::cout<<"cluster size";
	        // pointProcessorI->numPoints(cluster);
	        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[0]);
	        Box box = pointProcessorI->BoundingBox(cluster);
	        renderBox(viewer,box,clusterId);        
	        ++clusterId;
    }    
  
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer,pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator==stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}