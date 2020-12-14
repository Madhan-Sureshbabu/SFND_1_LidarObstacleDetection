/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <math.h>

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
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
  auto startTime = std::chrono::steady_clock::now();
	// For max iterations 
  for (int i=0;i<maxIterations;i++)
  {
    // std::cout<<"In ransac"<<endl;
    std::unordered_set<int> inliers;
    while (inliers.size()<2)
      inliers.insert(rand()%cloud->points.size());

    // std::cout<<"In ransac"<<endl;
    auto itr = inliers.begin();

    float x1, y1, x2, y2;
    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;
    itr++;
    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;

    float a,b,c;
    a = y1 - y2;
    b = x2 - x1;
    c = (x1*y2 - x2*y1);

    for (int index=0; index<cloud->points.size(); index++)
    {
      if (inliers.count(index)>0)
        continue;

      float x3,y3;
      x3 = cloud->points[index].x;
      y3 = cloud->points[index].y;

      float d = fabs(a*x3 + b*y3 + c)/(a*a+b*b);
      if (d<distanceTol)
        inliers.insert(index);
    }

    if (inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    }
    

  }

  auto endTime = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout<<"Ransac took "<<duration.count() <<" milliseconds"<<std::endl;

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  
  // TODO: Fill in this function
  auto startTime = std::chrono::steady_clock::now();
  // For max iterations 
  for (int i=0;i<maxIterations;i++)
  {
    // std::cout<<"In ransac"<<endl;
    std::unordered_set<int> inliers;
    while (inliers.size()<3)
      inliers.insert(rand()%cloud->points.size());

    // std::cout<<"In ransac"<<endl;
    auto itr = inliers.begin();

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;
    z1 = cloud->points[*itr].z;
    itr++;
    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;
    z2 = cloud->points[*itr].z;
    itr++;
    x3 = cloud->points[*itr].x;
    y3 = cloud->points[*itr].y;
    z3 = cloud->points[*itr].z;
    
    float a,b,c,d;
    a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
    b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
    c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
    d = -(a*x1+b*y1+c*z1);

    for (int index=0; index<cloud->points.size(); index++)
    {
      if (inliers.count(index)>0)
        continue;

      float x4,y4,z4;
      x4 = cloud->points[index].x;
      y4 = cloud->points[index].y;
      z4 = cloud->points[index].z;

      float dist = fabs(a*x4 + b*y4 + c*z4 + d)/(a*a+b*b+c*c);
      if (dist<distanceTol)
        inliers.insert(index);
    }

    if (inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    }
    

  }

  auto endTime = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout<<"Ransac took "<<duration.count() <<" milliseconds"<<std::endl;

  // Randomly sample subset and fit line

  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier

  // Return indicies of inliers from fitted line with most inliers
  
  return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	
  // std::cout<<"In main"<<endl;
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.0005);
  // std::cout<<"In main"<<endl;

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
