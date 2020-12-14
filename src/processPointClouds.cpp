// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <pcl/PointIndices.h>
#include <unordered_set>
#include "./quiz/cluster/kdtree.h"
//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*cloud);

    pcl::CropBox<PointT> reg(true);
    reg.setMin(minPoint);
    reg.setMax(maxPoint);
    reg.setInputCloud(cloud);
    reg.filter(*cloud);

    pcl::CropBox<PointT> reg2(true);
    std::vector<int> indices;
    reg2.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    reg2.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    reg2.setInputCloud(cloud);
    reg2.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> ec;
    ec.setInputCloud(cloud);
    ec.setIndices(inliers);
    ec.setNegative(true);
    ec.filter(*cloud);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane_cloud(new typename pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new typename pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;

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

	      float dist = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a+b*b+c*c);
	      // std::cout<<"distance : "<<d<<", distanceTol : "<<distanceThreshold<<std::endl;
	      if (dist<distanceThreshold)
	        inliers.insert(index);
	    }

	    if (inliers.size() > inliersResult.size())
	    {
	      inliersResult = inliers;
	    }
    }

    // cout<<"Total points : "<<cloud->points.size()<<", total plane points : "<<inliersResult.size()<<std::endl;
    pcl::PointIndices::Ptr ptIndices(new pcl::PointIndices); // 

    for (auto it = inliersResult.begin(); it!=inliersResult.end(); it++) // i=0;i<inliersResult.size();i++)
    {
    	ptIndices->indices.push_back(*it);
    }

    // cout<<"Total points : "<<cloud->points.size()<<", total plane points : "<<ptIndices->indices.size()<<std::endl;
  
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	// std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(ptIndices,cloud);
    return segResult; // pair<obstacle,plane>
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(maxIterations);
    seg.setInputCloud (cloud);

    seg.segment(*inliers,*coefficients);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int index, KdTree* tree, std::vector<std::vector<float>> points, std::vector<bool>& processed_id, std::vector<int>& cluster, float distanceTol)
{
	processed_id[index] = true;
	cluster.push_back(index);
	std::vector<int> nearby = tree->search(points[index],distanceTol);
	for (int id : nearby)
	{
		if (processed_id[id]==false)
			proximity(id,tree,points,processed_id,cluster,distanceTol);
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringKdTree(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
	auto startTime = std::chrono::steady_clock::now();

	KdTree* tree = new KdTree;
	std::vector<std::vector<float>> points;
	std::vector<float> point;
	// typename PointT pclPt;
	// 	pclPt = 

	for (int i=0; i<cloud->points.size();i++)
	{
		point.push_back(cloud->points[i].x);
		point.push_back(cloud->points[i].y);
		point.push_back(cloud->points[i].z);
		points.push_back(point);
		tree->insert(point,i);
        point.erase(point.begin(),point.end());
	}

    // std::cout<<"Tree insertion done"<<std::endl;
	std::vector<std::vector<int>> clusters;
 	std::vector<bool> processed_id(points.size(),false);
 	int i=0;
 	while (i < points.size())
 	{
 		if (processed_id[i])
 		{
 			i++;
 			continue;
 		}

		std::vector<int> cluster;
		proximity(i,tree,points,processed_id,cluster,distanceTol);
		clusters.push_back(cluster);	
		i++;

 	}
    std::cout<<"Clustering done ****** num clusters : "<< clusters.size() <<std::endl;
	std::vector<typename pcl::PointCloud<PointT>::Ptr> CloudClusters;
	for (int i=0; i<clusters.size(); i++)
	{
		typename pcl::PointCloud<PointT>::Ptr CloudCluster(new pcl::PointCloud<PointT>);
		if (clusters[i].size()>=minSize && clusters[i].size()<=maxSize)
		{
    		for (auto it = clusters[i].begin(); it!=clusters[i].end(); it++)
    		{
    			CloudCluster->points.push_back(cloud->points[*it]);
    		}
    		CloudCluster->width = CloudCluster->points.size();
    		CloudCluster->height = 1;
    		CloudClusters.push_back(CloudCluster);
		}
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);	
	// std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
	return CloudClusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // std::cout<<"Extraction complete"<<std::endl;

    std::vector<pcl::PointIndices>::const_iterator it;
    for (it = cluster_indices.begin(); it!=cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new typename pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit=it->indices.begin();pit!=it->indices.end();++pit)
            cloud_cluster->push_back(cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    cout << "Clustering done ***** num clusters : " << clusters.size() << endl;
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}