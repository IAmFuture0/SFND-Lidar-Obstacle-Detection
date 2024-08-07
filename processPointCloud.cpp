// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

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

    // SementPlane Function
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	// create the filtering object
  	pcl::VoxelGrid<PointT> sor;
  	sor.setInputCloud(cloud);
  	sor.setLeafSize(0.2f, 0.2f, 0.2f);
  	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());
  	sor.filter(*cloud_filtered);
    
  	// Region based filtering (remove roof)
  	typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
  	pcl::CropBox<PointT> region(true);
  	region.setMin(minPoint);
  	region.setMax(maxPoint);
  	region.setInputCloud(cloud_filtered);
  	region.filter(*cloudRegion);
  
  	// Region of Interest (roof)
  	std::vector<int> indices;
  	pcl::CropBox<PointT> roof(true);
  	roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  	roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  	roof.setInputCloud(cloudRegion);
  	roof.filter(indices);
  
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	for(int point : indices){
      inliers->indices.push_back(point);
    }
  
  	// Remove points in the roof
  	pcl::ExtractIndices<PointT> extract;
  	extract.setInputCloud(cloudRegion);
  	extract.setIndices(inliers);
  	extract.setNegative(true);
  	extract.filter(*cloudRegion);
  	
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  	return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	typename pcl::PointCloud<PointT>::Ptr obstaCloud (new pcl::PointCloud<PointT>);
  	typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>);
  
  	// add inliners to the plane cloud
  	for(auto index : inliers->indices){
      planeCloud->points.push_back(cloud->points[index]);
    }
      
  	pcl::ExtractIndices<pcl::PointXYZ> extract;
  	extract.setInputCloud(cloud);
  	extract.setIndices(inliers);
  	extract.setNegative(true); // remove the inliers in the cloud
  	extract.filter(*obstaCloud);
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
  
    // TODO:: Fill in this function to find inliers for the cloud.
  	// Create the segmentation object
  	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    
  	seg.setModelType(pcl::SACMODEL_PLANE);
  	seg.setMethodType(pcl::SAC_RANSAC);
  	seg.setMaxIterations(maxIterations);
  	seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0){
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
  
  	// separate clouds
  	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> seg_result = SeparateClouds(inliers, cloud);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  	tree->setInputCloud(cloud);
  
  	std::vector<pcl::PointIndices>cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setClusterTolerance(clusterTolerance);
  	ec.setMinClusterSize(minSize);
  	ec.setMaxClusterSize(maxSize);
  	ec.setSearchMethod(tree);
  	ec.setInputCloud(cloud);
  	ec.extract(cluster_indices);
  
  	for(const auto& cluster: cluster_indices){
    	typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      	for(const auto& idx : cluster.indices){
          cloud_cluster->push_back((*cloud)[idx]);
        }
      	cloud_cluster->width = cloud_cluster->size();//?
      	cloud_cluster->height = 1;//?
      	cloud_cluster->is_dense = true; //?
      
      clusters.push_back(cloud_cluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol){
	std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    int lowerbound = 0;
    int upperbound = cloud->points.size()-1;
    
    for (int i = 0 ; i < maxIterations; i++){

        // *** random sampling ***
    	int random_index1 = lowerbound + std::rand() % (upperbound - lowerbound + 1);
        int random_index2 = lowerbound + std::rand() % (upperbound - lowerbound + 1);
        int random_index3 = lowerbound + std::rand() % (upperbound - lowerbound + 1);
        
        // fit the plane by finding the vectors between points
        double x1 = cloud->points[random_index1].x;
        double y1 = cloud->points[random_index1].y;
      	double z1 = cloud->points[random_index1].z;
      	double x2 = cloud->points[random_index2].x;
      	double y2 = cloud->points[random_index2].y;
      	double z2 = cloud->points[random_index2].z;
	  	double x3 = cloud->points[random_index3].x;
      	double y3 = cloud->points[random_index3].y;
      	double z3 = cloud->points[random_index3].z;
        
        double vec1_x = x2-x1, vec1_y = y2-y1, vec1_z = z2-z1;
     	double vec2_x = x3-x1, vec2_y = y3-y1, vec2_z = z3-z1;
        
        double coeffA = vec1_y*vec2_z - vec1_z*vec2_y, coeffB = vec1_z*vec2_x - vec1_x*vec2_z, coeffC = vec1_x*vec2_y -vec1_y*vec2_x;
      	double coeffD = -1*(coeffA*x1 + coeffB*y1 + coeffC*z1);
      
      	// Measure distance between every point and fitted plane
        std::unordered_set<int> tmp_inliers;
        for(int i = 0; i < cloud->points.size(); i++){
        	double pointX = cloud->points[i].x;
        	double pointY = cloud->points[i].y;
        	double pointZ = cloud->points[i].z;

        	double distance = fabs(coeffA * pointX + coeffB * pointY + coeffC * pointZ + coeffD) / sqrt(coeffA*coeffA + coeffB*coeffB + coeffC*coeffC);
        	// If distance is smaller than threshold count it as inlier
        	if (distance < distanceTol){
        		tmp_inliers.insert(i);
        	}
      	}
      	if(tmp_inliers.size() > inliersResult.size()){
        	inliersResult = tmp_inliers;
      	}
    }
    return inliersResult;
}

