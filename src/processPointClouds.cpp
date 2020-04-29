// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"
#include <unordered_set>


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
     float filterRes,
     Eigen::Vector4f minPoint, 
     Eigen::Vector4f maxPoint
)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    //pcl::PCLPointCloud::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    typename pcl::PointCloud<PointT>::Ptr 
        cloud_filtered(new pcl::PointCloud<PointT>());
    pcl::PCLPointCloud2Ptr cloud_unfiltered;
    
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);
    typename pcl::PointCloud<PointT>::Ptr cloudRes(new pcl::PointCloud<PointT>());
    
   
    typename pcl::CropBox<PointT>::Ptr box(new pcl::CropBox<PointT>(true));
    box->setInputCloud(cloud_filtered);
    box->setMin(minPoint);
    box->setMax(maxPoint);
    box->filter(*cloudRes);
    
    typename pcl::CropBox<PointT>::Ptr boxRoof(new pcl::CropBox<PointT>(true));
    typename pcl::PointCloud<PointT>::Ptr cloudRes2(new pcl::PointCloud<PointT>());
   
    Eigen::Vector4f minPoint2{-1.5,1.7,-1,1.0}; 
    Eigen::Vector4f maxPoint2{2.5,1.7,0,1.0};
    std::vector<int> indices;
  
    boxRoof->setInputCloud(cloud);
    boxRoof->setMin(minPoint2);
    boxRoof->setMax(maxPoint2);
    boxRoof->filter(indices);
   
    pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(auto& idx : indices)
    {
         inliers->indices.push_back(idx);
    }
    typename pcl::PointCloud<PointT>::Ptr cloudNoRoof(new pcl::PointCloud<PointT>());
    extract.setInputCloud (cloudRes);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudNoRoof);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
   
    return cloudNoRoof;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers, 
    typename pcl::PointCloud<PointT>::Ptr cloud
) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>); 
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>);
    //The inliers can be added to the plane cloud by looping over 
    //the inlier indices and pushing the 
    //corresponding inlier point into the plane cloud’s point vector.
    for(int i : inliers->indices)
    {
        cloud_p->points.push_back(cloud->points[i]);
    }

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    int i = 0, nr_points = (int) cloud->points.size ();
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacles);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
        segResult(cloud_p, obstacles);
    return segResult;
}

template<typename PointT>
double ProcessPointClouds<PointT>::DistFromPlane(
	PointT pt1,
	double A, double B, double C, double D)
{
	double den = std::sqrt(A*A + B*B + C*C);
	return fabs((pt1.x*A + pt1.y*B + pt1.z*C + D)/(den));
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::MyRansac3DPlane(
	typename pcl::PointCloud<PointT>::Ptr cloud, 
	int maxIterations, 
	float distanceTol
)
{
std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int size = cloud->points.size();
	int max_inliers = 0;
	// For max iterations 
	for(int i = 0 ;i < maxIterations;i++)
	{
		std::unordered_set<int> inliersTmpResult;
		int indx1 = 0 ;
		int indx2 = 0;
		int indx3 = 0;
		while(inliersTmpResult.size() < 3) {
			indx1 = rand()%size;
			indx2 = rand()%size;
			indx3 = rand()%size;
			inliersTmpResult.insert(indx1);
			inliersTmpResult.insert(indx2);
			inliersTmpResult.insert(indx3);	
		}
	
		PointT pt1 = cloud->points[indx1];
		PointT pt2 = cloud->points[indx2];
		PointT pt3 = cloud->points[indx3];
		
		double A = (pt2.y - pt1.y)*(pt3.z - pt1.z) -
				   (pt2.z - pt1.z)*(pt3.y - pt1.y);
		double B =  (pt2.z - pt1.z)*(pt3.x - pt1.x) - 
					(pt2.x - pt1.x)*(pt3.z - pt1.z);
		double C = (pt2.x - pt1.x)*(pt3.y - pt1.y) - 
					(pt2.y - pt1.y)*(pt3.x - pt1.x);
		double D = -1.0*(A*pt1.x + B*pt1.y + C*pt1.z);
		
		for(int k =0;k < size;k++)
		{
			if (inliersTmpResult.count(k) == 1) continue;
			auto pt = cloud->points[k];
			double dist = DistFromPlane(pt, A, B,C,D);
			if (std::abs(dist) <=  distanceTol)
			{
				inliersTmpResult.insert(k);
			}
		}
		if (inliersTmpResult.size() > max_inliers)
		{
			max_inliers = inliersTmpResult.size();
			//std::cout<< "Max Inliers " << max_inliers << std::endl;	
			inliersResult = inliersTmpResult;
		}
	}

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	std::cout <<" Final Result " << inliersResult.size() << std::endl;
	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud,
     int maxIterations, 
     float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    std::unordered_set<int> tmpInlier  =  MyRansac3DPlane(cloud, maxIterations, distanceThreshold);
    for(auto &inLier: tmpInlier)
    {
        inliers->indices.push_back(inLier);
    }

    // Create the segmentation object
    /*  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>); 
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    std::cout << " Size of the points " << cloud->points.size() << std::endl;
    int i = 0, nr_points = (int) cloud->points.size ();
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);   
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }*/

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = 
        SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::FindProximity(
	KdTree* tree,
 	float distanceTol, 
 	int Indx,
 	std::vector<int>& clusterIndices,
	std::map<int,bool>& visited,
	const std::vector<std::vector<float>>& points
)
{
	if (visited.find(Indx) == visited.end()) {
		clusterIndices.push_back(Indx);
		visited.insert(std::make_pair(Indx, true));
		std::vector<int> list = tree->search(points[Indx], distanceTol);
		for(auto& pt: list)
		{
			if (visited.find(pt) == visited.end())
			{
				FindProximity(
						tree, 
						distanceTol,
						pt,
						clusterIndices,
						visited,
						points);
			}
		}
	}
		 
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(
	const std::vector<std::vector<float>>& points,
	 KdTree* tree,
	 float distanceTol,
     int minSize,
     int maxSize)
{
	// TODO: Fill out this function to return list of indices for each cluster
	std::map<int, bool> visited;
	std::vector<std::vector<int>> clusters;
	for(int i =0;i < points.size();i++)
	{
		if (visited.find(i) == visited.end())
		{
			std::vector<int> clusterIndx;
			FindProximity(tree, distanceTol, i,clusterIndx, visited, points);
			if (clusterIndx.size() > minSize && clusterIndx.size() < maxSize)
			{
				clusters.push_back(clusterIndx);
			}
		}
	}
	return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    float clusterTolerance, 
    int minSize, 
    int maxSize
)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;


    KdTree* ktree = new KdTree;
    std::vector<std::vector<float>> vecOfPoints;
  
    for (int i=0; i<cloud->points.size(); i++) {
        std::vector<float> pts(3);
        pts[0]  = cloud->points[i].x;
    
        pts[1] = cloud->points[i].y;
        pts[2] = cloud->points[i].z;
        vecOfPoints.push_back(pts);
    	ktree->insert(pts, i);
    }

    std::vector<std::vector<int>> clusterOfCluster = 
        euclideanCluster(vecOfPoints, ktree, clusterTolerance, minSize, maxSize);
    if (clusterOfCluster.size() > 0) {
        for(auto& clusterSet: clusterOfCluster)
        {
            typename pcl::PointCloud<PointT>::Ptr new_cluster 
                (new pcl::PointCloud<PointT>);
            for(auto& pt: clusterSet)
            {
                new_cluster->points.push_back(cloud->points[pt]);
            }
            new_cluster->width = new_cluster->points.size();
            new_cluster->height = 1;
            new_cluster->is_dense = true;
            clusters.push_back(new_cluster);
        }
    }


    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    /*typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
   
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for (
        std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
        it != cluster_indices.end (); 
        ++it
    )
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster 
            (new pcl::PointCloud<PointT>);
        for (
            std::vector<int>::const_iterator pit = it->indices.begin ();
             pit != it->indices.end (); 
             ++pit
        ) {
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);        
    }*/

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