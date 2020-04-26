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


double GetDistanceFromPlane(
	pcl::PointXYZ pt1,
	double A, double B, double C, double D)
{
	double den = std::sqrt(A*A + B*B + C*C);
	std::cout << "Deno " << den << std::endl;
	return fabs((pt1.x*A + pt1.y*B + pt1.z*C + D)/(den));
}


std::unordered_set<int> Ransac3D(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
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
	
		pcl::PointXYZ pt1 = cloud->points[indx1];
		pcl::PointXYZ pt2 = cloud->points[indx2];
		pcl::PointXYZ pt3 = cloud->points[indx3];
		
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
			double dist = GetDistanceFromPlane(pt, A, B,C,D);
			if (std::abs(dist) <=  distanceTol)
			{
				std::cout << dist << " Distance " << std::endl;	
				inliersTmpResult.insert(k);
			}
		}
		//std::cout << " Iteraton " << i << " complete " << std::endl;
		std::cout <<inliersTmpResult.size() << std::endl;
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

double GetDistance(pcl::PointXYZ pt1,double A, double B, double C)
{
	if (A <= 0.0001 && B <= 0.0001) return 9999;
	return std::fabs((A*pt1.x + B*pt1.y + C) /(std::sqrt(A*A + B*B)));
}
std::unordered_set<int> Ransac(
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
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
		while(inliersTmpResult.size() < 2) {
			indx1 = rand()%size;
			indx2 = rand()%size;
			inliersTmpResult.insert(indx1);
			inliersTmpResult.insert(indx2);
		}
	
		pcl::PointXYZ pt1 = cloud->points[indx1];
		pcl::PointXYZ pt2 = cloud->points[indx2];
		
		double A = pt2.y - pt1.y;
		double B = pt2.x - pt1.x;
		double C = pt1.x*pt2.y - pt1.y*pt2.x;
		for(int k =0;k < size;k++)
		{
			if (inliersTmpResult.count(k) == 1) continue;
			auto pt = cloud->points[k];
			double dist = GetDistance(pt, A, B,C);
			if (std::abs(dist) <=  distanceTol)
			{
				std::cout << dist << " Distance " << std::endl;	
				inliersTmpResult.insert(k);
			}
		}
		std::cout << " Iteraton " << i << " complete " << std::endl;
		std::cout <<inliersTmpResult.size() << std::endl;
		if (inliersTmpResult.size() > max_inliers)
		{
			max_inliers = inliersTmpResult.size();
			std::cout<< "Max Inliers " << max_inliers << std::endl;	
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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	//CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);
	// Ransac(cloud, 50, 0.6);

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
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,0,1));
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
