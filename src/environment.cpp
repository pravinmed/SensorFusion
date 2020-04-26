/* \author Aaron Brown */
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


void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;//true;
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = 
    //    pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    Eigen::Vector4f minPoint;
    minPoint[0]=-9;  // define minimum point x
    minPoint[1]=-5;  // define minimum point y
    minPoint[2]=-2;  // define minimum point z
    minPoint[3]= 1.0;
    Eigen::Vector4f maxPoint;
    maxPoint[0]=25;  // define max point x
    maxPoint[1]=5;  // define max point y
    maxPoint[2]=1;  // define max point z
    maxPoint[3]=1.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloud = 
        pointProcessorI->FilterCloud(inputCloud, 0.2f, minPoint, maxPoint);
   
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>     
      segmentCloud = pointProcessorI->SegmentPlane(outputCloud, 50, 0.3);
    
    //renderPointCloud(viewer,segmentCloud.first,"inputCloud");
    //renderPointCloud(viewer,segmentCloud.second,"outputCloud");
   
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = 
        pointProcessorI->Clustering(segmentCloud.second, .6, 20, 250);
   
    //renderPointCloud(viewer,segmentCloud.second,"outputCloud");
    std::vector<Color> colors = {
            Color(1,0,0),
            Color(0,1,0),
            Color(0,0,1.0),
            Color(0.5,0,0),
            Color(0,0.5,0),
            Color(0,0,0.5),
            Color(0.5,0.2,0.2),
            Color(0,1,0.2)};

    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        pointProcessorI->numPoints(cluster);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderPointCloud(
            viewer,
            cluster,
            "obstCloud"+std::to_string(clusterId),
            colors[clusterId%colors.size()]
        );
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;//true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
     // TODO:: Create lidar sensor 
    double slope = 0.0;
    const std::shared_ptr<Lidar> lidar = std::make_shared<Lidar>(cars, slope);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloud = lidar->scan();
    Vect3 vec(0.0,0.0,3.0);
    //renderRays(viewer, vec,ptCloud);
    Color c(1,0.5,0.4);
    //renderPointCloud(viewer, ptCloud, "Point", c);
    
    // TODO:: Create point processor
    std::shared_ptr<ProcessPointClouds<pcl::PointXYZ>> processPtCloud =
         std::make_shared<ProcessPointClouds<pcl::PointXYZ>>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> 
        segmentCloud =  processPtCloud->SegmentPlane(ptCloud, 100, 0.2);
    //renderPointCloud(viewer, segmentCloud.second, "PointCloud", Color(0,0,1));
    //renderPointCloud(viewer, segmentCloud.first, "Obstacle ", Color(1,0,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = 
        processPtCloud->Clustering(segmentCloud.second, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        processPtCloud->numPoints(cluster);
        Box box = processPtCloud->BoundingBox(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
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
    //simpleHighway(viewer);
    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        
        viewer->removeAllShapes();
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);
        streamIterator++;
        if(streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    } 
}