/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"

#include "processPointClouds.h"
#include "kdtree.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1( Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2( Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3( Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
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
    Lidar* lidar_sensor = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_result =  lidar_sensor->scan();

    ProcessPointClouds<pcl::PointXYZ> pointProcessor; // = new ProcessPointClouds<pcl::PointXYZ>();
    //std::pair<pcl::PointCloud<PointT>::Ptr,        pcl::PointCloud<PointT>::Ptr>
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(scan_result, 200, 0.08);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    /*    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1, 3, 30);

        int clusterId = 0;
        std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

        for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
            ++clusterId;
        }*/

    //delete lidar_sensor;
    //delete pointProcessor;
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
    case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
    case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
    case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem (1.0);
}

void proximity(const std::vector<std::vector<float>>& point, std::vector<int> &cluster, std::unordered_set<int> &point_index, int id, float distanceTol, KdTree* tree)
{
    point_index.insert(id);

    //std::cout << "size:"  << point_index.size() << " id:" << id << std::endl;
    cluster.push_back(id);
    //std::cout << "id" << id << std::endl;
    std::vector<int> nearby = tree->search(point[id], distanceTol);
    std::cout << "nearby:"  << nearby.size() << "point id:" << id << std::endl;
    for (int iter = 0; iter < nearby.size(); iter++)
    {
        //std::cout << "nearby point:"  << nearby[iter] << "iter id:" << iter << std::endl;
        if (point_index.count(nearby[iter]) == 0)
        {
            //std::cout << "New id" << id << std::endl;
            proximity(point, cluster, point_index, nearby[iter] , distanceTol, tree);
        }

    }

    return;

}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
    std::unordered_set<int> point_index;

    for (int id = 0; id < points.size(); ++id)
    {
        if (point_index.count(id) > 0)
        {
            continue;
        }
        std::vector<int> cluster;
        proximity(points, cluster, point_index, id, distanceTol, tree);
        clusters.push_back(cluster);
       // std::cout << "cluster size :" << cluster.size() << std::endl;

    }

    return clusters;

}

void sample()
{

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1f , Eigen::Vector4f ( -15, -6.5, -20, 1), Eigen::Vector4f ( 25, 7.5, 20, 1));
    //renderPointCloud(viewer, filterCloud, "filterCloud");
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filterCloud, 100, 0.1);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 3, 40, 600 );

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 3]);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
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
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, inputCloud, "inputCloud");
    cityBlock(viewer, pointProcessorI, inputCloud);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }



    /*    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
        std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
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
            if (streamIterator == stream.end())
                streamIterator = stream.begin();

            viewer->spinOnce ();
        }*/
}