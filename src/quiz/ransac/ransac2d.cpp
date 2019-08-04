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
  for (int i = -5; i < 5; i++)
  {
    double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--)
  {
    double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
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

/*std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  pcl::PointXYZ pointOne;
  pcl::PointXYZ pointTwo;
  pcl::PointXYZ pointThree;
  float ACoeff, BCoeff, CCoeff;
  float dist;
  srand(time(NULL));
  std::cout << "size: " << cloud->points.size() << cloud->width << std::endl;
  // TODO: Fill in this function
  for (int iter = 0; iter < maxIterations; ++iter)
  {

    std::unordered_set<int> inliers;


    while (inliers.size() < 2)
    {
      inliers.insert((rand() % cloud->points.size()));
    }
    auto pointOneIndex = inliers.begin();

    pointOne = cloud->points[*pointOneIndex];
    pointOneIndex++;
    pointTwo = cloud->points[*pointOneIndex];
    ACoeff = pointOne.y - pointTwo.y;
    BCoeff = pointTwo.x - pointOne.x;
    CCoeff = (pointOne.x * pointTwo.y) - (pointTwo.x * pointOne.y) ;
    //std::cout << "dist" << dist <<  std::endl;
    for (int point = 0; point < cloud->points.size(); ++point)
    {

      if (inliers.count(point) > 0 )
      {
        //std::cout << "dist" << dist <<  std::endl;
        continue;
      }
      pointThree = cloud->points[point];
      dist = (fabs)(ACoeff * pointThree.x + BCoeff * pointThree.y + CCoeff) / sqrt(ACoeff * ACoeff + BCoeff * BCoeff);
      std::cout << "distance : " << dist <<  std::endl;
      if (dist <= distanceTol)
      {
        std::cout << "Adding Point" << std::endl;
        inliers.insert(point);
      }
    }

    if (inliersResult.size() < inliers.size())
    {
      inliersResult = inliers;
    }





  }
  // For max iterations

  // Randomly sample subset and fit line

  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier

  // Return indicies of inliers from fitted line with most inliers
  std::cout << inliersResult.size() << std::endl;
  return inliersResult;

}*/

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  pcl::PointXYZ pointOne;
  pcl::PointXYZ pointTwo;
  pcl::PointXYZ pointThree;
  pcl::PointXYZ reference_point;
  float ACoeff, BCoeff, CCoeff, DCoeff;
  float dist;
  srand(time(NULL));
  std::cout << "size: " << cloud->points.size() << cloud->width << std::endl;
  // TODO: Fill in this function
  for (int iter = 0; iter < maxIterations; ++iter)
  {

    std::unordered_set<int> inliers;


    while (inliers.size() < 3)
    {
      inliers.insert((rand() % cloud->points.size()));
    }
    auto reference = inliers.begin();

    reference_point = cloud->points[*reference];
    reference++;
    pointOne = cloud->points[*reference];
    reference++;
    pointTwo = cloud->points[*reference];
 


    ACoeff = ((pointOne.y - reference_point.y) * (pointTwo.z - reference_point.z)) - ((pointOne.z - reference_point.z) * (pointTwo.y - reference_point.y)); //(y2−y1)(z3−z1)−(z2−z1)(y3−y1)
    BCoeff = ((pointOne.z - reference_point.z) * (pointTwo.x - reference_point.x)) - ((pointOne.x - reference_point.x) * (pointTwo.z - reference_point.z)); //(z2−z1)(x3−x1)−(x2−x1)(z3−z1),
    CCoeff = ((pointOne.x * reference_point.x) * (pointTwo.y - reference_point.y)) - ((pointOne.y * reference_point.y) * (pointTwo.x - reference_point.z)); //(x2-x1)(y3-y1)-(y2-y1)(x3-x1)
    DCoeff = -1 * ((ACoeff * reference_point.x) + (BCoeff * reference_point.y) + (CCoeff * reference_point.z));
    //std::cout << "dist" << dist <<  std::endl;
    for (int point = 0; point < cloud->points.size(); ++point)
    {

      if (inliers.count(point) > 0 )
      {
        //std::cout << "dist" << dist <<  std::endl;
        continue;
      }
      pointThree = cloud->points[point];
      dist = (fabs)(ACoeff * pointThree.x + BCoeff * pointThree.y + CCoeff * pointThree.z + DCoeff) / sqrt(ACoeff * ACoeff + BCoeff * BCoeff + CCoeff * CCoeff);
      std::cout << "distance : " << dist <<  std::endl;
      if (dist <= distanceTol)
      {
        std::cout << "Adding Point" << std::endl;
        inliers.insert(point);
      }
    }

    if (inliersResult.size() < inliers.size())
    {
      inliersResult = inliers;
    }

  }

  std::cout << inliersResult.size() << std::endl;
  return inliersResult;

}


int main ()
{

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


  // TODO: Change the max iteration and distance tolerance arguments for Ransac function
  std::unordered_set<int> inliers = Ransac(cloud, 200, 0.2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++)
  {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }


  // Render 2D point cloud with inliers and outliers
  if (inliers.size())
  {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  }
  else
  {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce ();
  }

}
