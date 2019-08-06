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
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT> ());

    pcl::CropBox< PointT > region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);

    std::vector<int> indices;
    pcl::CropBox< PointT > roof(true);
    roof.setMin(Eigen::Vector4f(-3, -3, -3, 1));
    roof.setMax(Eigen::Vector4f( 3,  3,  3, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);


    pcl::PointIndices::Ptr inliers (new  pcl::PointIndices);
    for (auto point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_region);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_region);





    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
void ProcessPointClouds<PointT>::Ransac3D( typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices::Ptr inliers_point, int maxIterations, float distanceTol)
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
    for (auto iter : inliersResult)
    {
        inliers_point->indices.push_back(iter);
    }

    std::cout << inliersResult.size() << std::endl;
    return ;

}




template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeClouds (new typename pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new typename pcl::PointCloud<PointT> ());
    for (int i = 0; i < inliers->indices.size(); ++i)
    {
        planeClouds->points.push_back(cloud->points[i]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacleCloud);
    //std::cout << "dataset." << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeClouds);
    return segResult;
}




template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr inliers_point {new pcl::PointIndices};


    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    PointT pointOne;
    PointT pointTwo;
    PointT pointThree;
    PointT reference_point;
    float ACoeff, BCoeff, CCoeff, DCoeff;
    float dist;
    srand(time(NULL));
    //std::cout << "size: " << cloud->points.size() << cloud->width << std::endl;
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
            // std::cout << "distance : " << dist <<  std::endl;
            if (dist <= distanceThreshold)
            {
                // std::cout << "Adding Point" << std::endl;
                inliers.insert(point);
            }
        }

        if (inliersResult.size() < inliers.size())
        {
            inliersResult = inliers;
        }

    }
    for (auto iter : inliersResult)
    {
        inliers_point->indices.push_back(iter);
    }


    if (inliers_point->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers_point, cloud);

    auto endTime = std::chrono::steady_clock::now();
    //std::cout << "end time" << endTime.count() << std::endl;

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}

/*template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //std::cout << "start time" << startTime.count() << std::endl;
    //pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    //std::cout << "a planar model for the given dataset." << std::endl;
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    auto endTime = std::chrono::steady_clock::now();
    //std::cout << "end time" << endTime.count() << std::endl;

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}*/



/*template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>());

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}*/
template<typename PointT>
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::unordered_set<int> &point_index, int id, float distanceTol, KdTree3D<PointT>* tree)
{
    point_index.insert(id);

    //std::cout << "size:"  << point_index.size() << " id:" << id << std::endl;
    cluster.push_back(id);
    //std::cout << "id" << id << std::endl;
    std::vector<int> nearby = tree->search(cloud->points[id], distanceTol);
    //std::cout << "nearby:"  << nearby.size() << "point id:" << id << std::endl;
    for (int iter = 0; iter < nearby.size(); iter++)
    {
        //std::cout << "nearby point:"  << nearby[iter] << "iter id:" << iter << std::endl;
        if (point_index.count(nearby[iter]) == 0)
        {
            //std::cout << "New id" << id << std::endl;
            proximity(cloud, cluster, point_index, nearby[iter] , distanceTol, tree);
        }

    }

    return;

}

//std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>:: euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{
    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::unordered_set<int> point_index;

    KdTree3D<PointT>* tree = new KdTree3D<PointT>();
    for (int i = 0; i < cloud->points.size(); i++)
    {

        tree->insert(cloud->points[i], i);

    }

    for (int id = 0; id < cloud->points.size(); ++id)
    {
        if (point_index.count(id) > 0)
        {
            continue;
        }
        //typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>());
        std::vector<int> cluster;
        proximity(cloud, cluster, point_index, id, distanceTol, tree);

        if ((cluster.size() >= minSize) && (cluster.size() <= maxSize))
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>());

            for (int cluster_index : cluster)
            {
                cloud_cluster->points.push_back(cloud->points[cluster_index]);
            }
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
        }
        else
        {
            point_index.erase(id);
           // std::cout << "  " << std::endl;
        }
        

    }

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = euclideanCluster( cloud, clusterTolerance, minSize, maxSize );



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
    std::cerr << "Saved " << cloud->points.size () << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from " + file << std::endl;

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

