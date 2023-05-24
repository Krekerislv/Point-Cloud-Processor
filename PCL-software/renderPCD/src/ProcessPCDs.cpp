#include "ProcessPCDs.h"

template<typename PointType>
ProcessPCDs<PointType>::ProcessPCDs() {}

template<typename PointType>
ProcessPCDs<PointType>::~ProcessPCDs() {}



template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr ProcessPCDs<PointType>::loadCloud(std::string path_to_pcd) {
    typename pcl::PointCloud<PointType>::Ptr point_cloud (new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType> (path_to_pcd, *point_cloud) == -1) {
        std::cerr<< "Couldn't read file " << path_to_pcd << "\n";
    }
    std::cout << "Loaded " << point_cloud->width * point_cloud->height << " data points from " << path_to_pcd << "\n";
    return point_cloud;
}


template<typename PointType>
std::vector<boost::filesystem::path> ProcessPCDs<PointType>::streamPcd(std::string pcdDirectory) {
    std::vector<boost::filesystem::path> pcdPaths;
    for (const auto & file : boost::filesystem::directory_iterator(pcdDirectory)) {
        if (file.path().extension() == ".pcd") {
            pcdPaths.push_back(file.path());
        }
    }
    sort(pcdPaths.begin(), pcdPaths.end()); //sorts files in ascending order
    return pcdPaths;
}

template<typename PointType>
void ProcessPCDs<PointType>::findMinMaxPoints(typename pcl::PointCloud<PointType> &cloud) {
    pcl::getMinMax3D(cloud, tempMinPt, tempMaxPt);
    if (tempMinPt.x < globalMinX) { globalMinX = tempMinPt.x; }
    if (tempMaxPt.x > globalMaxX) { globalMaxX = tempMaxPt.x; }

    if (tempMinPt.y < globalMinY) { globalMinY = tempMinPt.y; }
    if (tempMaxPt.y > globalMaxY) { globalMaxY = tempMaxPt.y; }

    if (tempMinPt.z < globalMinZ) { globalMinZ = tempMinPt.z; }
    if (tempMaxPt.z > globalMaxZ) { globalMaxZ = tempMaxPt.z; }
}

