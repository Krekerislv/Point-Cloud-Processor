#ifndef PROCESSPCDS_H_
#define PROCESSPCDS_H_

#include <iostream>
#include <thread>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>



template<typename PointType>
class ProcessPCDs {
    public:
        ProcessPCDs();
        ~ProcessPCDs();
        double globalMinX = std::numeric_limits<float>::max(); //sets start value
        double globalMaxX = std::numeric_limits<float>::min();
        double globalMinY = std::numeric_limits<float>::max(); 
        double globalMaxY = std::numeric_limits<float>::min();
        double globalMinZ = std::numeric_limits<float>::max(); 
        double globalMaxZ = std::numeric_limits<float>::min();

        typename pcl::PointCloud<PointType>::Ptr loadCloud(std::string path_to_pcd);
        std::vector<boost::filesystem::path> streamPcd(std::string pcdDirectory);

        void findMinMaxPoints(pcl::PointCloud<PointType> &cloud);

        private:
            PointType tempMinPt;
            PointType tempMaxPt;

};

#endif /* PROCESSPCDS_H_ */