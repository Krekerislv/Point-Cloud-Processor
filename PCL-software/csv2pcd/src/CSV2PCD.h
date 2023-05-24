#ifndef CSV2PCD_H_
#define CSV2PCD_H_

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sstream>
#include <fstream>




class CSV2PCD {
    public:

        std::string input_path;
        long int PPC;
        std::string output_path;

        CSV2PCD(std::string user_input_path);
        ~CSV2PCD();
        
        void writeToPCD(std::string outputDir , long int PPC);

    private:
        std::ifstream input_stream;
        int digitCount(int x);
        void initCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, long points_per_cloud);
        std::vector<float> get_Max_csv_values();
        std::vector<std::string> split_string(std::string str, std::string deli);
        std::string multiplyString(std::string a, unsigned int b);

        std::ifstream getInputFileStream(std::string input_path);
        long int getPointsPerCloud(long int PPC);
        std::string getOutputDirectory(std::string output_path);
};



#endif /* CSV2PCD_H_ */