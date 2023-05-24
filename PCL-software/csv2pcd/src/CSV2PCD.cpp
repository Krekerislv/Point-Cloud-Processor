#include "CSV2PCD.h"


CSV2PCD:: CSV2PCD(std::string user_input_path) {
    CSV2PCD::input_path = user_input_path;
};
CSV2PCD::~CSV2PCD () {}



std::vector<std::string> CSV2PCD::split_string(std::string str, std::string deli) {
    int begin = 0; int end = str.find(deli); //finds first instance
    std::vector<std::string> split_string;
    while (end != -1) {
        split_string.push_back(str.substr(begin, end - begin));
        begin = end+deli.size();
        end = str.find(deli,begin);
    }
    return split_string;
}


std::ifstream CSV2PCD::getInputFileStream(std::string input_path) {
    std::ifstream  data(input_path, std::ios::in);
    if (data.fail()) {
        std::cout << "Input file does not exist or is inaccessible.\n";
        exit(0);
    } else {
        return data;
    }

}


long int CSV2PCD::getPointsPerCloud(long int PPC) {
    if (PPC <= 0) {
        std::cout << "Points per cloud has to be a positive integer!\n";
        exit(0);
    } else {
        return PPC;
    }
}


std::string CSV2PCD::getOutputDirectory(std::string output_path){

    try {
        boost::filesystem::path outputDirectory(output_path);
        if (!boost::filesystem::exists(outputDirectory)) {
            std::cout << "Output directory does not exist!\nWould you like to create it? (y/n) ";
            std::string input;
            std::cin >> input;
            if ((input == "y") || (input == "Y") ) {
                if (boost::filesystem::create_directories(outputDirectory)) {
                    std::cout << "Directory created sucessfully\n";
                } else {
                    std::cout << "Couldn't create directory!\n";
                    exit(0);
                }
            } else if ((input == "n") || (input == "N") ) {
                std::cout << "No output directory.\n";
                exit(0);
            } else {
                std::cout << "Unexpected input.\n";
                exit(0);
            }
        }
        return outputDirectory.string()+"/"; //it doesn't add "/"  automatically
    } catch (boost::filesystem::filesystem_error & e) {
        throw;
        exit(0);
    }
    

}

void CSV2PCD::initCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, long int points_per_cloud) {
    cloud.clear();
    cloud.width = points_per_cloud;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);
}


std::vector<float> CSV2PCD::get_Max_csv_values() {
    std::cout << "Finding max XYZ values...\n";
    float max_x=std::numeric_limits<float>::min(); 
    float max_y=std::numeric_limits<float>::min();
    float max_z=std::numeric_limits<float>::min();
    std::vector<float> maxXYZ;
    /*
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    */
    
    std::string line;
    while(std::getline(CSV2PCD::input_stream,line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<float> parsedRow;
        while(std::getline(lineStream,cell,',')) {
            parsedRow.push_back(std::stod(cell));
        }
        if (parsedRow[0] > max_x) {
            max_x = parsedRow[0];
        }

        if (parsedRow[1] > max_y) {
            max_y = parsedRow[1];
        }

        if (parsedRow[2] > max_z) {
            max_z = parsedRow[2];
        }
        /*         
        if (parsedRow[0] < min_x) {
            min_x = parsedRow[0];
        }
        if (parsedRow[1] < min_y) {
            min_y = parsedRow[1];
        }
        if (parsedRow[2] < min_z) {
            min_z = parsedRow[2];
        }
        */

        }
        CSV2PCD::input_stream.clear();
        CSV2PCD::input_stream.seekg(0);
        maxXYZ.push_back(max_x);
        maxXYZ.push_back(max_y);
        maxXYZ.push_back(max_z);
        std::cout << "done\n";
        return maxXYZ;

}

int CSV2PCD::digitCount(int x) {
        int count = 0;
    while(x > 0) {
        count++;
        x = x/10;
    }
    return count;
}

std::string CSV2PCD::multiplyString(std::string a, unsigned int b) {
    std::string output = "";
    while (b--) {output += a;}
    return output;
}



void CSV2PCD::writeToPCD(std::string outputDir , long int PPC){
    CSV2PCD::input_stream = getInputFileStream(CSV2PCD::input_path);
    CSV2PCD::PPC = getPointsPerCloud(PPC);
    CSV2PCD::output_path = getOutputDirectory(outputDir);
    std::vector<float> maxXYZ = get_Max_csv_values();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    initCloud(cloud, CSV2PCD::PPC);
    
    int i = 0;
    int outputFileCount = 0;
    std::string line;
    while(std::getline(CSV2PCD::input_stream,line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<float> parsedRow;
        while(std::getline(lineStream,cell,',')) {
            parsedRow.push_back(std::stof(cell));
        }
        cloud[i].x = maxXYZ[0] - parsedRow[0];
        cloud[i].y = maxXYZ[1] - parsedRow[1];
        cloud[i].z = maxXYZ[2] - parsedRow[2];
        i++;
        if(i == CSV2PCD::PPC) {
            i = 0; outputFileCount++;
            std::string fileNo = CSV2PCD::multiplyString("0", 8-digitCount(outputFileCount)) + std::to_string(outputFileCount) + ".pcd";
            pcl::io::savePCDFileBinary (CSV2PCD::output_path + fileNo, cloud);
            std::cout << "Saved " << cloud.size() << " data points to: " << CSV2PCD::output_path + fileNo << "\n";
            initCloud(cloud, CSV2PCD::PPC);
        }
        
    }
    CSV2PCD::input_stream.close();
    if (i!=0) { //writes last cloud which (almost) always is smaller
        initCloud(cloud,i+1);
        std::string fileNo = CSV2PCD::multiplyString("0", 8-digitCount(outputFileCount+1)) + std::to_string(outputFileCount+1) + ".pcd";
        pcl::io::savePCDFileBinary (CSV2PCD::output_path + fileNo, cloud);
        std::cout << "Saved " << cloud.size() << " data points to: " << CSV2PCD::output_path + fileNo << "\n";
        
    }
}
