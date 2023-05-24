#include "CSV2PCD.h"


bool validateArgs(int argc, char** argv) {
    /*
    Possible cases of user input:
        wrong number of arguments
        points_per_cloud is NaN (atol returns 0 -> CSV2PCD handles it)
        wrong input_file -> CSV2PCD handles it
        wrong output_path -> CSV2PCD handles it
        ...?
    */
    if ((argc != 4)){
        return false;
    }
    

    return true;
}


int main(int argc, char** argv) {
    if (validateArgs(argc, argv)) {

        CSV2PCD processor(argv[1]);
        processor.writeToPCD(argv[3],atol(argv[2]));

    } else {
        std::cout << "Invalid input arguments!\nUsage: ./csv2pcd *.csv_file points_per_cloud output_directory!\n";
        return 0;
    }


}