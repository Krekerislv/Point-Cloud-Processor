#include "ProcessPCDs.h"
#include "ProcessPCDs.cpp"



void initViewer(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    viewer->setBackgroundColor (255, 255, 255);
    viewer->initCameraParameters();
    viewer->resetCamera();
}

void adjustCamera(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPCDs<pcl::PointXYZ>* pcdProcessor) {
    int viewX = (pcdProcessor->globalMinX + pcdProcessor->globalMaxX)/2;
    int viewY = (pcdProcessor->globalMinY + pcdProcessor->globalMaxY)/2;
    int viewZ = (pcdProcessor->globalMinZ + pcdProcessor->globalMaxZ)/2;
    
    //vector at an angle = atan(deltaZ / deltaX) against x axis 
    int deltaX = 10; int deltaZ = 1;
    int camX = viewX + deltaX;
    int camY = viewY;
    int camZ = viewZ + deltaZ;
    viewer->setCameraPosition(camX, camY, camZ, viewX, viewY, viewZ, 0,0,1);

    viewer->resetCamera(); //automatically adjusts vector length to fit clouds in window (adjusts zoom)
}

std::string validateDirectory(int argc, char ** argv) {
    bool exitFlag = false;
    if ((argc != 2)) {
        exitFlag = true;
        std::cout << "Use ./renderCloud directory_of_pcd_files \n";
        exit(0);
    }

    std::string pcd_directory(argv[1]);
    if ((pcd_directory == "--help")) {
        std::cout << "Use ./renderCloud directory_of_pcd_files \n";
        exit(0);
    }

    if (pcd_directory.back() != '/') { pcd_directory += "/";}
    try {
        if (!boost::filesystem::is_directory(pcd_directory)) { 
            std::cout << "Directory does not exist!\n";
            exit(0);       
        }
    } catch (boost::filesystem::filesystem_error & e) {
            std::cout << "Cannot access output directory.\n";
            exit(0); 
    }

    bool warningFlag = false;
    bool pcdExists = false;
    for (const auto & file : boost::filesystem::directory_iterator(pcd_directory)) {
        if ((file.path().extension()!= ".pcd") && !warningFlag) {
            std::cout << "Warning! Directory contains non *.pcd files or folders.\n";
            warningFlag = true;
        } else {
            pcdExists = true;
        }
    }  
    if (pcdExists) {
        return pcd_directory;
    } else {
        std::cout << "No *.pcd files found!\n";
        exit(0);
    }
    
}


int main(int argc, char** argv) {
    std::string validDir = validateDirectory(argc, argv);


    ProcessPCDs<pcl::PointXYZ>* pcdProcessor = new ProcessPCDs<pcl::PointXYZ>();
    std::vector<boost::filesystem::path> stream = pcdProcessor->streamPcd(validDir);
    auto streamIterator = stream.begin();


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    initViewer(viewer);

    


    int cloud_id_int=1; bool setGradientFlag = false;
    while (!viewer->wasStopped ()) {
        if (streamIterator != stream.end()) {
            std::string cloud_id = std::to_string(cloud_id_int++);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcdProcessor->loadCloud((*streamIterator).string());

            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> intensity(cloud,"z");
            viewer->addPointCloud<pcl::PointXYZ> (cloud, intensity, cloud_id);

            pcdProcessor->findMinMaxPoints(*cloud);

            ++streamIterator;
        } else if (!setGradientFlag) { //set gradient color AFTER all clouds have been added. This guarantees correct globalZmin and globalZmax values
            setGradientFlag = true;
            for (int k=1; k < cloud_id_int; k++) {

                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_RANGE,pcdProcessor->globalMinZ,pcdProcessor->globalMaxZ, std::to_string(k));
                /*
                change colormap:
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT, pcl::visualization::LookUpTableRepresentationProperties::PCL_VISUALIZER_LUT_JET_INVERSE, std::to_string(k));
                */
                adjustCamera(viewer, pcdProcessor);
            
            }
        }
        viewer->spinOnce ();
    }
      
}


