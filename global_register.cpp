#include "global_register.h"

GlobalDQReg::GlobalDQReg(){

    loadPCDFiles();
}

void GlobalDQReg::loadPCDFiles(){
    std::cerr << "Loading PCD Files\n";
    static std::string names[10]={"bun000", "bun045", "bun090", "bun180", "bun270", "bun315", "chin", "ear_back", "top2", "top3"};

    pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>);
    for (int i=0; i<10; i++){
        pcl::io::loadPCDFile("data/"+names[i]+"_UnStructured.pcd", *cloud);
        bunny_clouds_.push_back(cloud);
    }

}

void GlobalDQReg::pairwiseRegister()
{
    std::cerr << "Finding Pairwise registration\n";
    // TODO

}
void GlobalDQReg::runDQDiffusion()
{
    std::cerr << "Running Dual Quaterion diffusion\n";
    // TODO
}
