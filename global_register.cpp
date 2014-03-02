#include "global_register.h"

#define NUM_CLOUDS 10
#define NUM_PAIRS 

static std::string cloud_names[NUM_CLOUDS]={"bun000", "bun045", "bun090", "bun180", "bun270", "bun315", "chin", "ear_back", "top2", "top3"};

int ar[][2] = { {0,1}, {1,2}, {0,2}, {2,3}, {3,4} , {4,5} ,{5,0} };
std::vector<int*> cloud_pairs(ar, ar+sizeof(ar)/sizeof(ar[0]));

GlobalDQReg::GlobalDQReg(){

    loadPCDFiles();
}

void GlobalDQReg::loadPCDFiles(){
    std::cerr << "Loading PCD Files\n";

    pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>);
    for (int i=0; i<NUM_CLOUDS; i++){
        pcl::io::loadPCDFile("data/"+cloud_names[i]+"_UnStructured.pcd", *cloud);
        bunny_clouds_.push_back(cloud);
        PCL_INFO("loadPCDFiles : Cloud %s has %d points\n", cloud_names[i].c_str(), cloud->width);
    }

}

void GlobalDQReg::getTransformOfPair(pcl::PointCloud<Point>::Ptr& cloud_1, pcl::PointCloud<Point>::Ptr& cloud_2, Eigen::Matrix4f& tranform)
{
    // TODO
    // KeyPoints

    // Features

    // SACIA

    // ICP
}

void GlobalDQReg::pairwiseRegister()
{
    PCL_INFO("pairwiseRegister : Finding Pairwise registration for %d pairs\n", cloud_pairs.size());
    pairwise_transformations_.clear();
    for (size_t i=0; i<cloud_pairs.size(); ++i){
        // Get a Pair and do stuff
        Eigen::Matrix4f t;
        int* pair = cloud_pairs[i];
        getTransformOfPair(bunny_clouds_[pair[0]], bunny_clouds_[pair[1]], t);
        pairwise_transformations_.push_back(t);
    }

}
void GlobalDQReg::runDQDiffusion()
{
    PCL_INFO("runDQDiffusion: Running Dual Quaterion diffusion\n");
    // TODO
    // Setup DQs with cloud names and transforms

    // Runn diffusion
}
