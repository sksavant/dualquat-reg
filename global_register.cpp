#include "global_register.h"

#define NUM_CLOUDS 10
#define NUM_PAIRS 7

double harris_radius = 0.01;
int* current_pair;

static std::string cloud_names[NUM_CLOUDS]={"bun000", "bun045", "bun090", "bun180", "bun270", "bun315", "chin", "ear_back", "top2", "top3"};

int ar[][2] = { {0,1}, {1,2}, {0,2}, {2,3}, {3,4} , {4,5} ,{5,0} };
std::vector<int*> cloud_pairs(ar, ar+sizeof(ar)/sizeof(ar[0]));

GlobalDQReg::GlobalDQReg(){

    loadPCDFiles();
}

void GlobalDQReg::loadPCDFiles(){
    std::cerr << "loadPCDFiles : Loading PCD Files\n";

    for (int i=0; i<NUM_CLOUDS; i++){
        pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>);
        pcl::io::loadPCDFile("data/"+cloud_names[i]+"_UnStructured.pcd", *cloud);
        bunny_clouds_.push_back(cloud);
        PCL_INFO("loadPCDFiles : Cloud %s has %d points\n", cloud_names[i].c_str(), cloud->width);
    }

}

void GlobalDQReg::saveCloudKeyPoints(std::string file_name, int cloud_index){
    int i = cloud_index;
    pcl::PointCloud<Point>::Ptr cloud_1 = bunny_clouds_[i];
    PCL_INFO("saveAllKeyPoints: Computing keypoints of %s\n", cloud_names[i].c_str());
    PCL_INFO("saveAllKeyPoints : Cloud %s has %d points\n", cloud_names[i].c_str(), bunny_clouds_[i]->width);

    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::KdTree<Point>::Ptr tree_1(new pcl::search::KdTree<Point>());
    pcl::HarrisKeypoint3D<Point, pcl::PointXYZI, pcl::PointNormal>::Ptr hkp_1(new pcl::HarrisKeypoint3D<Point, pcl::PointXYZI, pcl::PointNormal>);

    hkp_1->setRadius(harris_radius);
    hkp_1->setSearchMethod(tree_1);
    hkp_1->setInputCloud(cloud_1);
    hkp_1->compute(*keypoints_1);

    PCL_INFO("saveAllKeyPoints: Saving %d keypoints to %s\n", keypoints_1->points.size(), file_name.c_str());
    pcl::io::savePCDFileASCII(file_name, *keypoints_1);
}

void GlobalDQReg::saveAllKeyPoints(std::string folder_name){
    for (int i=0; i<NUM_CLOUDS; ++i){
        std::string file_name = folder_name+"/"+cloud_names[i]+"_kp.pcd";
        saveCloudKeyPoints(file_name, i);
    }
}

void GlobalDQReg::getTransformOfPair(pcl::PointCloud<Point>::Ptr& cloud_1, pcl::PointCloud<Point>::Ptr& cloud_2, Eigen::Matrix4f& tranform)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_1(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_2(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::search::KdTree<Point>::Ptr tree_1(new pcl::search::KdTree<Point>());
    pcl::search::KdTree<Point>::Ptr tree_2(new pcl::search::KdTree<Point>());
    // TODO
    // KeyPoints
    pcl::HarrisKeypoint3D<Point, pcl::PointXYZI, pcl::PointNormal> hkp_1;
    pcl::HarrisKeypoint3D<Point, pcl::PointXYZI, pcl::PointNormal> hkp_2;

    hkp_1.setRadius(harris_radius);
    hkp_1.setSearchMethod(tree_1);
    hkp_1.setInputCloud(cloud_1);
    hkp_1.compute(*keypoints_1);
    PCL_INFO("getTransformOfPair: (%s) : Found keypoints :%d\n", cloud_names[current_pair[0]].c_str(), keypoints_1->points.size());

    hkp_2.setRadius(harris_radius);
    hkp_2.setSearchMethod(tree_2);
    hkp_2.setInputCloud(cloud_2);
    hkp_2.compute(*keypoints_2);
    PCL_INFO("getTransformOfPair: (%s) : Found keypoints :%d\n", cloud_names[current_pair[1]].c_str(), keypoints_2->points.size());

    // Features


    // SACIA


    // ICP
}

void GlobalDQReg::pairwiseRegister()
{
    PCL_INFO("pairwiseRegister: Finding Pairwise registration for %d pairs\n", cloud_pairs.size());
    pairwise_transformations_.clear();
    for (size_t i=0; i<cloud_pairs.size(); ++i){
        // Get a Pair and do stuff
        Eigen::Matrix4f t;
        int* current_pair = cloud_pairs[i];
        PCL_INFO("pairwiseRegister: Pairwise registration of %s and %s\n", cloud_names[current_pair[0]].c_str(), cloud_names[current_pair[1]].c_str());
        getTransformOfPair(bunny_clouds_[current_pair[0]], bunny_clouds_[current_pair[1]], t);
        pairwise_transformations_.push_back(t);
        break; // Test one first
    }
}

void GlobalDQReg::runDQDiffusion()
{
    PCL_INFO("runDQDiffusion: Running Dual Quaterion diffusion\n");
    // TODO
    // Setup DQs with cloud names and transforms

    // Runn diffusion
}
