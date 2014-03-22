#include "global_register.h"

#define NUM_CLOUDS 10
#define NUM_CONNECTED 6
#define NUM_PAIRS 7

double harris_radius = 0.005;
double normal_estimation_radius = 0.01;
double fpfh_estimation_radius = 0.02;
double min_sample_distance = 0.1;
double max_correspondence_distance = 0.1*0.1;
int nr_iterations = 500;
int* current_pair;

static std::string cloud_names[NUM_CLOUDS]={"bun000", "bun045", "bun090", "bun180", "bun270", "bun315", "chin", "ear_back", "top2", "top3"};

int ar[][2] = { {0,1}, {1,2}, {0,2}, {2,3}, {3,4} , {4,5} ,{5,0} };
std::vector<int*> cloud_pairs(ar, ar+sizeof(ar)/sizeof(ar[0]));

pcl::PointCloud<pcl::PointXYZ>::Ptr toPointXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i=0; i<cloud->width; ++i){
        pcl::PointXYZ new_point;
        new_point.x = cloud->points[i].x;
        new_point.y = cloud->points[i].y;
        new_point.z = cloud->points[i].z;
        new_cloud->points.push_back(new_point);
    }
    new_cloud->width = cloud->width;
    new_cloud->height = 1;
    return new_cloud;
}

std::vector<int> findNearestPointIndices(Point& searchPoint, pcl::PointCloud<Point>::Ptr& cloud, pcl::KdTreeFLANN<Point>& kdtree, int K){
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
        return pointIdxNKNSearch;
    }
    pointIdxNKNSearch.clear();
    return pointIdxNKNSearch;
}

GlobalDQReg::GlobalDQReg() :
dq_transducer(NUM_CONNECTED)
{
    loadPCDFiles();
}

void GlobalDQReg::loadPCDFiles(){
    PCL_INFO("loadPCDFiles : Loading PCD Files\n");

    for (int i=0; i<NUM_CLOUDS; i++){
        pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>);
        pcl::PointCloud<Point>::Ptr filtered_cloud(new pcl::PointCloud<Point>);
        std::vector<int> mapping;

        pcl::io::loadPCDFile("data/"+cloud_names[i]+"_UnStructured.pcd", *cloud);
        pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, mapping);
        bunny_clouds_.push_back(cloud);
        PCL_INFO("loadPCDFiles : Cloud %s has %d points\n", cloud_names[i].c_str(), cloud->width);
    }

}

void GlobalDQReg::computeHarrisKeyPoint(pcl::PointCloud<Point>::Ptr& cloud, pcl::PointCloud<Point>::Ptr& out_keypoints, int cloud_id)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> mapping;
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
    pcl::HarrisKeypoint3D<Point, pcl::PointXYZI, pcl::PointNormal> hkp;

    hkp.setRadius(harris_radius);
    hkp.setSearchMethod(tree);
    hkp.setInputCloud(cloud);
    hkp.compute(*keypoints_temp);
    pcl::removeNaNFromPointCloud(*keypoints_temp, *keypoints, mapping);

    out_keypoints = toPointXYZ(keypoints);
    PCL_INFO("computeHarrisKeyPoint: (%s) : Found keypoints :%d\n", cloud_names[cloud_id].c_str(), out_keypoints->points.size());

}

void GlobalDQReg::computeFPFHFeatures(pcl::PointCloud<Point>::Ptr& cloud, pcl::PointCloud<Point>::Ptr& keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhs, int cloud_id)
{
    PCL_INFO("computeFPFHFeatures: Found %d keypoints of %s\n", keypoints->points.size(), cloud_names[cloud_id].c_str());
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
    pcl::FPFHEstimationOMP<Point, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<Point, pcl::Normal> normal_estimation;

    //std::clock_t start_normal = std::clock();
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch(normal_estimation_radius);
    normal_estimation.compute(*normals);
    //double time = (std::clock() - start_normal)/(double) CLOCKS_PER_SEC;
    //std::cerr << "Normal estimation time: " << time << " ms\n";
    fpfh.setSearchSurface(cloud);
    fpfh.setInputNormals(normals);
    fpfh.setInputCloud(keypoints);
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(fpfh_estimation_radius);
    fpfh.compute(*fpfhs);
    //time = (std::clock() - start_normal)/(double) CLOCKS_PER_SEC;
    //std::cerr << "FPFH compute time: " << time << " ms\n";
    PCL_INFO("computeFPFHFeatures: Found FPFH features at %d keypoints of %s\n", fpfhs->points.size(), cloud_names[cloud_id].c_str());
}

void GlobalDQReg::saveFilesofCloud(int cloud_id, pcl::PointCloud<Point>::Ptr& cloud, pcl::PointCloud<Point>::Ptr& keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhs)
{
    pcl::io::savePCDFileASCII("data/run/"+cloud_names[cloud_id]+".pcd", *cloud);
    pcl::io::savePCDFileASCII("data/run/"+cloud_names[cloud_id]+"_kp.pcd", *keypoints);
    pcl::io::savePCDFileASCII("data/run/"+cloud_names[cloud_id]+"_fpfh.pcd", *fpfhs);
}

void GlobalDQReg::getTransformOfPair(pcl::PointCloud<Point>::Ptr& cloud_1, pcl::PointCloud<Point>::Ptr& cloud_2, Eigen::Matrix4f& transform)
{
    // TODO
    // KeyPoints
    pcl::PointCloud<Point>::Ptr keypoints_1(new pcl::PointCloud<Point>);
    pcl::PointCloud<Point>::Ptr keypoints_2(new pcl::PointCloud<Point>);

    computeHarrisKeyPoint(cloud_1, keypoints_1, current_pair[0]);
    computeHarrisKeyPoint(cloud_2, keypoints_2, current_pair[1]);

    // Features

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_1(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_2(new pcl::PointCloud<pcl::FPFHSignature33>);

    computeFPFHFeatures(cloud_1, keypoints_1, fpfhs_1, current_pair[0]);
    computeFPFHFeatures(cloud_2, keypoints_2, fpfhs_2, current_pair[1]);

    // SACIA
    pcl::SampleConsensusInitialAlignment<Point, Point, pcl::FPFHSignature33> sac_ia;
    pcl::PointCloud<Point> ransaced_source;
    pcl::PointCloud<Point>::Ptr transformed_cloud_2(new pcl::PointCloud<Point>);
    pcl::KdTreeFLANN<Point> kdtree;
    Eigen::Matrix4f transformation_2_1;
    Eigen::Matrix4f transformation_temp;
    float fitness_score;
    float min_fitness_score = FLT_MAX;
    float min_error = FLT_MAX;
    float max_error = 0.0;
    float error = 0.0;
    sac_ia.setMinSampleDistance(min_sample_distance);
    //sac_ia.setMaxCorrespondenceDistance(max_correspondence_distance);
    sac_ia.setMaximumIterations(nr_iterations);

    sac_ia.setInputSource(keypoints_2);
    sac_ia.setSourceFeatures(fpfhs_2);
    sac_ia.setInputTarget(keypoints_1);
    sac_ia.setTargetFeatures(fpfhs_1);

    saveFilesofCloud(current_pair[0], cloud_1, keypoints_1, fpfhs_1);
    saveFilesofCloud(current_pair[1], cloud_2, keypoints_2, fpfhs_2);

    int count = 0;
    int max_count = 15;
    while (count<max_count){
        sac_ia.align(ransaced_source);
        fitness_score = sac_ia.getFitnessScore(max_correspondence_distance);
        transformation_temp = sac_ia.getFinalTransformation();
        PCL_INFO("Pointclouds aligned, fitness score (with keypoints only) is :%f\n", fitness_score);

        error = 0.0;

        pcl::transformPointCloud(*cloud_2, *transformed_cloud_2, transformation_temp);
        kdtree.setInputCloud(cloud_1);
        fitness_score = 0.0;

        for (size_t i=0; i<transformed_cloud_2->width; ++i){
            Point searchPoint = transformed_cloud_2->points[i];
            int point_index = findNearestPointIndices(searchPoint, cloud_1, kdtree, 1)[0];
            Point resultPoint = cloud_1->points[point_index];
            Eigen::Vector4d p1 = Eigen::Vector4d (searchPoint.x, searchPoint.y, searchPoint.z, 0);
            Eigen::Vector4d p2 = Eigen::Vector4d (resultPoint.x, resultPoint.y, resultPoint.z, 0);
            error = (p1-p2).squaredNorm();
            if (sqrt(error) < max_correspondence_distance){
                error = error/2.0;
            }else{
                error = max_correspondence_distance*(sqrt(error)-max_correspondence_distance/2.0);
            }
            if (error > max_error){
                max_error = error;
            }
            if (error < min_error){
                min_error = error;
            }
            fitness_score += fabs(error);
        }
        float avg_error = fitness_score/transformed_cloud_2->width;
        PCL_INFO("Average error (huber fitness score) is %f\n",avg_error);
        if (avg_error <= min_fitness_score){
            min_fitness_score = avg_error;
            transformation_2_1 = transformation_temp;
        }
        count++;
    }
    transform = transformation_2_1;
    // ICP
}

void GlobalDQReg::pairwiseRegister()
{
    PCL_INFO("pairwiseRegister: Finding Pairwise registration for %d pairs\n", cloud_pairs.size());
    pairwise_transformations_.clear();
    for (size_t i=0; i<cloud_pairs.size(); ++i){
        // Get a Pair and do stuff
        Eigen::Matrix4f t;
        current_pair = cloud_pairs[i];
        PCL_INFO("pairwiseRegister: Pairwise registration of %s and %s\n", cloud_names[current_pair[0]].c_str(), cloud_names[current_pair[1]].c_str());
        getTransformOfPair(bunny_clouds_[current_pair[0]], bunny_clouds_[current_pair[1]], t);
        pairwise_transformations_.push_back(t);
        //break; // Test one first
    }
}

void GlobalDQReg::runDQDiffusion()
{
    PCL_INFO("runDQDiffusion: Running Dual Quaterion diffusion\n");
    // TODO
    // Setup DQs with cloud names and transforms

    math3d::quaternion<double> R;
    point3d t;
    Eigen::Matrix4f transform;
    double rot[9];
    for (int i=0; i<NUM_PAIRS; ++i){
        transform = pairwise_transformations_[i];
        for (int j=0; j<3; ++j){
            for (int k=0; k<3; ++k){
                rot[j+3*k] = (float)transform(j,k);
            }
        }
        R = math3d::rot_matrix_to_quaternion(math3d::matrix3x3<double>(rot));
        t = math3d::vec3d<double>(transform(0,3), transform(1,3), transform(2,3));
        dq_transducer.add_transformation(cloud_pairs[i][0], cloud_pairs[i][1], R, t, 1.0);
    }

    dq_transducer.get_estimate();
    PCL_INFO("runDQDiffusion: Initial Error before diffusion is %f\n", dq_transducer.rmste());

    // Runn diffusion
    dq_transducer.linear_transduce();
    PCL_INFO("runDQDiffusion: Error after diffusion is %f\n", dq_transducer.rmste());
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
