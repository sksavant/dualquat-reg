#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/io/pcd_io.h>

#include "dualquat/dual_quaternions.h"
#include "dualquat/mesh.h"
#include "dualquat/math3d.h"

typedef pcl::PointXYZ Point;

class GlobalDQReg {
  public:
    GlobalDQReg();
    void pairwiseRegister();
    void runDQDiffusion();

  private:
    std::vector<pcl::PointCloud<Point>::Ptr > bunny_clouds_;
    void loadPCDFiles();

};
