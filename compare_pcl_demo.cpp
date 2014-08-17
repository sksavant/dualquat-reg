#include <diffusion.hpp>

#include <pcl/registration/dq_diffusion.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>

/*
 * ======================================================================
 *                                DEMO TEST
 * ======================================================================
 *
 * This demo compares the results of the PCLized code and the demo code from dualquat/
 *
*/

int main(int argc, char** argv)
{
  if (argc != 3)
  {
     std::cout << "Usage: ./demo <data_dir> <n_rangemaps>" << std::endl;
#ifdef WIN32
     system("pause");
#endif
     return 0;
  }

  std::set_terminate (unhandled_exception);
  std::cout << std::fixed << std::setprecision (8);

  srand (time (NULL));

  const std::string prefix (argv[1]);
  const int nranges = atoi (argv[2]);

  const int links_per_view = 1;

  // -------------------------------------------------------
  // Load all the rangemaps and their ground-truth motions
  // -------------------------------------------------------

  // first get the average resolution of the mesh, this is needed to perturb
  // the translation component of the rigid motions in a reasonable measure

  std::cout << "Loading model mesh... " << std::flush;

  double resolution = 0.;
  try
  {
     mesh model;
     model.load_mesh_from_ply (prefix + "/model.ply", false);
     resolution = model.get_resolution ();
  }
  catch (std::exception& e)
  {
    std::cerr << "[ERROR] Exception caught: " << e.what () << std::endl;
    return 1;
  }

  std::cout << "done." << std::endl;

  vector<mesh*> rangemaps (nranges);
  vector<rigid_motion_t> absolute_ground (nranges);

  // applying the 'absolute_ground' motions to each rangemap brings them
  // to their ground-truth positions

  for (int i=0; i<nranges; ++i)
  {
    std::cout << "Loading rangemap " << i+1 << "/" << nranges << "... " << std::flush;

    try
    {
      rangemaps[i] = new mesh;
      rangemaps[i]->load_mesh_from_ply (prefix + "/view" + ntos (i+1) + ".ply", false);

      load_fine_results (
               prefix + "/view" + ntos (i+1) + ".fine.txt",
               absolute_ground[i].first, absolute_ground[i].second);
    }
    catch (std::exception& e)
    {
       std::cerr << "[ERROR] Exception caught: " << e.what () << std::endl;
       return 1;
    }

    std::cout << std::endl;
  }

  // -------------------------------------------------------
  // Perturb pairwise motions (to simulate the result of an ICP process)
  // -------------------------------------------------------

  vector<rigid_motion_t> pairwise_displaced;
  perturb_pairwise (links_per_view, absolute_ground, 10, 1.0, resolution, pairwise_displaced);

  // for visualization purposes, save the rangemaps as .ply in their perturbed positions

  vector<rigid_motion_t> absolute_displaced (nranges);
  launch_pairwise (links_per_view, pairwise_displaced, absolute_displaced);

  for (int i = 0; i < nranges; ++i)
  {
    mesh m;
    m.load_mesh_from_ply (prefix + "/view" + ntos (i+1) + ".ply", false);
    m.apply_rigid_transform (absolute_displaced[i].first, absolute_displaced[i].second);
    m.save_mesh_as_ply (prefix + "/view" + ntos (i+1) + "-displaced.ply");
  }

  // ------------------------------------------------------
  // PCL Initialization
  // ------------------------------------------------------
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

  pcl::registration::DQDiffusion<pcl::PointXYZ> dq; // #PCL : Init dq
  std::vector<PointCloudPtr> cloud_ptrs;
  std::cerr << "PCL: Loading Displaced files into PCL\n";

  for (int i = 0; i < nranges; ++i)
  {
    pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh ());
    pcl::io::loadPolygonFilePLY (prefix + "/view" + ntos(i+1) + "-displaced.ply", *mesh);
    PointCloudPtr pc (new PointCloud ());
    pcl::fromPCLPointCloud2(mesh->cloud, *pc);
    cloud_ptrs.push_back (pc);
    std::cerr << "PCL : Cloud " << i << " loaded : Width " << pc->width << "\n";
    dq.addPointCloud (pc);
  }

  for (int i = 0; i < nranges * links_per_view; ++i)
  {
    int next = (i == nranges - 1 ? 0 : i + 1);
    Eigen::Matrix4f transform;
    for (int r = 0; r < 3; ++r){
      for (int c = 0; c < 3; ++c){
        transform (r,c) = pairwise_displaced[i].first (r,c);
      }
    }
    transform (0,3) = pairwise_displaced[i].second.x;
    transform (1,3) = pairwise_displaced[i].second.y;
    transform (2,3) = pairwise_displaced[i].second.z;
    transform (3,3) = 0;

    dq.addPairwiseTransformation (i, next, transform);
  }

  dq.setLinearApproximation (true);
  std::cerr << "PCL : Initial error is " << std::endl;//dq.getFitnessScore () << std::endl;
  dq.compute ();
  std::cerr << "PCL : Final error is " << dq.getFitnessScore () << std::endl;

  // -------------------------------------------------------
  // Run Dual Quaternion Diffusion
  // -------------------------------------------------------

  vector<rigid_motion_t> absolute_diffused (nranges);
  run_diffusion (links_per_view, pairwise_displaced, absolute_diffused);

  // for visualization purposes, save the rangemaps as .ply in their error-diffused positions

  for (int i = 0; i < nranges; ++i)
  {
    rangemaps[i]->apply_rigid_transform (absolute_diffused[i].first, absolute_diffused[i].second);

    vector<math3d::color_rgb24> colors (rangemaps[i]->get_n_vertices (), math3d::color_rgb24 (rand ()%255,rand ()%255,rand ()%255));
    rangemaps[i]->save_as_ply_colors (prefix + "/view" + ntos (i+1) + "-diffused.ply", colors);
  }

  // Cleanup

  for (int i = 0; i < nranges; ++i)
    delete rangemaps[i];

  std::cout << std::endl << "Done." << std::endl;

  return 0;
}
