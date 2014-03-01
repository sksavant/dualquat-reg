#include "diffusion.hpp"
/*
 * ======================================================================
 *                                DEMO
 * ======================================================================
 *
 * The demo simulates a process in which a collection of rangemaps along a ring
 * topology are pairwise-aligned with an ICP-like process. This process inevitably
 * induces alignment error accumulation on the view graph, which is then globally
 * minimized with the dual quaternion method.
 *
 * For technical details, please refer to the paper:
 *
 * "Multiview Registration via Graph Diffusion of Dual Quaternions".
 * A.Torsello, E.Rodola, and A.Albarelli. Proc. CVPR 2011.
 *
 * Note that the code below saves meshes to disk for visualization purposes. The actual
 * error diffusion step takes place at the call run_diffusion(), and is supposed
 * to be very efficient as it does not depend on mesh data for the optimization.
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

   std::set_terminate(unhandled_exception);
   std::cout << std::fixed << std::setprecision(8);

   srand(time(NULL));

   const std::string prefix(argv[1]);
   const int nranges = atoi(argv[2]);

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
      model.load_mesh_from_ply(prefix + "/model.ply", false);
      resolution = model.get_resolution();
   }
   catch (std::exception& e)
   {
      std::cerr << "[ERROR] Exception caught: " << e.what() << std::endl;
      return 1;
   }

   std::cout << "done." << std::endl;

   vector<mesh*> rangemaps(nranges);
   vector<rigid_motion_t> absolute_ground(nranges);

   // applying the 'absolute_ground' motions to each rangemap brings them
   // to their ground-truth positions

   for (int i=0; i<nranges; ++i)
   {
      std::cout << "Loading rangemap " << i+1 << "/" << nranges << "... " << std::flush;

      try
      {
         rangemaps[i] = new mesh;
         rangemaps[i]->load_mesh_from_ply(prefix + "/view" + ntos(i+1) + ".ply", false);

         load_fine_results(
               prefix + "/view" + ntos(i+1) + ".fine.txt",
               absolute_ground[i].first, absolute_ground[i].second);
      }
      catch (std::exception& e)
      {
         std::cerr << "[ERROR] Exception caught: " << e.what() << std::endl;
         return 1;
      }

      std::cout << std::endl;
   }

   // -------------------------------------------------------
   // Perturb pairwise motions (to simulate the result of an ICP process)
   // -------------------------------------------------------

   vector<rigid_motion_t> pairwise_displaced;
   perturb_pairwise(links_per_view, absolute_ground, 10, 1.0, resolution, pairwise_displaced);

   // for visualization purposes, save the rangemaps as .ply in their perturbed positions

   vector<rigid_motion_t> absolute_displaced(nranges);
   launch_pairwise(links_per_view, pairwise_displaced, absolute_displaced);

   for (int i=0; i<nranges; ++i)
   {
      mesh m;
      m.load_mesh_from_ply(prefix + "/view" + ntos(i+1) + ".ply", false);
      m.apply_rigid_transform(absolute_displaced[i].first, absolute_displaced[i].second);
      m.save_mesh_as_ply(prefix + "/view" + ntos(i+1) + "-displaced.ply");
   }

   // -------------------------------------------------------
   // Run Dual Quaternion Diffusion
   // -------------------------------------------------------

   vector<rigid_motion_t> absolute_diffused(nranges);
   run_diffusion(links_per_view, pairwise_displaced, absolute_diffused);

   // for visualization purposes, save the rangemaps as .ply in their error-diffused positions

   for (int i=0; i<nranges; ++i)
   {
      rangemaps[i]->apply_rigid_transform(absolute_diffused[i].first, absolute_diffused[i].second);

      vector<math3d::color_rgb24> colors(rangemaps[i]->get_n_vertices(), math3d::color_rgb24(rand()%255,rand()%255,rand()%255));
      rangemaps[i]->save_as_ply_colors(prefix + "/view" + ntos(i+1) + "-diffused.ply", colors);
   }

   // Cleanup

   for (int i=0; i<nranges; ++i)
      delete rangemaps[i];

   std::cout << std::endl << "Done." << std::endl;

   return 0;
}
