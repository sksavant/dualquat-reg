/**
 * @date   06 October 2013
 * @author Emanuele Rodola <rodola@in.tum.de>
 */

#include "dual_quaternions.h"
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include "mesh.h"
#include <fstream>
#include <ctime>

using std::vector;
using math3d::matrix3x3;
using math3d::point3d;
using math3d::rigid_motion_t;

/**
 * Generator of pseudo-random numbers according to a normal distribution
 * with given mean and variance.
 * Normalizes the outcome of function normal. Uses the Box-Muller method.
 */
double rand_gauss(double mean, double stdev)
{
   double u1=0, u2=0;

   while (u1 * u2 == 0.) {
      u1 = rand() / (double)RAND_MAX;
      u2 = rand() / (double)RAND_MAX;
   }

   double normal = std::sqrt(-2. * std::log(u1)) * std::cos(2 * M_PI * u2);
   return (stdev * normal + mean);
}

/**
 * Randomly perturb a given rigid motion (R,T).
 *
 * @param R       Input rotation matrix to perturb.
 * @param T       Input translation vector to perturb.
 * @param noise_r Perturbation intensity on the rotation component (5-50 is a reasonable range).
 * @param noise_t Variance of the Gaussian used for the translation component.
 * @param res     Mesh resolution.
 * @sa perturb_pairwise()
 */
void perturb_motion(matrix3x3<double>& R, point3d& T, int noise_r, double noise_t, double res)
{
   // Perturb rotation

   math3d::normal3d dir;
   while(true)
   {
      dir.x = rand_gauss(0,1);
      dir.y = rand_gauss(0,1);
      dir.z = rand_gauss(0,1);
      if (math3d::norm2(dir) <= 1.)
         break;
   }
   math3d::normalize(dir);

   const double angle = static_cast<double>(rand() % noise_r) * math3d::rad_on_deg / 10.;
   const double sa = std::sin(0.5*angle);

   math3d::quaternion<double> q(std::cos(0.5*angle), dir.x*sa, dir.y*sa, dir.z*sa);
   math3d::mult_matrix_inplace(math3d::quaternion_to_rot_matrix(q), R, R);

   // Perturb translation

   T += point3d( res * rand_gauss(0,std::sqrt(noise_t)),
                 res * rand_gauss(0,std::sqrt(noise_t)),
                 res * rand_gauss(0,std::sqrt(noise_t)) );
}

/**
 * Given a collection of absolute camera poses, perturb the pairwise rigid motions.
 *
 * @param ring_steps      How many edges in the view graph per-camera pose.
 * @param absolute_ground Starting rigid motions to perturb, wrt world coordinates.
 * @param noise_r         Perturbation intensity on the rotation component (5-50 is a reasonable range).
 * @param noise_t         Variance of the Gaussian used for the translation component.
 * @param resolution      Average mesh resolution.
 * @param pairwise        Output pairwise perturbed rigid motions.
 * @sa perturb_motion()
 */
void perturb_pairwise(
      int ring_steps,
      const vector<rigid_motion_t>& absolute_ground,
      int noise_r,
      double noise_t,
      double resolution,
      vector<rigid_motion_t>& pairwise
){
   pairwise.clear();
   const int nranges = absolute_ground.size();

   for (int i=0; i<nranges; ++i)
   {
      // For each rangemap P we produce ring_steps pairs

      const int P = i;

      int prev = P;
      for (int step=0; step<ring_steps; ++step)
      {
         const int Q = (prev == nranges-1 ? 0 : prev+1);
         prev = Q;

         matrix3x3<double> R_PQ; point3d T_PQ;
         math3d::relative_motion( absolute_ground[P].first, absolute_ground[P].second,
                                 absolute_ground[Q].first, absolute_ground[Q].second,
                                 R_PQ, T_PQ );

         perturb_motion(R_PQ, T_PQ, noise_r, noise_t, resolution);
         pairwise.push_back( std::make_pair(R_PQ, T_PQ) );
      }
   } // next rangemap
}

/**
 * Loads a rigid motion from file.
 */
void load_fine_results(
      const std::string& fname,
      matrix3x3<double>& R,
      point3d& T,
      double* rmse_start=0,
      double* rmse_end=0,
      int* iters=0
){
   std::ifstream fin(fname.c_str(), std::ios::in);
   if (!fin.is_open())
      throw std::runtime_error("Cannot read fine results from " + fname);

   fin >> R.r00 >> R.r01 >> R.r02 >>
         R.r10 >> R.r11 >> R.r12 >>
         R.r20 >> R.r21 >> R.r22 >>
         T.x >> T.y >> T.z;

   double rs, re;
   int i;
   fin >> rs >> re >> i;

   if (rmse_start) *rmse_start = rs;
   if (rmse_end) *rmse_end = re;
   if (iters) *iters = i;

   fin.close();
}

/**
 * Applies dual quaternion error diffusion on the given view graph.
 *
 * Note that for the purpose of this demo, the view graph assumed here has
 * ring topology, where each view is connected to the 'links_per_view' views
 * ahead.
 *
 * In general, one just needs to call transducer::add_transformation() for
 * each pose in the view graph.
 *
 * @param links_per_view How many edges in the view graph per-camera pose.
 * @param pairwise       Input collection of pairwise motions.
 * @param diffused       Output _absolute_ motions after error diffusion.
 */
void run_diffusion(
      int links_per_view,
      const vector<rigid_motion_t>& pairwise,
      vector<rigid_motion_t>& diffused
){
   const int nranges = diffused.size();
   std::cout << std::fixed << std::setprecision(8);

   transducer x(nranges);

   int p=0;
   for (int i=0; i<nranges; ++i)
   {
      const int P = i;

      int prev = P;
      for (int edge=0; edge<links_per_view; ++edge)
      {
         const int Q = (prev == nranges-1 ? 0 : prev+1);
         prev = Q;

         // Add this relative motion to the view graph.
         // Note that we are assigning uniform weights (=1.0) to the different views,
         // but this can be changed if confidence information is available (e.g. coming
         // from the ICP process).

         x.add_transformation(
               P, Q,
               math3d::rot_matrix_to_quaternion( pairwise[p].first ),
               pairwise[p].second,
               1.0);
         ++p;
      }
   }

   x.get_estimate();
   std::cout << "Initial RMSTE " << x.rmste() << std::endl;

   x.linear_transduce();
   std::cout << "Final RMSTE " << x.rmste() << std::endl;

   for (int i=0; i<nranges; ++i)
   {
      dual_quaternion tt;
      x.get_position(i,tt);

      matrix3x3<double> R = math3d::quaternion_to_rot_matrix(tt.R);
      point3d T = tt.get_translation();

      diffused[i].first = R;
      diffused[i].second = T;
      math3d::invert(diffused[i].first, diffused[i].second);
   }
}

/**
 * Given a collection of pairwise rigid transformations, obtain the corresponding
 * absolute motions with respect to the first rangemap; thus, the first transformation
 * in the output vector will be the identity. Note that this code builds a minimum
 * spanning tree according to the adjacency information of the given pairwise motions,
 * thus any cycle will be broken.
 *
 * @param ring_steps How many edges in the view graph per-camera pose.
 * @param pairwise   Input pairwise rigid motions.
 * @param absolute   Output vector containing the absolute rigid motions corresponding
 *                   to the relative transformations given in input as pairwise.
 */
void launch_pairwise(
      int ring_steps,
      const vector<rigid_motion_t>& pairwise,
      vector<rigid_motion_t>& absolute
){
   const int nranges = absolute.size();

   // In the following the transducer is used to obtain the absolute
   // positions by concatenating the perturbed pairwise motions

   transducer x(nranges);

   int p=0;
   for (int i=0; i<nranges; ++i)
   {
      const int P = i;

      int prev = P;
      for (int step=0; step<ring_steps; ++step)
      {
         const int Q = (prev == nranges-1 ? 0 : prev+1);
         prev = Q;

         x.add_transformation(
               P, Q,
               math3d::rot_matrix_to_quaternion( pairwise[p].first ),
               pairwise[p].second);
         ++p;
      }
   }

   x.get_estimate();

   for (int i=0; i<nranges; ++i)
   {
      math3d::quaternion<double> q;
      point3d t;
      x.get_position(i, q, t);
      matrix3x3<double> r = math3d::quaternion_to_rot_matrix(q);

      math3d::invert(r,t);

      absolute[i].first = r;
      absolute[i].second = t;
   }
}

template <typename T>
std::string ntos(T n)
{
   std::stringstream ss;
   ss << n;
   return ss.str();
}

void unhandled_exception()
{
   std::cerr << "Unhandled exception or terminate() caught." << std::endl;
#ifdef WIN32
   system("pause");
#endif
   abort();
}


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
