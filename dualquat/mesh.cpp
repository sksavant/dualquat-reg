#ifndef MESH_HPP
#define MESH_HPP

#include "mesh.h"
#include <fstream>
#include <iomanip>
#include <queue>
#include <cstdlib>
#include <map>
#include <ctime>
#include <algorithm>
#include <sstream>

using std::vector;
using std::string;
using std::cout;
using std::endl;
using math3d::matrix3x3;
using math3d::normal3d;

void tokenize_str(const string& str, vector<string>& tokens, const string& delimiters)
{
   // Skip delimiters at beginning.
   string::size_type lastPos = str.find_first_not_of(delimiters, 0);
   // Find first "non-delimiter".
   string::size_type pos = str.find_first_of(delimiters, lastPos);

   while (string::npos != pos || string::npos != lastPos)
   {
      // Found a token, add it to the vector.
      tokens.push_back(str.substr(lastPos, pos - lastPos));
      // Skip delimiters.  Note the "not_of"
      lastPos = str.find_first_not_of(delimiters, pos);
      // Find next "non-delimiter"
      pos = str.find_first_of(delimiters, lastPos);
   }
}

template<typename T>
T convert_str(const string& s)
{
   std::istringstream i(s);
   T x;
   if (!(i >> x))
      throw std::invalid_argument("Exception: convert_str()\n");
   return x;
}

/**
 * Apply the inverse of the given motion.
 */
void mesh::apply_inverse_transform(const math3d::matrix3x3<double>& R, const point3d& T)
{
   matrix3x3<double> Ri(R); point3d Ti(T);
   math3d::invert(Ri, Ti);
   apply_rigid_transform(Ri, Ti);
}

/**
 * Apply a rigid transformation to the whole mesh.
 *
 * @param R Rotation matrix.
 * @param T Translation vector.
 */
void mesh::apply_rigid_transform(const math3d::matrix3x3<double>& R, const point3d& T)
{
   const uint32_t nverts = vertices.size();
   const uint32_t ntris = triangles.size();

   // transform vertices and their normals

   for (uint32_t k = 0; k<nverts; ++k) {
      math3d::rotate_translate( vertices[k].p, R, T );
      math3d::rotate( vertices[k].p.n, R );
   }

   // transform triangles normals and centroids

   for (uint32_t k = 0; k<ntris; ++k) {
      math3d::rotate( triangles[k].n, R );
      math3d::rotate_translate( triangles[k].centroid, R, T );
   }
}

void mesh::apply_rigid_transform(const math3d::matrix<double>& R, const point3d& T)
{
   matrix3x3<double> R_;
   R_.r00 = R(0,0); R_.r01 = R(0,1); R_.r02 = R(0,2);
   R_.r10 = R(1,0); R_.r11 = R(1,1); R_.r12 = R(1,2);
   R_.r20 = R(2,0); R_.r21 = R(2,1); R_.r22 = R(2,2);
   apply_rigid_transform(R_, T);
}

/**
 * Get an estimate of the resolution of the current mesh.
 * The estimate is calculated as the median edge length throughout the mesh.
 *
 * @param recompute TRUE to compute resolution estimate from scratch, FALSE to use the cached value from a previous estimate.
 * @return The resolution estimate is returned.
 */
//FIXME
double mesh::get_resolution(bool recompute) const
{
   if (resolution_cache == 0. || recompute)
   {
      const uint32_t ntris = triangles.size();

      if (ntris == 0) {
         resolution_cache = 0.;
         return resolution_cache;
      }

      // this is inefficient and inaccurate...

      vector<double> lengths;

      for (uint32_t k=0; k<ntris; ++k) // consider all the edges of all the triangles
      {
         const point3d& p0 = vertices[ triangles[k].p0 ].p;
         const point3d& p1 = vertices[ triangles[k].p1 ].p;
         const point3d& p2 = vertices[ triangles[k].p2 ].p;

         lengths.push_back( math3d::dist(p0,p1) );
         lengths.push_back( math3d::dist(p2,p1) );
         lengths.push_back( math3d::dist(p0,p2) );

      } // next triangle

      std::sort(lengths.begin(),lengths.end());
      //lengths.resize( std::unique(lengths.begin(),lengths.end()) - lengths.begin() );

      if (lengths.empty())
         resolution_cache = 0.;
      else
         resolution_cache = math3d::median(lengths.begin(), lengths.end());
   }

   return resolution_cache;
}

/**
 * Loads a mesh from an ASCII ply file. The ply file is assumed to contain
 * a tessellation.
 *
 * @param fname Filename of the ply mesh.
 * @throw math3d::file_not_found, math3d::invalid_file_format
 */
void mesh::load_mesh_from_ply(const string& fname, bool verbose)
{
   if (verbose)
      cout << "Loading mesh from " << fname << "..." << endl;

   std::ifstream file_in(fname.c_str(), std::ios::in);

   if (!file_in.is_open())
      throw std::runtime_error("Cannot read "+fname);

   string line;
   unsigned int cur_l=0;
   vector<string> tokens;

   uint32_t nverts = 0, ntris = 0;
   string format;

   // Read the number of points and triangles

   int header_lines = 0;

   do
   {
      tokens.clear();
      std::getline(file_in, line);
      tokenize_str(line, tokens, " ");
      ++cur_l;

      if (tokens.size() == 3)
      {
         if (tokens[0] == "format")
            format = tokens[1];

         else if (tokens[0]=="element" && tokens[1]=="vertex")
            nverts = convert_str<uint32_t>(tokens[2]);

         else if (tokens[0]=="element" && tokens[1]=="face") {
            ntris = convert_str<uint32_t>(tokens[2]);

            do {
               tokens.clear();
               std::getline(file_in,line);
               tokenize_str(line, tokens, " ");
               ++cur_l;
            } while(tokens[0].substr(0,10) != "end_header");

            break;
         }
      }
   } while (!file_in.eof());

   if (format != "ascii")
      throw std::runtime_error("Only ASCII ply can be loaded");

   if (nverts == 0 || ntris == 0)
      throw std::runtime_error("Can't parse mesh in "+fname);

   // We are now at the right position to start reading in vertices

   if (verbose)
   {
      cout << "\tmesh" << endl;
      cout << "\t" << nverts << " points... " << std::flush;
   }

   header_lines = cur_l;
   cur_l = 0;
   vector<point3d> verts(nverts);

   while (cur_l<nverts)
   {
      tokens.clear();
      std::getline(file_in, line);
      tokenize_str(line, tokens, " ");
      verts[cur_l] = point3d(
            convert_str<double>(tokens[0]),
            convert_str<double>(tokens[1]),
            convert_str<double>(tokens[2]));
      ++cur_l;
   }

   put_vertices(verts);
   if (verbose) cout << "loaded." << endl;

   // The mesh faces are now read.

   if (verbose)
      cout << "\t" << ntris << " triangles... " << std::flush;

   for (cur_l=0; cur_l<ntris; ++cur_l)
   {
      tokens.clear();
      std::getline(file_in, line);
      tokenize_str(line, tokens, " ");

      if (tokens.size()!=4 || tokens[0] != "3")
         throw std::runtime_error("The mesh is composed of non-triangular faces.");

      //  p0--,p1
      //   | / |    0,1,3 and 1,2,3
      //  p3'--p2
      add_triangle(
            convert_str<uint32_t>(tokens[1]),
            convert_str<uint32_t>(tokens[2]),
            convert_str<uint32_t>(tokens[3])
            );
   }

   file_in.close();

   // Vertex / triangle normals are finally calculated and assigned to each vertex / triangle

   if (verbose)
      cout << "loaded.\n\tnormals... " << std::flush;

   calc_normals();

   if (verbose)
      cout << "done." << endl;

   if (verbose) cout << "done." << endl;
}

/**
 * The given collection of points is used to populate the mesh vertices.
 * Note that this function does NOT compute vertex normals.
 *
 * @param points
 */
void mesh::put_vertices(const vector<point3d>& points)
{
   const uint32_t n_points = points.size();
   vertices.resize(n_points);

   // Put the points in the appropriate structure for the kd-tree
   // and in the vertices collection for future indexed reference

   for (uint32_t k=0; k<n_points; ++k)
   {
      // indexed vertices
      vertex_data v;
      v.p = oriented_point3d(points[k]);
      v.n_tris = 0;
      vertices[k] = v;
   }
}

/**
 * Adds a triangle to the mesh whose 3 vertices are indicized by the given indices.
 * The added triangle is assigned a centroid and a unit-length normal vector.
 *
 * The triangle is assumed to be given with the vertex order:
 *        p0--,p1
 *         | /
 *        p2'
 *
 * @param p0 Index of the first vertex in the triangle.
 * @param p1 Index of the second vertex in the triangle.
 * @param p2 Index of the third vertex in the triangle.
 * @throw std::out_of_range
 */
void mesh::add_triangle(uint32_t p0, uint32_t p1, uint32_t p2)
{
   triangle_data td;

   td.p0 = p0;
   td.p1 = p1;
   td.p2 = p2;

   const point3d& p3d0 = vertices.at(p0).p;
   const point3d& p3d1 = vertices.at(p1).p;
   const point3d& p3d2 = vertices.at(p2).p;

   td.centroid = point3d( p3d0 + p3d1 + p3d2 );
   td.centroid /= 3;

   // oriented edges: p0->p1 and p0->p2
   // thus n is pointing inside the screen

   td.n = normal3d( cross_product(p3d1-p3d0, p3d2-p3d0) );
   math3d::normalize(td.n);

   triangles.push_back(td);
   const uint32_t tri_idx = triangles.size() - 1;

   vertices.at(p0).tri_indices.push_back(tri_idx);
   vertices.at(p0).n_tris++;
   vertices.at(p1).tri_indices.push_back(tri_idx);
   vertices.at(p1).n_tris++;
   vertices.at(p2).tri_indices.push_back(tri_idx);
   vertices.at(p2).n_tris++;
}

/**
 * Calculates and assigns unit-length normals at each vertex and triangle of the mesh.
 */
void mesh::calc_normals()
{
   const size_t n_points = vertices.size();
   for (size_t k = 0; k < n_points; ++k)
   {
      vertex_data& vert = vertices[k];

      vert.p.n.x = 0.;
      vert.p.n.y = 0.;
      vert.p.n.z = 0.;

      std::list<int>::const_iterator it = vert.tri_indices.begin(), it_end = vert.tri_indices.end();
      for (; it!=it_end; ++it)
      {
         triangle_data& tri = triangles[ *it ];
         const oriented_point3d& p0 = vertices[tri.p0].p;
         const oriented_point3d& p1 = vertices[tri.p1].p;
         const oriented_point3d& p2 = vertices[tri.p2].p;

         // Triangle normal is ccw cross product assuming the vertices are:
         //
         //  p2___p1
         //   |  /
         //   | /
         //   |/
         //   p0
         //

         tri.n = cross_product(p1-p0, p2-p0);
         vert.p.n += tri.n;

         tri.n /= math3d::magnitude(tri.n);

      } // next triangle

      double mag = math3d::magnitude(vert.p.n);
      vert.p.n /= (mag != 0. ? mag : 1.); // some vertices might be outliers

   } // next vertex
}

/**
 * Flip triangle and vertex normals for the current mesh.
 */
void mesh::flip_normals()
{
   uint32_t n_points = vertices.size();
   for (uint32_t k = 0; k < n_points; ++k)
      vertices[k].p.n *= -1.;

   uint32_t n_tris = triangles.size();
   for (uint32_t k = 0; k < n_tris; ++k)
      triangles[k].n *= -1.;
}

/**
 *
 */
void mesh::save_mesh_as_ply(const string& fname) const
{
   cout << "Saving to " << fname << "... " << std::flush;

   const int nv = get_n_vertices();
   const int nt = get_n_tris();

   std::ofstream out(fname.c_str(),std::ios::out);

   out << "ply\n" << "format ascii 1.0\n" << "element vertex " << nv << "\n"<<
         "property float x\nproperty float y\nproperty float z\n"<<
         "element face " << nt <<"\n"<<
         "property list uchar int vertex_indices\nend_header\n";

   for (int k=0; k<nv; ++k)
   {
      const math3d::point3d& pt = get_vertex(k);
      out << pt.x << " " << pt.y << " " << pt.z << endl;
   }

   for (uint32_t k=0; k<get_n_tris(); ++k)
      out << "3 " << triangles[k].p0 << " " << triangles[k].p1 << " " << triangles[k].p2 << "\n";

   out.close();
   cout << "done." << endl;
}

/**
 * Save a color-shaded PLY mesh in ASCII format
 */
void mesh::save_as_ply_colors(const string& fname, const vector<math3d::color_rgb24>& colors) const
{
   cout << "Saving to " << fname << "... " << std::flush;

   const int nv = get_n_vertices();
   const int nt = get_n_tris();

   std::ofstream out(fname.c_str(),std::ios::out);

   out << "ply\n" << "format ascii 1.0\n" << "element vertex " << nv << "\n"<<
         "property float x\nproperty float y\nproperty float z\n"<<
         "property uchar red\nproperty uchar green\nproperty uchar blue\n"<<
         "element face " << nt <<"\n"<<
         "property list uchar int vertex_indices\nend_header\n";

   for (int k=0; k<nv; ++k)
   {
      const math3d::point3d& pt = get_vertex(k);
      out << pt.x << " " << pt.y << " " << pt.z << " ";   
      out << (int)colors[k].r << " " << (int)colors[k].g << " " << (int)colors[k].b << "\n";
   }

   for (uint32_t k=0; k<get_n_tris(); ++k)
      out << "3 " << triangles[k].p0 << " " << triangles[k].p1 << " " << triangles[k].p2 << "\n";

   out.close();
   cout << "done." << endl;
}
#endif
