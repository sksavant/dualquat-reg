#ifndef MESH_H
#define MESH_H

#include "math3d.h"
#include <list>
#include <limits>
#include <stdexcept>

using math3d::oriented_point3d;
using math3d::point3d;

/**
 * This class describes a mesh as a collection of oriented vertices and triangles.
 */
class mesh
{
   struct vertex_data {
      vertex_data() : n_tris(0) {}
      oriented_point3d p;
      uint32_t n_tris; // # triangles this vertex belongs to
      std::list<int> tri_indices; // triangles this vertex belongs to
   };

   struct triangle_data {
      int p0, p1, p2; // indices of the 3 vertices
      math3d::normal3d n;
      point3d centroid;
   };

   std::vector<vertex_data> vertices;
   std::vector<triangle_data> triangles;

   mutable double resolution_cache;

public:
   explicit mesh() : resolution_cache(0.) {}

   size_t get_n_tris() const { return triangles.size(); }
   size_t get_n_vertices() const { return vertices.size(); }
   double get_resolution(bool recompute=false) const;
   bool empty() const { return (get_n_vertices()==0); }

   // -----------------------------------------------------
   // Mesh transformation
   // -----------------------------------------------------

   void apply_rigid_transform(const math3d::matrix3x3<double>& R, const point3d& T);
   void apply_inverse_transform(const math3d::matrix3x3<double>& R, const point3d& T);

   void apply_rigid_transform(const math3d::matrix<double>& R, const point3d& T);

   void apply_rigid_transform(const math3d::quaternion<double>& Q, const point3d& T) {
      apply_rigid_transform(quaternion_to_rot_matrix(Q) ,T);
   }

   // -----------------------------------------------------
   // Creation and queries
   // -----------------------------------------------------

   const oriented_point3d& get_vertex(uint32_t k) const { return vertices.at(k).p; }

   void put_vertices(const std::vector<point3d>&);
   void add_triangle(uint32_t p1, uint32_t p2, uint32_t p3);

   void calc_normals();
   void flip_normals();

   // -----------------------------------------------------
   // Load & save
   // -----------------------------------------------------

   void load_mesh_from_ply(const std::string& fname, bool verbose=true);
   void save_mesh_as_ply(const std::string& fname) const;
   void save_as_ply_colors(const std::string& fname, const std::vector<math3d::color_rgb24>& colors) const;

}; // mesh

#endif
