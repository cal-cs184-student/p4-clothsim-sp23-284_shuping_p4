#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.

    // initialize
    double dx = width / (num_width_points - 1.0);
    double dy = height / (num_height_points - 1.0);
    for (int i = 0; i < num_height_points; i++) {
        for (int j = 0; j < num_width_points; j++) {
            double x = j * dx;
            double y, z;
            // check orientation to determine z
            if (orientation == HORIZONTAL) {
                y = 1.0;
                z = i * dy;
            }
            else {
                y = i * dy;
                z = (rand() - 0.5) / 500.0;
            }
            // store in row major
            Vector3D pos(x, y, z);
            //PointMass point(pos, false);
            point_masses.emplace_back(pos, false);
        }
    }

    // set mass pinned
    for (int i = 0; i < pinned.size(); i++) {
        int y = pinned[i][0];
        int x = pinned[i][1];
        PointMass* pos = &point_masses[y * num_width_points + x];
        pos->pinned = true;
    }

    // apply structual, shear, bending constraints
    // structual
    for (int i = 0; i < num_height_points; i++) {
        for (int j = 0; j < num_width_points; j++) {
            if (i == 0 && j == 0) {
                continue;
            }

            PointMass* pos = &point_masses[i * num_width_points + j];
            // top
            if (i != 0) {
                PointMass *top = &point_masses[(i - 1) * num_width_points + j];
                springs.emplace_back(pos, top, STRUCTURAL);
            }
            // left
            if (j != 0) {
                PointMass* left = &point_masses[i * num_width_points + (j - 1)];
                springs.emplace_back(pos, left, STRUCTURAL);
            }
        }
    }
    // shear
    for (int i = 1; i < num_height_points; i++) {
        for (int j = 1; j < num_width_points; j++) {
            PointMass* ul = &point_masses[(i - 1) * num_width_points + (j - 1)];
            PointMass* ur = &point_masses[(i - 1) * num_width_points + j];
            PointMass* bl = &point_masses[i * num_width_points + (j - 1)];
            PointMass* br = &point_masses[i * num_width_points + j];

            springs.emplace_back(br, ul, SHEARING);
            springs.emplace_back(bl, ur, SHEARING);
        }
    }
    // bending
    for (int i = 0; i < num_height_points; i++) {
        for (int j = 0; j < num_width_points; j++) {
            if ((i - 2) < 0 && (j - 2) < 0) {
                continue;
            }

            PointMass* pos = &point_masses[i * num_width_points + j];
            // top
            if ((i - 2) >= 0) {
                PointMass* top = &point_masses[(i - 2) * num_width_points + j];
                springs.emplace_back(pos, top, BENDING);
            }
            // left
            if ((j - 2) >= 0) {
                PointMass* left = &point_masses[i * num_width_points + (j - 2)];
                springs.emplace_back(pos, left, BENDING);
            }
        }
    }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  // total external force
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* pm = &point_masses[i];
      pm->forces = Vector3D(0);
      for (Vector3D acc : external_accelerations) {
          pm->forces += mass * acc;
      }
  }
  // spring correction forces
  for (int i = 0; i < springs.size(); i++) {
      Spring* sp = &springs[i];
      Vector3D pointA = sp->pm_a->position;
      Vector3D pointB = sp->pm_b->position;
      if (cp->enable_structural_constraints) {
          sp->pm_a->forces += cp->ks * ((pointA - pointB).norm() - sp->rest_length) * (pointB - pointA).unit();
          sp->pm_b->forces += cp->ks * ((pointA - pointB).norm() - sp->rest_length) * (pointA - pointB).unit();
      }
      if (cp->enable_shearing_constraints) {
          sp->pm_a->forces += cp->ks * ((pointA - pointB).norm() - sp->rest_length) * (pointB - pointA).unit();
          sp->pm_b->forces += cp->ks * ((pointA - pointB).norm() - sp->rest_length) * (pointA - pointB).unit();
      }
      if (cp->enable_bending_constraints) {
          sp->pm_a->forces += 0.2 * cp->ks * ((pointA - pointB).norm() - sp->rest_length) * (pointB - pointA).unit();
          sp->pm_b->forces += 0.2 * cp->ks * ((pointA - pointB).norm() - sp->rest_length) * (pointA - pointB).unit();
      }
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* pm = &point_masses[i];
      if (pm->pinned) {
          continue;
      }
      Vector3D curPos = pm->position;
      pm->position = pm->position + (1.0 - cp->damping / 100.0) * (pm->position - pm->last_position) + (pm->forces / mass) * (delta_t * delta_t);
      pm->last_position = curPos;
  }

  // TODO (Part 4): Handle self-collisions.


  // TODO (Part 3): Handle collisions with other primitives.


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (int i = 0; i < springs.size(); i++) {
      Spring* s = &springs[i];
      PointMass* pointA = s->pm_a;
      PointMass* pointB = s->pm_b;

      double spring_length = (pointA->position - pointB->position).norm();
      double max_length = 1.1 * s->rest_length;

      if (spring_length > max_length) {
          if (pointA->pinned && pointB->pinned) {
              continue;
          }
          else if (pointA->pinned) {
              pointB->position = pointA->position + (pointB->position - pointA->position).unit() * max_length;
          }
          else if (pointB->pinned) {
              pointA->position = pointB->position + (pointA->position - pointB->position).unit() * max_length;
          }
          else {
              pointA->position += (spring_length - max_length) / 2.0 * (pointB->position - pointA->position).unit();
              pointB->position += (spring_length - max_length) / 2.0 * (pointA->position - pointB->position).unit();
          }
      }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

  return 0.f; 
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
