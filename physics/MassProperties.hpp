#pragma once

#include "Config.hpp"
#include "ConvexMesh.hpp"
#include <stdio.h>

struct MassProperties {

  float mass;
  float inv_mass;
  glm::vec3 centre_of_mass; // Should be (0, 0, 0 ) in local space?
  glm::mat3 inertia_tensor; // At COM
  glm::mat3 inv_inertia_tensor;

  static MassProperties compute(const ConvexMesh &mesh, float density) {

    float mass;
    float inv_mass;
    glm::vec3 com;
    glm::mat3 inertia;

    // constants
    const float mult[10] = {
        1.0f / 6, 1.0f / 24, 1.0f / 24, 1.0f / 24, 1.0f / 60, 1.0f / 60, 1.0f / 60, 1.0f / 120, 1.0f / 120, 1.0f / 120};

    // volume integrals order: 1, x, y, z, x^2, y^2, z^2, xy, yz, zx
    float intg[10] = {0.0f};

    glm::vec3 mesh_centroid = mesh.compute_geometric_centroid();

    // debug
    int flipped_faces = 0;
    int negative_contrib = 0;
    float total_volume_contrib = 0.0f;

    for (size_t i = 0; i < mesh.faces.size(); i++) {

      auto &face = mesh.faces[i];

      // get face vertices
      auto v = mesh.get_face_vertices(face);
      glm::vec3 v0 = mesh.vertices[v[0]];
      glm::vec3 v1 = mesh.vertices[v[1]];
      glm::vec3 v2 = mesh.vertices[v[2]];

      // get edges and cross products
      // glm::vec3 n = glm::normalize(getTriangleNormal(v1, v2, v0));
      glm::vec3 e1 = v1 - v0;
      glm::vec3 e2 = v2 - v0;
      glm::vec3 n = glm::cross(e1, e2);

      glm::vec3 face_center = (v0 + v1 + v2) / 3.0f;
      glm::vec3 outward_direction = face_center - mesh_centroid;

      // flip normal
      if (glm::dot(n, outward_direction) < 0.0f) {
        n = -n;
        flipped_faces++;
      }

      // surface integral shortcuts

      // x-components
      float fX, fSqX, fCuX, gX0, gX1, gX2;
      compute_subexpressions(v0.x, v1.x, v2.x, fX, fSqX, fCuX, gX0, gX1, gX2);

      // y-components
      float fY, fSqY, fCuY, gY0, gY1, gY2;
      compute_subexpressions(v0.y, v1.y, v2.y, fY, fSqY, fCuY, gY0, gY1, gY2);

      // z-components
      float fZ, fSqZ, fCuZ, gZ0, gZ1, gZ2;
      compute_subexpressions(v0.z, v1.z, v2.z, fZ, fSqZ, fCuZ, gZ0, gZ1, gZ2);

      // DEBUG: Check contribution
      float face_volume_contrib = n.x * fX;
      total_volume_contrib += face_volume_contrib;
      if (face_volume_contrib < 0) {
        negative_contrib++;
      }

      // update integrals
      // 1
      intg[0] += n.x * fX;

      // x, y, z
      intg[1] += n.x * fSqX;
      intg[2] += n.y * fSqY;
      intg[3] += n.z * fSqZ;

      // x^2. y^2, z^2
      intg[4] += n.x * fCuX;
      intg[5] += n.y * fCuY;
      intg[6] += n.z * fCuZ;

      // xy, yz, zx
      intg[7] += n.x * (v0.y * gX0 + v1.y * gX1 + v2.y * gX2);
      intg[8] += n.y * (v0.z * gY0 + v1.z * gY1 + v2.z * gY2);
      intg[9] += n.z * (v0.x * gZ0 + v1.x * gZ1 + v2.x * gZ2);
    }

    // DEBUG OUTPUT

    /*
    printf("=== Mass Properties Calculation ===\n");
    printf("Total faces: %zu\n", mesh.faces.size());
    printf("Faces with negative contribution: %d\n", negative_contrib);
    printf("Faces that needed flipping: %d\n", flipped_faces);
    printf("Raw volume integral: %f\n", total_volume_contrib);
    */

    // multiply constants
    for (size_t i = 0; i < 10; i++) {
      intg[i] *= mult[i];
    }
    float vol = intg[0];
    CLOGI("Final volume: %f\n", vol);

    // Volume should be positive!
    if (vol <= 0) {
      CLOGI("ERROR: Non-positive volume! Mesh likely has wrong winding order.\n");
      // You might want to flip all normals and try again
    }

    // float vol = intg[0];
    mass = density * vol;
    CLOGI("Final mass: %f\n", mass);

    inv_mass = 1.0f / mass;
    CLOGI("Final inverse mass: %f\n", inv_mass);

    com.x = intg[1] / vol;
    com.y = intg[2] / vol;
    com.z = intg[3] / vol;

    float Ixx_origin = density * (intg[5] + intg[6]);
    float Iyy_origin = density * (intg[4] + intg[6]);
    float Izz_origin = density * (intg[4] + intg[5]);
    float Ixy_origin = -density * intg[7];
    float Iyz_origin = -density * intg[8];
    float Izx_origin = -density * intg[9];

    // Translate to center of mass using parallel axis theorem
    inertia[0][0] = Ixx_origin - mass * (com.y * com.y + com.z * com.z);
    inertia[1][1] = Iyy_origin - mass * (com.z * com.z + com.x * com.x);
    inertia[2][2] = Izz_origin - mass * (com.x * com.x + com.y * com.y);
    inertia[0][1] = inertia[1][0] = Ixy_origin + mass * com.x * com.y;
    inertia[1][2] = inertia[2][1] = Iyz_origin + mass * com.y * com.z;
    inertia[2][0] = inertia[0][2] = Izx_origin + mass * com.z * com.x;

    return MassProperties{.mass = mass,
        .inv_mass = inv_mass,
        .centre_of_mass = com,
        .inertia_tensor = inertia,
        .inv_inertia_tensor = glm::inverse(inertia)};

    /*
    // xx, yx, zx
    glm::vec3 col1(
            density * (intg[5] + intg[6] - vol * (com.y * com.y + com.z *
    com.z)), density * -(intg[7] - vol * (com.y * com.x)), density * -(intg[9] -
    vol * (com.z * com.x))
    );

    // xy, yy, zy
    glm::vec3 col2(
            -(intg[7] - vol * (com.x * com.y)),
            intg[4] + intg[6] - vol * (com.z * com.z + com.x * com.x),
            -(intg[8] - vol * (com.z * com.y))
    );

    // xz, yz, zz
    glm::vec3 col3(
            -(intg[9] - vol * (com.x * com.z)),
            -(intg[8] - vol * (com.y * com.z)),
            intg[4] + intg[5] - vol * (com.x * com.x + com.y * com.y)
    );
    */

    /*
                // xx, yx, zx
                glm::vec3 col1(
                        density * (intg[5] + intg[6] - vol * (com.y * com.y +
       com.z * com.z)),  // Ixx density * (-(intg[7] - vol * com.x * com.y)), //
       Iyx density * (-(intg[9] - vol * com.x * com.z)) // Izx
                );

                // xy, yy, zy
                glm::vec3 col2(
                        density * (-(intg[7] - vol * com.x * com.y)), // Ixy
                        density * (intg[4] + intg[6] - vol * (com.x * com.x +
       com.z * com.z)),  // Iyy density * (-(intg[8] - vol * com.y * com.z)) //
       Izy
                );

                // xz, yz, zz
                glm::vec3 col3(
                        density * (-(intg[9] - vol * com.x * com.z)), // Ixz
                        density * (-(intg[8] - vol * com.y * com.z)), // Iyz
                        density * (intg[4] + intg[5] - vol * (com.x * com.x +
       com.y * com.y))   // Izz
                );

                inertia = glm::mat3(col1, col2, col3);
    */
  }

private:
  // helper function for computing surface integrals
  static void compute_subexpressions(
      float w0, float w1, float w2, float &f1, float &f2, float &f3, float &g0, float &g1, float &g2) {

    float temp0 = w0 + w1;
    f1 = temp0 + w2;

    float temp1 = w0 * w0;
    float temp2 = temp1 + w1 * temp0;

    f2 = temp2 + w2 * f1;
    f3 = w0 * temp1 + w1 * temp2 + w2 * f2;

    g0 = f2 + w0 * (f1 + w0);
    g1 = f2 + w1 * (f1 + w1);
    g2 = f2 + w2 * (f1 + w2);
  }
};
