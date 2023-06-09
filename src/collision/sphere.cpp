#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
	double dist = (pm.position - origin).norm() - radius;
	if (dist <= 0) {
		Vector3D tangent = origin + (pm.position - origin).unit() * radius;
		Vector3D correction = tangent - pm.last_position;
		pm.position = pm.last_position + (1 - friction) * correction;
	}
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
