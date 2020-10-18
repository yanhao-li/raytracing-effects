////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

// Eigen for matrix operations
#include <Eigen/Dense>

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION  // Do not include this line twice in
                                        // your project!
#include "stb_image_write.h"
#include "utils.h"

// JSON parser library (https://github.com/nlohmann/json)
#include "json.hpp"
using json = nlohmann::json;

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Define types & classes
////////////////////////////////////////////////////////////////////////////////

struct Ray {
  Vector3d origin;
  Vector3d direction;
  Ray() {}
  Ray(Vector3d o, Vector3d d) : origin(o), direction(d) {}
};

struct Light {
  Vector3d position;
  Vector3d intensity;
};

struct Intersection {
  Vector3d position;
  Vector3d normal;
  double ray_param;
};

struct Camera {
  bool is_perspective;
  Vector3d position;
  double field_of_view;  // between 0 and PI
  double focal_length;
  double lens_radius;  // for depth of field
};

struct Material {
  Vector3d ambient_color;
  Vector3d diffuse_color;
  Vector3d specular_color;
  double specular_exponent;  // Also called "shininess"

  Vector3d reflection_color;
  Vector3d refraction_color;
  double refraction_index;
};

struct Object {
  Material material;
  virtual ~Object() = default;  // Classes with virtual methods should have a
                                // virtual destructor!
  virtual bool intersect(const Ray& ray, Intersection& hit) = 0;
};

// We use smart pointers to hold objects as this is a virtual class
typedef std::shared_ptr<Object> ObjectPtr;

struct Sphere : public Object {
  Vector3d position;
  double radius;

  virtual ~Sphere() = default;
  virtual bool intersect(const Ray& ray, Intersection& hit) override;
};

struct Parallelogram : public Object {
  Vector3d origin;
  Vector3d u;
  Vector3d v;

  virtual ~Parallelogram() = default;
  virtual bool intersect(const Ray& ray, Intersection& hit) override;
};

struct Scene {
  Vector3d background_color;
  Vector3d ambient_light;

  Camera camera;
  std::vector<Material> materials;
  std::vector<Light> lights;
  std::vector<ObjectPtr> objects;
};

////////////////////////////////////////////////////////////////////////////////

bool Sphere::intersect(const Ray& ray, Intersection& hit) {
  // TODO: - Done
  //
  // Compute the intersection between the ray and the sphere
  // If the ray hits the sphere, set the result of the intersection in the
  // struct 'hit'

	// vector (e - c)
  Vector3d vec = ray.origin - this->position;  
	// ray direction
  Vector3d rd = ray.direction;  

	// d dot d
	double dd = rd.dot(rd);              
	// discriminant, B^2 - 4AC
  double df = pow(2 * rd.dot(vec), 2) -
              4 * dd * ((vec).dot(vec) - pow(this->radius, 2));
  if (df >= 0) {
    double t;
		double t1 = (-2 * rd.dot(vec) + sqrt(df)) / (2 * dd);
		double t2 = (-2 * rd.dot(vec) - sqrt(df)) / (2 * dd);

    if (t1 < t2 && t1 > 0) {
      t = t1;
    } else if (t2 < t1 && t2 > 0) {
      t = t2;
    } else {
      return false;
    }

		hit.position = ray.origin + t * rd;
		hit.normal = (hit.position - this->position).normalized();
		hit.ray_param = t;
    return true;
  }
  return false;
}

bool Parallelogram::intersect(const Ray& ray, Intersection& hit) {
  // TODO

  return false;
}

////////////////////////////////////////////////////////////////////////////////
// Define ray-tracing functions
////////////////////////////////////////////////////////////////////////////////

// Function declaration here (could be put in a header file)
Vector3d ray_color(const Scene& scene, const Ray& ray, const Object& object,
                   const Intersection& hit, int max_bounce);
Object* find_nearest_object(const Scene& scene, const Ray& ray,
                            Intersection& closest_hit);
bool is_light_visible(const Scene& scene, const Ray& ray, const Light& light);
Vector3d shoot_ray(const Scene& scene, const Ray& ray, int max_bounce);

// -----------------------------------------------------------------------------

Vector3d ray_color(const Scene& scene, const Ray& ray, const Object& obj,
                   const Intersection& hit, int max_bounce) {
  // Material for hit object
  const Material& mat = obj.material;

  // Ambient light contribution
  Vector3d ambient_color = mat.ambient_color.array() * scene.ambient_light.array();

  // Punctual lights contribution (direct lighting)
  Vector3d lights_color(0, 0, 0);
  for (const Light& light : scene.lights) {
    Vector3d Li = (light.position - hit.position).normalized();
    Vector3d N = hit.normal;

    // TODO: Shoot a shadow ray to determine if the light should affect the
    // intersection point - Done
    Intersection shadow_ray_hit;
    Ray shadow_ray;
    shadow_ray.direction = (light.position - hit.position).normalized();
    shadow_ray.origin = hit.position + std::numeric_limits<double>::epsilon() * shadow_ray.direction;
    
    if (!is_light_visible(scene, shadow_ray, light)) {
      continue;
    }

    // Diffuse contribution
    Vector3d diffuse = mat.diffuse_color * std::max(Li.dot(N), 0.0);

    // TODO: Specular contribution - Done.
    Vector3d h = (ray.origin + light.position - 2 * hit.position).normalized();
    Vector3d specular = mat.specular_color * pow(std::max(N.dot(h), 0.0), mat.specular_exponent);
		// Vector3d specular(0, 0, 0);

    // Attenuate lights according to the squared distance to the lights
    Vector3d D = light.position - hit.position;
    lights_color +=
        (diffuse + specular).cwiseProduct(light.intensity) / D.squaredNorm();
  }

  // TODO: Compute the color of the reflected ray and add its contribution to
  // the current point color.
  Vector3d reflection_color = Vector3d(0,0,0);
  //prepare reflection ray
  Ray reflection_ray;
  reflection_ray.direction = ray.direction - 2 * (ray.direction.dot(hit.normal)) * hit.normal;
  reflection_ray.origin = hit.position + std::numeric_limits<double>::epsilon() * reflection_ray.direction;
  Intersection reflection_hit;
  Object* reflection_obj = find_nearest_object(scene, reflection_ray, reflection_hit);
  if (reflection_obj && max_bounce > 0) {
    reflection_color = reflection_color + mat.specular_color.cwiseProduct(ray_color(scene, reflection_ray, *reflection_obj, reflection_hit, max_bounce - 1));
  }

  // TODO: Compute the color of the refracted ray and add its contribution to
  // the current point color.
  //       Make sure to check for total internal reflection before shooting a
  //       new ray.
  Vector3d refraction_color = mat.refraction_color;

  // Rendering equation
  Vector3d C =
      ambient_color + lights_color + reflection_color + refraction_color;

  return C;
}

// -----------------------------------------------------------------------------

Object* find_nearest_object(const Scene& scene, const Ray& ray,
                            Intersection& closest_hit) {
  int closest_index = -1;
  // TODO:
  //
  // Find the object in the scene that intersects the ray first
  // The function must return 'nullptr' if no object is hit, otherwise it must
  // return a pointer to the hit object, and set the parameters of the argument
  // 'hit' to their expected values.
	double closest_dist = INT_MAX;

  for (int i = 0; i < scene.objects.size(); i++) {
    Intersection hit;

    if (scene.objects[i]->intersect(ray, hit)) {
   		// calculate distance between hit and ray origin
			double dist = (hit.position - ray.origin).norm();
			if (dist < closest_dist) {
				closest_dist = dist;
				closest_hit = hit;
				closest_index = i;
			}
		};
  }

  if (closest_index < 0) {
    // Return a NULL pointer
    return nullptr;
  } else {
    // Return a pointer to the hit object. Don't forget to set 'closest_hit'
    // accordingly!
    return scene.objects[closest_index].get();
  }
}

bool is_light_visible(const Scene& scene, const Ray& ray, const Light& light) {
  // TODO: Determine if the light is visible here - Done.
  for (const auto object : scene.objects) {
    Intersection hit;
    if (object->intersect(ray, hit)) {
      return false;
    }
  }
  return true;
}

Vector3d shoot_ray(const Scene& scene, const Ray& ray, int max_bounce) {
  Intersection hit;
  if (Object* obj = find_nearest_object(scene, ray, hit)) {
    // 'obj' is not null and points to the object of the scene hit by the ray
    return ray_color(scene, ray, *obj, hit, max_bounce);
  } else {
    // 'obj' is null, we must return the background color
    return scene.background_color;
  }
}

////////////////////////////////////////////////////////////////////////////////

void render_scene(const Scene& scene) {
  std::cout << "Simple ray tracer." << std::endl;

  // the final output image is 640x480 px
  // int w = 640;
  int w = 640;
  // int h = 480;
  int h = 480;
  MatrixXd R = MatrixXd::Zero(w, h);
  MatrixXd G = MatrixXd::Zero(w, h);
  MatrixXd B = MatrixXd::Zero(w, h);
  MatrixXd A = MatrixXd::Zero(w, h);  // Store the alpha mask

  // The camera always points in the direction -z
  // The sensor grid is at a distance 'focal_length' from the camera center,
  // and covers an viewing angle given by 'field_of_view'.
  double aspect_ratio = double(w) / double(h);

	double y_tmp = tan(scene.camera.field_of_view / 2) * scene.camera.focal_length;
  double scale_y = y_tmp;  // TODO: Stretch the pixel grid by the proper amount here - Done
  double scale_x = y_tmp * aspect_ratio;  //

  // The pixel grid through which we shoot rays is at a distance 'focal_length'
  // from the sensor, and is scaled from the canonical [-1,1] in order
  // to produce the target field of view.
  Vector3d grid_origin(-scale_x, scale_y, -scene.camera.focal_length);
  Vector3d x_displacement(2.0 * scale_x / w, 0, 0);
  Vector3d y_displacement(0, -2.0  * scale_y / h, 0);

  for (unsigned i = 0; i < w; ++i) {
    for (unsigned j = 0; j < h; ++j) {
      // TODO: Implement depth of field
      Vector3d shift = grid_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;
      Vector3d noise(scene.camera.lens_radius * (1.0f * rand() / RAND_MAX), scene.camera.lens_radius * (1.0f * rand() / RAND_MAX), scene.camera.position[2]);

      // Prepare the ray
      Ray ray;

      if (scene.camera.is_perspective) {
        // Perspective camera
        ray.origin = noise;
        ray.direction = shift;
      } else {
        // Orthographic camera
        ray.origin = scene.camera.position + Vector3d(shift[0], shift[1], 0);
        ray.direction = Vector3d(0, 0, -1);
      }

      int max_bounce = 5;
      Vector3d C = shoot_ray(scene, ray, max_bounce);
      R(i, j) = C(0);
      G(i, j) = C(1);
      B(i, j) = C(2);
      A(i, j) = 1;
    }
  }

  // Save to png
  const std::string filename("raytrace.png");
  write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

Scene load_scene(const std::string& filename) {
  Scene scene;

  // Load json data from scene file
  json data;
  std::ifstream in(filename);
  in >> data;

  // Helper function to read a Vector3d from a json array
  auto read_vec3 = [](const json& x) { return Vector3d(x[0], x[1], x[2]); };

  // Read scene info
  scene.background_color = read_vec3(data["Scene"]["Background"]);
  scene.ambient_light = read_vec3(data["Scene"]["Ambient"]);

  // Read camera info
  scene.camera.is_perspective = data["Camera"]["IsPerspective"];
  scene.camera.position = read_vec3(data["Camera"]["Position"]);
  scene.camera.field_of_view = data["Camera"]["FieldOfView"];
  scene.camera.focal_length = data["Camera"]["FocalLength"];
  scene.camera.lens_radius = data["Camera"]["LensRadius"];

  // Read materials
  for (const auto& entry : data["Materials"]) {
    Material mat;
    mat.ambient_color = read_vec3(entry["Ambient"]);
    mat.diffuse_color = read_vec3(entry["Diffuse"]);
    mat.specular_color = read_vec3(entry["Specular"]);
    mat.reflection_color = read_vec3(entry["Mirror"]);
    mat.refraction_color = read_vec3(entry["Refraction"]);
    mat.refraction_index = entry["RefractionIndex"];
    mat.specular_exponent = entry["Shininess"];
    scene.materials.push_back(mat);
  }

  // Read lights
  for (const auto& entry : data["Lights"]) {
    Light light;
    light.position = read_vec3(entry["Position"]);
    light.intensity = read_vec3(entry["Color"]);
    scene.lights.push_back(light);
  }

  // Read objects
  for (const auto& entry : data["Objects"]) {
    ObjectPtr object;
    if (entry["Type"] == "Sphere") {
      auto sphere = std::make_shared<Sphere>();
      sphere->position = read_vec3(entry["Position"]);
      sphere->radius = entry["Radius"];
      object = sphere;
    } else if (entry["Type"] == "Parallelogram") {
      // TODO
    }
    object->material = scene.materials[entry["Material"]];
    scene.objects.push_back(object);
  }

  return scene;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " scene.json" << std::endl;
    return 1;
  }
  Scene scene = load_scene(argv[1]);
  render_scene(scene);
  return 0;
}
