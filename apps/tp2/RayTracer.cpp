//[]---------------------------------------------------------------[]
//|                                                                 |
//| Copyright (C) 2018, 2023 Paulo Pagliosa.                        |
//|                                                                 |
//| This software is provided 'as-is', without any express or       |
//| implied warranty. In no event will the authors be held liable   |
//| for any damages arising from the use of this software.          |
//|                                                                 |
//| Permission is granted to anyone to use this software for any    |
//| purpose, including commercial applications, and to alter it and |
//| redistribute it freely, subject to the following restrictions:  |
//|                                                                 |
//| 1. The origin of this software must not be misrepresented; you  |
//| must not claim that you wrote the original software. If you use |
//| this software in a product, an acknowledgment in the product    |
//| documentation would be appreciated but is not required.         |
//|                                                                 |
//| 2. Altered source versions must be plainly marked as such, and  |
//| must not be misrepresented as being the original software.      |
//|                                                                 |
//| 3. This notice may not be removed or altered from any source    |
//| distribution.                                                   |
//|                                                                 |
//[]---------------------------------------------------------------[]
//
// OVERVIEW: RayTracer.cpp
// ========
// Source file for simple ray tracer.
//
// Author: Paulo Pagliosa
// Last revision: 30/07/2023

#include "graphics/Camera.h"
#include "utils/Stopwatch.h"
#include "RayTracer.h"
#include <iostream>
#include <cmath>

using namespace std;

namespace cg
{ // begin namespace cg

namespace
{ // begin namespace

inline void
printElapsedTime(const char* s, Stopwatch::ms_time time)
{
  printf("%sElapsed time: %g ms\n", s, time);
}

inline auto
arand()
{
  return ((float)std::rand() / RAND_MAX / 4) - 0.125f;
}

} // end namespace

inline auto
maxRGB(const Color& c)
{
  return math::max(math::max(c.r, c.g), c.b);
}

/////////////////////////////////////////////////////////////////////
//
// RayTracer implementation
// =========
RayTracer::RayTracer(SceneBase& scene, Camera& camera):
  Renderer{scene, camera},
  _maxRecursionLevel{6},
  _minWeight{minMinWeight},
  _colorThreshold{0.3f}
{
  // do nothing
}

void
RayTracer::update()
{
  // Delete current BVH before creating a new one
  _bvh = nullptr;

  PrimitiveBVH::PrimitiveArray primitives;
  auto np = uint32_t(0);

  primitives.reserve(_scene->actorCount());
  for (auto actor : _scene->actors())
    if (actor->visible)
    {
      auto p = actor->mapper()->primitive();

      assert(p != nullptr);
      if (p->canIntersect())
      {
        primitives.push_back(p);
        np++;
      }
    }
  _bvh = new PrimitiveBVH{std::move(primitives)};
}

void
RayTracer::render()
{
  throw std::runtime_error("RayTracer::render() invoked");
}

void
RayTracer::renderImage(Image& image)
{
  Stopwatch timer;

  update();
  timer.start();
  {
    const auto& m = _camera->cameraToWorldMatrix();

    // VRC axes
    _vrc.u = m[0];
    _vrc.v = m[1];
    _vrc.n = m[2];
  }

  // init auxiliary mapping variables
  auto w = image.width(), h = image.height();

  setImageSize(w, h);
  _Iw = math::inverse(float(w));
  _Ih = math::inverse(float(h));
  {
    auto wh = _camera->windowHeight();

    if (w >= h)
      _Vw = (_Vh = wh) * w * _Ih;
    else
      _Vh = (_Vw = wh) * h * _Iw;
  }

  // init pixel ray
  float F, B;

  _camera->clippingPlanes(F, B);
  if (_camera->projectionType() == Camera::Perspective)
  {
    // distance from the camera position to a frustum back corner
    auto z = B / F * 0.5f;
    B = vec3f{_Vw * z, _Vh * z, B}.length();
  }
  _pixelRay.tMin = F;
  _pixelRay.tMax = B;
  _pixelRay.set(_camera->position(), -_vrc.n);
  _numberOfRays = _numberOfHits = 0;
  superScan(image);

  auto et = timer.time();

  std::cout << "\nNumber of rays: " << _numberOfRays;
  std::cout << "\nNumber of hits: " << _numberOfHits;
  printElapsedTime("\nDONE! ", et);
}

void
RayTracer::setPixelRay(float x, float y)
//[]---------------------------------------------------[]
//|  Set pixel ray                                      |
//|  @param x coordinate of the pixel                   |
//|  @param y cordinates of the pixel                   |
//[]---------------------------------------------------[]
{
  auto p = imageToWindow(x, y);

  switch (_camera->projectionType())
  {
    case Camera::Perspective:
      _pixelRay.direction = (p - _camera->nearPlane() * _vrc.n).versor();
      break;

    case Camera::Parallel:
      _pixelRay.origin = _camera->position() + p;
      break;
  }
}

void
RayTracer::superScan(Image& image)
{
  const auto length = _maxSteps + 1;
  buffer = new PixelBuffer[image.width() * length];

  ImageBuffer scanLine{ _viewport.w, 1 };

  for (auto j = 0; j < _viewport.h; j++)
  {
    printf("Scanning line %d of %d\r", j + 1, _viewport.h);

    for (auto i = 0; i < length; i++)
      window[i][0].baked = 0;

    for (auto i = 0; i < _viewport.w; i++)
    {
      for (auto k = 0; k < length; k++)
        if (buffer[(i * (length - 1)) + k].baked)
          window[0][k] = buffer[(i * (length - 1)) + k];

      for (auto k = 1; k < length; k++)
        for (auto l = 1; l < length; l++)
          window[k][l].baked = 0;

      Color color = subdivide(0, 0, (float)i, (float)j, _maxSteps);
      scanLine[i] = color;
      
      for (auto k = 0; k < length; k++)
        if (window[length - 1][k].baked)
          buffer[(i * (length - 1)) + k] = window[length - 1][k];

      for (auto k = 0; k < length; k++)
        if (window[k][length - 1].baked)
          window[k][0] = window[k][length - 1];
    }

    image.setData(0, j, scanLine);
  }

  delete[] buffer;
}

void
RayTracer::scan(Image& image)
{
  ImageBuffer scanLine{_viewport.w, 1};

  for (auto j = 0; j < _viewport.h; j++)
  {
    auto y = (float)j + 0.5f;

    printf("Scanning line %d of %d\r", j + 1, _viewport.h);
    for (auto i = 0; i < _viewport.w; i++)
      scanLine[i] = shoot((float)i + 0.5f, y);
    image.setData(0, j, scanLine);
  }
}

Color
RayTracer::subdivide(unsigned i, unsigned j, float x, float y, unsigned steps)
{
  auto offset = steps / (float)_maxSteps;

  Color color[4];
  Color colorMean = Color::black;

  for (int w = 0, c = 0; w < 2; w++)
  {
    for (int z = 0; z < 2; z++)
    {
      auto& buf = window[i + (steps * w)][j + (steps * z)];
      if (buf.baked)
      {
        color[c].setRGB(buf.p.r, buf.p.g, buf.p.b);
      }
      else
      {
        float xr = math::max(math::min((float)x + math::abs(arand()), (float)x + 0.999f), (float)x);
        float yr = math::max(math::min((float)y + math::abs(arand()), (float)y + 0.999f), (float)y);
        color[c] = shoot(xr + (offset * w), yr + (offset * z));
        buf.p.set(color[c]);
        buf.baked = 1;
      }

      c++;
    }
  }

  for (int k = 0; k < 4; k++)
    colorMean += color[k];
  colorMean *= (1.0f / 4);

  for (auto k = 0; k < 4; k++) {
    auto d = math::abs(maxRGB(color[k] - colorMean));
    if (d > _colorThreshold && steps > 1)
    {
      steps >>= 1;

      colorMean = Color::black;
      colorMean += subdivide(i, j, x, y, steps);
      colorMean += subdivide(i + steps, j, x, y, steps);
      colorMean += subdivide(i, j + steps, x, y, steps);
      colorMean += subdivide(i + steps, j + steps, x, y, steps);
      colorMean *= (1.0f / 4);
    }
  }

  return colorMean;
}

Color
RayTracer::shoot(float x, float y)
//[]---------------------------------------------------[]
//|  Shoot a pixel ray                                  |
//|  @param x coordinate of the pixel                   |
//|  @param y cordinates of the pixel                   |
//|  @return RGB color of the pixel                     |
//[]---------------------------------------------------[]
{
  // set pixel ray
  setPixelRay(x, y);

  // trace pixel ray
  Color color = trace(_pixelRay, 0, 1, 1.0f);

  // adjust RGB color
  if (color.r > 1.0f)
    color.r = 1.0f;
  if (color.g > 1.0f)
    color.g = 1.0f;
  if (color.b > 1.0f)
    color.b = 1.0f;
  // return pixel color
  return color;
}

Color
RayTracer::trace(const Ray3f& ray, uint32_t level, float weight, float ior)
//[]---------------------------------------------------[]
//|  Trace a ray                                        |
//|  @param the ray                                     |
//|  @param recursion level                             |
//|  @param ray weight                                  |
//|  @return color of the ray                           |
//[]---------------------------------------------------[]
{
  if (level > _maxRecursionLevel)
    return Color::black;
  ++_numberOfRays;

  Intersection hit;

  return intersect(ray, hit) ? shade(ray, hit, level, weight, ior) : background();
}

inline constexpr auto
rt_eps()
{
  return 1e-4f;
}

bool
RayTracer::intersect(const Ray3f& ray, Intersection& hit)
//[]---------------------------------------------------[]
//|  Ray/object intersection                            |
//|  @param the ray (input)                             |
//|  @param information on intersection (output)        |
//|  @return true if the ray intersects an object       |
//[]---------------------------------------------------[]
{
  hit.object = nullptr;
  hit.distance = ray.tMax;
  return _bvh->intersect(ray, hit) ? ++_numberOfHits : false;
}

Color
RayTracer::shade(const Ray3f& ray,
  Intersection& hit,
  uint32_t level,
  float weight,
  float ior)
//[]---------------------------------------------------[]
//|  Shade a point P                                    |
//|  @param the ray (input)                             |
//|  @param information on intersection (input)         |
//|  @param recursion level                             |
//|  @param ray weight                                  |
//|  @return color at point P                           |
//[]---------------------------------------------------[]
{
  auto primitive = (Primitive*)hit.object;

  assert(nullptr != primitive);

  auto N = primitive->normal(hit);
  const auto& V = ray.direction;
  auto NV = N.dot(V);

  // Make sure "real" normal is on right side
  if (NV > 0)
    N.negate(), NV = -NV;

  // reflection vector
  auto R = V - (2 * NV) * N; 

  auto m = primitive->material();
  // Start with ambient lighting
  auto color = _scene->ambientLight * m->ambient;
  // Intersection point 
  auto P = ray(hit.distance);

  // Compute direct lighting
  for (auto light : _scene->lights())
  {
    // If the light is turned off, then continue
    if (!light->isTurnedOn())
      continue;

    vec3f L;
    float d;

    // If the point P is out of the light range (for finite
    // point light or spotlight), then continue
    if (!light->lightVector(P, L, d))
      continue;

    auto NL = N.dot(L);

    // If light vector is backfaced, then continue
    if (NL <= 0)
      continue;

    auto lightRay = Ray3f{P + L * rt_eps(), L};

    lightRay.tMax = d;
    ++_numberOfRays;

    // If the point P is shadowed, then continue
    if (shadow(lightRay))
      continue;

    auto lc = light->lightColor(d);

    color += lc * m->diffuse * NL;
    if (m->shine <= 0 || (d = R.dot(L)) <= 0)
      continue;
    color += lc * m->spot * pow(d, m->shine);
  }

  // Compute specular reflection
  if (m->specular != Color::black)
  {
    weight *= maxRGB(m->specular);
    if (weight > _minWeight && level < _maxRecursionLevel)
    {
      auto reflectionRay = Ray3f{P + R * rt_eps(), R};
      color += m->specular * trace(reflectionRay, level + 1, weight, ior);
    }
  }

  // Compute refraction
  if (m->transparency != Color::black)
  {
    weight *= maxRGB(m->transparency);
    if (weight > _minWeight && level < _maxRecursionLevel)
    {
      auto n12 = ior / m->ior;
      auto c1 = (-V).dot(N);
      auto c2 = 1 - (math::sqr(n12) * (1 - c1*c1));
      if (!math::isNegative(c2)) {
        auto T = (n12 * V + (n12 * c1 - std::sqrt(c2)) * N).versor();
        auto refractionRay = Ray3f{ P + T * rt_eps(), T};
        color += m->transparency * trace(refractionRay, level + 1, weight, m->ior);
      }
    }
  }

  return color;
}

Color
RayTracer::background() const
//[]---------------------------------------------------[]
//|  Background                                         |
//|  @return background color                           |
//[]---------------------------------------------------[]
{
  return _scene->backgroundColor;
}

bool
RayTracer::shadow(Ray3f ray)
//[]---------------------------------------------------[]
//|  Verifiy if ray is a shadow ray                     |
//|  @param the ray (input)                             |
//|  @return true if the ray intersects an object       |
//[]---------------------------------------------------[]
{
  for(Intersection hit; _bvh->intersect(ray, hit);)
  {
    auto m = ((Primitive*)hit.object)->material();
    if (m->transparency == Color::black) return ++_numberOfHits;

    ray.origin = ray(hit.distance + rt_eps());
    ray.tMax -= hit.distance;
  }

  return false;
}

} // end namespace cg
