#pragma once

#include "scene/camera.h"
#include "scene/geometry.h"
#include "scene/light.h"
#include "scene/material.h"

#include "util/matrix.h"
#include "util/transforms.h"

#include <vector>

using namespace std;

class Parser {
  public:

  Parser(int screenW = 800, int screenH = 600) :
    screenW(screenW), screenH(screenH) {}

  vector<CudaPrimitive> primitives; // triangles
  vector<Vector3D> vertices;
  vector<Vector3D> normals;
  vector<Vector2D> texcoords; // texture coordinates
  vector<Vector4D> tangents; // tangents
  vector<CudaBSDF> bsdfs; // materials
  vector<CudaTexture> textures; // textures
  vector<CudaLight> lights; // lights
  Camera camera; // camera
  int screenW, screenH; // screen width and height 

};