#pragma once

#include "linalg.h"
#include "ColorMap.h"
#include "enums.h"
#include "PolySet.h"
#include "Renderer.h"

void render_surface(const PolySet& geom, Renderer::csgmode_e csgmode, const Transform3d& m, int textureind, const Renderer::shaderinfo_t *shaderinfo = nullptr);
void render_edges(const PolySet& geom, Renderer::csgmode_e csgmode);
