#include "Physics.h"

#include <algorithm>
#include <cmath>
namespace Physics {
void integrate(Body& b, float dt, float gravity) {
  if (b.fixed) return;
  b.velocity.y += gravity * dt;
  b.box.left += b.velocity.x * dt;
  b.box.top += b.velocity.y * dt;
}
void collide(Body& a, Body& b, float e) {
  if (a.fixed && b.fixed) return;
  float dx = std::min(a.box.left + a.box.width, b.box.left + b.box.width) -
             std::max(a.box.left, b.box.left);
  float dy = std::min(a.box.top + a.box.height, b.box.top + b.box.height) -
             std::max(a.box.top, b.box.top);
  if (dx <= 0 || dy <= 0) return;
  float ia = a.fixed ? 0.f : 1.f / std::max(a.mass, 0.001f);
  float ib = b.fixed ? 0.f : 1.f / std::max(b.mass, 0.001f);
  float sum = ia + ib;
  if (sum == 0) return;
  if (dy < dx) {
    float s = (a.box.top + a.box.height * .5f < b.box.top + b.box.height * .5f)
                  ? -1.f
                  : 1.f;
    if (!a.fixed) a.box.top += s * dy * ia / sum;
    if (!b.fixed) b.box.top -= s * dy * ib / sum;
    float rel = (a.velocity.y - b.velocity.y) * s;
    if (rel < 0) {
      float j = -(1 + e) * rel / sum;
      if (!a.fixed) a.velocity.y += s * j * ia;
      if (!b.fixed) b.velocity.y -= s * j * ib;
    }
  } else {
    float s = (a.box.left + a.box.width * .5f < b.box.left + b.box.width * .5f)
                  ? -1.f
                  : 1.f;
    if (!a.fixed) a.box.left += s * dx * ia / sum;
    if (!b.fixed) b.box.left -= s * dx * ib / sum;
    float rel = (a.velocity.x - b.velocity.x) * s;
    if (rel < 0) {
      float j = -(1 + e) * rel / sum;
      if (!a.fixed) a.velocity.x += s * j * ia;
      if (!b.fixed) b.velocity.x -= s * j * ib;
    }
  }
}
void floor(Body& b, float y) {
  if (b.fixed) return;
  if (b.box.top + b.box.height > y) {
    b.box.top = y - b.box.height;
    if (b.velocity.y > 0) b.velocity.y = 0;
    b.velocity.x *= .985f;
    if (std::abs(b.velocity.x) < 1.f) b.velocity.x = 0;
  }
}
}  // namespace Physics
