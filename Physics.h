#pragma once
#include <SFML/Graphics.hpp>
struct Body {
  sf::FloatRect box;
  sf::Vector2f velocity{0.f, 0.f};
  sf::Color color = sf::Color::White;
  float mass = 1.f;
  bool fixed = false;
};
namespace Physics {
void integrate(Body& b, float dt, float gravity);
void collide(Body& a, Body& b, float restitution = 0.7f);
void floor(Body& b, float y);
}  // namespace Physics
