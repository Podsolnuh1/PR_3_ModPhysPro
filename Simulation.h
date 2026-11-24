#pragma once
#include <SFML/Graphics.hpp>
#include <string>
#include <vector>

#include "Physics.h"

class Simulation {
 public:
  explicit Simulation(sf::Vector2u size);
  void event(const sf::Event& e, sf::RenderWindow& w);
  void update(float frameDt);
  void draw(sf::RenderWindow& w, const sf::Font& font);

 private:
  enum Mode { Menu, Discrete, CCD, Same60, Mixed60_240, StaticMixed };
  struct Button {
    sf::FloatRect rect;
    std::string label;
    Mode mode;
  };
  sf::Vector2u size;
  Mode mode = Menu;
  std::vector<Button> buttons;
  sf::FloatRect wall;
  sf::Vector2f bullet, bulletV;
  float bulletR = 6.f, accumulator = 0.f;
  bool hit = false, tunnel = false;
  std::vector<Body> fixedBodies, normalBodies, fastBodies;
  float lastMs = 0.f, avgMs = 0.f;
  unsigned long long frames = 0, steps60 = 0, steps240 = 0;
  static constexpr float dt60 = 1.f / 60.f, dt240 = 1.f / 240.f;
  void start(Mode m);
  void resetStats();
  void setupBullet();
  void setupNormal();
  void setupMixed();
  void setupStatic();
  void updateBullet(bool ccd);
  void updateGroup(std::vector<Body>& group, float dt);
  void resolveFixed(std::vector<Body>& group);
  void label(sf::RenderWindow&, const sf::Font&, const std::string&, float,
             float, unsigned int = 18, sf::Color = sf::Color::White);
};
