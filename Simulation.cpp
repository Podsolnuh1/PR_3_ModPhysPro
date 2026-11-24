#include "Simulation.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>

Simulation::Simulation(sf::Vector2u s) : size(s) {
  wall = sf::FloatRect(s.x * .62f, 100.f, 10.f, s.y - 220.f);
  float y = 155.f, x = 55.f, width = float(s.x) - 110.f;
  buttons = {{{x, y, width, 58.f},
              "1. DISCRETE CHECK - projectile tunnels through wall",
              Discrete},
             {{x, y + 72, width, 58.f},
              "2. CCD - projectile detects impact and bounces",
              CCD},
             {{x, y + 195, width, 58.f},
              "3. SAME RATE - all dynamic bodies at 60 Hz",
              Same60},
             {{x, y + 267, width, 58.f},
              "4. MIXED RATE - normal 60 Hz + fast 240 Hz",
              Mixed60_240},
             {{x, y + 339, width, 58.f},
              "5. STATIC + DYNAMIC - fixed obstacles and falling bodies",
              StaticMixed}};
}
void Simulation::resetStats() {
  accumulator = lastMs = avgMs = 0;
  frames = steps60 = steps240 = 0;
  hit = tunnel = false;
}
void Simulation::setupBullet() {
  bullet = {100.f, size.y * .5f};
  bulletV = {2000.f, 0.f};
  bulletR = 6.f;
}
void Simulation::setupNormal() {
  normalBodies.clear();
  fastBodies.clear();
  fixedBodies.clear();

  constexpr int stackCount = 5;
  constexpr int blocksPerStack = 5;
  constexpr float blockWidth = 82.f;
  constexpr float blockHeight = 28.f;
  constexpr float stackSpacing = 195.f;
  const float floorY = float(size.y) - 90.f;
  const float bottomY = floorY - 5.f * (blockHeight + 3.f) - 200.f;

  for (int stack = 0; stack < stackCount; ++stack) {
    for (int level = 0; level < blocksPerStack; ++level) {
      Body b;
      b.box = {75.f + stack * stackSpacing,
               bottomY - level * (blockHeight + 3.f), blockWidth, blockHeight};
      b.mass = blockWidth * blockHeight / 900.f;
      b.color = sf::Color(75 + (stack * 23 + level * 11) % 110, 155, 225);
      normalBodies.push_back(b);
    }
  }
}
void Simulation::setupMixed() {
  normalBodies.clear();
  fastBodies.clear();
  fixedBodies.clear();

  constexpr int pairCount = 10;
  constexpr float bodyWidth = 58.f;
  constexpr float bodyHeight = 26.f;
  constexpr float pairSpacing = 105.f;
  const float normalY = 285.f;

  for (int i = 0; i < pairCount; ++i) {
    const float x = 32.f + i * pairSpacing;

    Body normal;
    normal.box = {x, normalY, bodyWidth, bodyHeight};
    normal.mass = 3.f * bodyWidth * bodyHeight / 900.f;
    normal.color = sf::Color(75, 155, 225);
    normalBodies.push_back(normal);

    Body fast;
    fast.box = {x, normalY - bodyHeight - 12.f, bodyWidth, bodyHeight};
    fast.velocity = {0.f, 240.f};
    fast.mass = bodyWidth * bodyHeight / 900.f;
    fast.color = sf::Color(255, 115, 70);
    fastBodies.push_back(fast);
  }
}
void Simulation::setupStatic() {
  setupMixed();
  float fy = size.y - 90.f;
  for (int i = 0; i < 5; ++i) {
    Body b;
    b.fixed = true;
    b.mass = 0;
    b.color = sf::Color(160, 165, 180);
    if (i < 4)
      b.box = {90.f + i * 210.f, fy - 60.f, 60.f, 60.f};
    else
      b.box = {850.f, fy - 170.f, 60.f, 60.f};
    fixedBodies.push_back(b);
  }
}
void Simulation::start(Mode m) {
  mode = m;
  resetStats();
  if (m == Discrete || m == CCD) setupBullet();
  if (m == Same60) setupNormal();
  if (m == Mixed60_240) setupMixed();
  if (m == StaticMixed) setupStatic();
}
void Simulation::event(const sf::Event& e, sf::RenderWindow& w) {
  if (e.type == sf::Event::KeyPressed) {
    if (e.key.code == sf::Keyboard::Escape) {
      if (mode == Menu)
        w.close();
      else
        mode = Menu;
    }
    if (e.key.code == sf::Keyboard::R && mode != Menu) start(mode);
  }
  if (mode == Menu && e.type == sf::Event::MouseButtonPressed &&
      e.mouseButton.button == sf::Mouse::Left) {
    sf::Vector2f p(float(e.mouseButton.x), float(e.mouseButton.y));
    for (const auto& b : buttons)
      if (b.rect.contains(p)) {
        start(b.mode);
        break;
      }
  }
}
void Simulation::updateBullet(bool ccd) {
  if (ccd) {
    float xContact = wall.left - bulletR;
    float t = (xContact - bullet.x) / bulletV.x;
    if (bulletV.x > 0 && t >= 0 && t <= dt60) {
      bullet.x = xContact;
      bulletV.x = -bulletV.x * .85f;
      bullet.x += bulletV.x * (dt60 - t);
      hit = true;
    } else
      bullet += bulletV * dt60;
  } else {
    bullet += bulletV * dt60;
    sf::FloatRect r(bullet.x - bulletR, bullet.y - bulletR, bulletR * 2,
                    bulletR * 2);
    if (r.intersects(wall)) {
      hit = true;
      bullet.x = wall.left - bulletR;
      bulletV.x = -bulletV.x * .85f;
    }
  }
  if (!hit && bullet.x - bulletR > wall.left + wall.width) tunnel = true;
  if (bullet.x < 0 || bullet.x > size.x) setupBullet();
}
void Simulation::updateGroup(std::vector<Body>& g, float dt) {
  for (auto& b : g) Physics::integrate(b, dt, 900.f);
  for (int pass = 0; pass < 3; ++pass) {
    for (size_t i = 0; i < g.size(); ++i)
      for (size_t j = i + 1; j < g.size(); ++j) Physics::collide(g[i], g[j]);
    for (auto& b : g) Physics::floor(b, float(size.y) - 90.f);
  }
}
void Simulation::resolveFixed(std::vector<Body>& g) {
  for (auto& b : g)
    for (auto& s : fixedBodies) Physics::collide(b, s, .02f);
}
void Simulation::update(float frameDt) {
  if (mode == Menu) return;
  frameDt = std::min(frameDt, .05f);
  auto begin = std::chrono::steady_clock::now();
  accumulator += frameDt;
  if (mode == Discrete || mode == CCD) {
    while (accumulator >= dt60) {
      updateBullet(mode == CCD);
      accumulator -= dt60;
      ++steps60;
    }
  } else if (mode == Same60) {
    while (accumulator >= dt60) {
      updateGroup(normalBodies, dt60);
      accumulator -= dt60;
      ++steps60;
    }
  } else {
    while (accumulator >= dt240) {
      updateGroup(fastBodies, dt240);
      ++steps240;
      for (auto& f : fastBodies)
        for (auto& n : normalBodies) Physics::collide(f, n, .7f);
      if (mode == StaticMixed) resolveFixed(fastBodies);
      accumulator -= dt240;
      if (steps240 % 4 == 0) {
        updateGroup(normalBodies, dt60);
        ++steps60;
        if (mode == StaticMixed) resolveFixed(normalBodies);
      }
    }
  }
  auto end = std::chrono::steady_clock::now();
  lastMs = std::chrono::duration<float, std::milli>(end - begin).count();
  ++frames;
  avgMs += (lastMs - avgMs) / float(frames);
}
void Simulation::label(sf::RenderWindow& w, const sf::Font& f,
                       const std::string& s, float x, float y, unsigned int n,
                       sf::Color c) {
  sf::Text t(s, f, n);
  t.setPosition(x, y);
  t.setFillColor(c);
  w.draw(t);
}
void Simulation::draw(sf::RenderWindow& w, const sf::Font& f) {
  if (mode == Menu) {
    label(w, f, "PHYSICS SIMULATION", 55, 35, 30);
    label(w, f, "PART 1. CONTINUOUS COLLISION DETECTION", 55, 105, 19,
          sf::Color(120, 190, 255));
    label(w, f, "PART 2. MULTI-RATE SIMULATION", 55, 315, 19,
          sf::Color(120, 225, 170));
    for (auto& b : buttons) {
      sf::RectangleShape r({b.rect.width, b.rect.height});
      r.setPosition(b.rect.left, b.rect.top);
      r.setFillColor(sf::Color(43, 55, 75));
      r.setOutlineThickness(1);
      r.setOutlineColor(sf::Color(115, 140, 175));
      w.draw(r);
      label(w, f, b.label, b.rect.left + 14, b.rect.top + 17, 17);
    }
    label(w, f, "Click an experiment | ESC: exit", 55, float(size.y) - 35, 15,
          sf::Color(190, 195, 205));
    return;
  }
  std::string title;
  if (mode == Discrete) title = "PART 1 - DISCRETE COLLISION";
  if (mode == CCD) title = "PART 1 - CONTINUOUS COLLISION DETECTION";
  if (mode == Same60) title = "PART 2 - ALL OBJECTS AT 60 Hz";
  if (mode == Mixed60_240) title = "PART 2 - NORMAL 60 Hz + FAST 240 Hz";
  if (mode == StaticMixed) title = "PART 2 - STATIC AND DYNAMIC OBJECTS";
  label(w, f, title, 20, 18, 22);
  label(w, f, "ESC: menu   R: restart", size.x - 250.f, 22, 15,
        sf::Color(190, 195, 205));
  if (mode == Discrete || mode == CCD) {
    sf::RectangleShape wallShape({wall.width, wall.height});
    wallShape.setPosition(wall.left, wall.top);
    wallShape.setFillColor(sf::Color(180, 185, 200));
    w.draw(wallShape);
    sf::CircleShape p(bulletR);
    p.setOrigin(bulletR, bulletR);
    p.setPosition(bullet);
    p.setFillColor(sf::Color(255, 95, 70));
    w.draw(p);
    label(w, f, "Speed: 2000 px/s | Wall: 10 px | dt = 1/60 s", 20, 65, 16);
    label(w, f,
          hit      ? "Impact detected - bounce"
          : tunnel ? "TUNNELING - bullet passed through wall"
                   : "Projectile in flight",
          20, 95, 18, tunnel ? sf::Color::Red : sf::Color(140, 235, 165));
  } else {
    float fy = size.y - 90.f;
    sf::RectangleShape floorShape({float(size.x) - 80, 12});
    floorShape.setPosition(40, fy);
    floorShape.setFillColor(sf::Color(175, 180, 195));
    w.draw(floorShape);
    auto drawGroup = [&](const std::vector<Body>& group) {
      for (const auto& b : group) {
        sf::RectangleShape r({b.box.width, b.box.height});
        r.setPosition(b.box.left, b.box.top);
        r.setFillColor(b.color);
        r.setOutlineThickness(1);
        r.setOutlineColor(sf::Color::White);
        w.draw(r);
      }
    };
    drawGroup(fixedBodies);
    drawGroup(normalBodies);
    drawGroup(fastBodies);
    label(w, f,
          "Normal: " + std::to_string(normalBodies.size()) +
              " bodies / 60 Hz (blue)",
          20, 60, 16, sf::Color(130, 185, 245));
    if (!fastBodies.empty())
      label(w, f,
            "Fast: " + std::to_string(fastBodies.size()) +
                " bodies / 240 Hz (orange)",
            20, 84, 16, sf::Color(255, 165, 115));
    if (!fixedBodies.empty())
      label(w, f, "Static: 5 fixed bodies (grey)", 20, 108, 16,
            sf::Color(205, 210, 220));
  }
  std::ostringstream s;
  s << std::fixed << std::setprecision(4) << "Physics update: " << lastMs
    << " ms   Average: " << avgMs << " ms";
  label(w, f, s.str(), 20, float(size.y) - 70, 16, sf::Color::Yellow);
  label(w, f,
        "60 Hz steps: " + std::to_string(steps60) +
            "   240 Hz steps: " + std::to_string(steps240),
        20, float(size.y) - 43, 15);
}
