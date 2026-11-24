#include <SFML/Graphics.hpp>
#include <iostream>

#include "Simulation.h"

int main() {
  sf::RenderWindow window(sf::VideoMode(1100, 800),
                          "CCD and Multi-rate physics");
  window.setFramerateLimit(144);
  sf::Font font;
  if (!font.loadFromFile("arial.ttf")) {
    std::cerr << "arial.ttf\n";
    return 1;
  }
  Simulation simulation(window.getSize());
  sf::Clock clock;
  while (window.isOpen()) {
    float frameDt = clock.restart().asSeconds();
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
      simulation.event(event, window);
    }
    simulation.update(frameDt);
    window.clear(sf::Color(24, 28, 38));
    simulation.draw(window, font);
    window.display();
  }
  return 0;
}
