#pragma once

#include <SFML/Graphics.hpp>
#include <iostream>
#include "constants.h"
#include "scene.h"

class Canvas {
public:
	sf::RenderWindow window;
	Canvas(int width, int height, const char* name): window(sf::VideoMode(width, height), name) {}
    Canvas() : Canvas(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_NAME) {};
	void animate() {
        Scene scene;
        while (window.isOpen()) {
            sf::Event event;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    window.close();
                }
            }

            scene.update();

            window.clear(sf::Color::White);
            scene.draw(window);
            window.display();
        }
	}
};