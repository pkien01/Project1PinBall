#pragma once
//Kien:
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
            bool enterPressed = false;
            while (window.pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    window.close();
                }
                else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Enter)
                    enterPressed = true;
            }

            scene.update(enterPressed);

            window.clear(sf::Color::White);
            scene.draw(window);
            window.display();
        }
	}
};