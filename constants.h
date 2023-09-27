#pragma once

#include <SFML/Graphics.hpp>
#include <array>

const char* WINDOW_NAME  = "Pinball Game";
const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 1500;

const float BALL_RADIUS = 25.f;
const sf::Color BALL_COLOR = sf::Color::Red;

const float LAUNCH_MAX_SPEED = .1f;

const float FLIPPER_WIDTH = 100.f;
const float FLIPPER_HEIGHT = 10.f;

const sf::Vector2f FLIPPER_SIZE(150.f, 20.f);
const sf::Vector2f LEFT_FLIPPER_POS(250.f, 1200.f);
const sf::Vector2f RIGHT_FLIPPER_POS(650., LEFT_FLIPPER_POS.y);
const sf::Color FLIPPER_COLOR = sf::Color::Blue;
const float FLIPPER_INIT_ANGLE = 25.f;
const float FLIPPER_ANGLE_SPEED = 1.f;

const sf::Vector2f LAUNCH_SPRING_SIZE(BALL_RADIUS * 2.f * 1.75f, 100.);
const sf::Vector2f LAUNCH_SPRING_POS(WINDOW_WIDTH - LAUNCH_SPRING_SIZE.x, WINDOW_HEIGHT - LAUNCH_SPRING_SIZE.y);
const sf::Color LAUNCH_SPRING_COLOR = sf::Color(192, 192, 192);
const float LAUNCH_SPRING_COMPRESS_SPEED = .4f;
const float MIN_SHRINK_PERCENT = .25f;

const sf::Vector2f LAUNCH_WALL_SIZE(20., WINDOW_HEIGHT - BALL_RADIUS * 2.f * 4.f);
const sf::Vector2f LAUNCH_WALL_POS(LAUNCH_SPRING_POS.x - LAUNCH_WALL_SIZE.x, WINDOW_HEIGHT - LAUNCH_WALL_SIZE.y);
const sf::Color LAUNCH_WALL_COLOR = sf::Color::Black;

const float GRAVITY_ACC = .0003f;

const std::array<sf::Vector2f, 6> ROOF_POINTS = {
	sf::Vector2f(0.f, 0.f),
	sf::Vector2f(100.f, 0.f),
	sf::Vector2f(0.f, 100.f),
	sf::Vector2f(WINDOW_WIDTH - 100.f, 0.f),
	sf::Vector2f(WINDOW_WIDTH, 0.f),
	sf::Vector2f(WINDOW_WIDTH, 100.f)
};
const sf::Color ROOF_COLOR = sf::Color::Black;

const float RESTITUTION = .5;