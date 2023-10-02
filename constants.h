#pragma once

#include <SFML/Graphics.hpp>
#include <array>

//Had to adjust all the constants for rendering by /2 or show on my mac - RUTH

//Kien:
const char* WINDOW_NAME  = "Pinball Game";
const int WINDOW_WIDTH = 1000/2;
const int WINDOW_HEIGHT = 1500/2;

const float BALL_RADIUS = 25.f/2;
const sf::Color BALL_COLOR = sf::Color::Red;
//Ruth:
const float OB_BALL_RADIUS = 35.f;
const sf::Color OB_BALL_COLOR = sf::Color::Yellow;
const float OB_MASS = 10.f;

//Kien:
const float LAUNCH_MAX_SPEED = .1f/10;

const float FLIPPER_WIDTH = 100.f/2;
const float FLIPPER_HEIGHT = 10.f/2;

const sf::Vector2f FLIPPER_SIZE(150.f/2, 20.f/2);
const sf::Vector2f LEFT_FLIPPER_POS(250.f/2, 1200.f/2);
const sf::Vector2f RIGHT_FLIPPER_POS(650./2, LEFT_FLIPPER_POS.y);
const sf::Color FLIPPER_COLOR = sf::Color::Blue;
const float FLIPPER_INIT_ANGLE = 25.f/2;
const float FLIPPER_ANGLE_SPEED = 1.f/2;

const sf::Vector2f LAUNCH_SPRING_SIZE(BALL_RADIUS * 2.f * 1.75f, 100.);
const sf::Vector2f LAUNCH_SPRING_POS(WINDOW_WIDTH - LAUNCH_SPRING_SIZE.x, WINDOW_HEIGHT - LAUNCH_SPRING_SIZE.y);
const sf::Color LAUNCH_SPRING_COLOR = sf::Color(192, 192, 192);
const float LAUNCH_SPRING_COMPRESS_SPEED = .4f/2;
const float MIN_SHRINK_PERCENT = .25f/2;

const sf::Vector2f LAUNCH_WALL_SIZE(20./2, (WINDOW_HEIGHT - BALL_RADIUS * 2.f * 4.f)/2);
const sf::Vector2f LAUNCH_WALL_POS(LAUNCH_SPRING_POS.x - LAUNCH_WALL_SIZE.x, WINDOW_HEIGHT - LAUNCH_WALL_SIZE.y);
const sf::Color LAUNCH_WALL_COLOR = sf::Color::Black;


const std::array<sf::Vector2f, 6> ROOF_POINTS = {
	sf::Vector2f(0.f, 0.f),
	sf::Vector2f(100.f, 0.f),
	sf::Vector2f(0.f, 100.f),
	sf::Vector2f(WINDOW_WIDTH - 100.f, 0.f),
	sf::Vector2f(WINDOW_WIDTH, 0.f),
	sf::Vector2f(WINDOW_WIDTH, 100.f)
};
const float RESTITUTION = .75f;

const float BALL_MASS = 1.f;
const float FLIPPER_MASS = 3.f;
const float SPRING_MASS = 5.f;
const float SPRING_CONSTANT = .01f;
const float GRAVITY_ACC = .00032f;
const sf::Color ROOF_COLOR = sf::Color::Black;

//Ruth:
const sf::Vector2f SCORE_POS((WINDOW_WIDTH - (LAUNCH_SPRING_SIZE.x * 5.f))/2, ((WINDOW_HEIGHT - LAUNCH_SPRING_SIZE.y)-500)/2);
const sf::Vector2f OB_POS((WINDOW_WIDTH - (LAUNCH_SPRING_SIZE.x * 5.f))/2, (WINDOW_HEIGHT - LAUNCH_SPRING_SIZE.y)/2);
const sf::Vector2f OB_POS_2((WINDOW_WIDTH - (LAUNCH_SPRING_SIZE.x * 5.f)), (WINDOW_HEIGHT - LAUNCH_SPRING_SIZE.y)/2);
const sf::Color BALL_COLOR_2 = sf::Color::Magenta;