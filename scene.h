#pragma once

#include "objects.h"
#include <algorithm>
class Scene {
public:
    LaunchWall launchWall;
    LaunchSpring launchSpring;
    std::vector<Ball> balls;
    LeftFlipper leftFlipper;
    RightFlipper rightFlipper;
    Roof roof;
    Scene() :
        leftFlipper(FLIPPER_SIZE, LEFT_FLIPPER_POS, FLIPPER_COLOR, FLIPPER_INIT_ANGLE),
        rightFlipper(FLIPPER_SIZE, RIGHT_FLIPPER_POS, FLIPPER_COLOR, FLIPPER_INIT_ANGLE),
        launchWall(LAUNCH_WALL_POS, LAUNCH_WALL_SIZE, LAUNCH_WALL_COLOR),
        launchSpring(LAUNCH_SPRING_POS, LAUNCH_SPRING_SIZE, LAUNCH_SPRING_COLOR),
        roof(ROOF_POINTS, ROOF_COLOR) {
        balls.emplace_back(BALL_RADIUS, sf::Vector2f(LAUNCH_SPRING_POS.x + LAUNCH_SPRING_SIZE.x / 2, LAUNCH_SPRING_POS.y - BALL_RADIUS), BALL_COLOR, BALL_MASS, 0);
    }
    
    void update(bool addBall) {
        if (addBall) {
            std::cout << "enter" << std::endl;
            int order = -1;
            for (int i = 0;; i++) {
                sf::Vector2f curCenter = sf::Vector2f(LAUNCH_SPRING_POS.x + LAUNCH_SPRING_SIZE.x / 2, LAUNCH_SPRING_POS.y - BALL_RADIUS* (2 * i + 1));
                if (curCenter.y <= BALL_RADIUS) break;
                if (std::all_of(balls.begin(), balls.end(), [&](const Ball& ball) {
                    return vector2fLengthSquare(ball.getCenter() - curCenter) > BALL_RADIUS*BALL_RADIUS*4.f;
                })) {
                    order = i;
                    break;
                }
            }
            std::cout << "order: " << order << std::endl;
            if (order >= 0)
                balls.emplace_back(BALL_RADIUS, sf::Vector2f(LAUNCH_SPRING_POS.x + LAUNCH_SPRING_SIZE.x / 2, LAUNCH_SPRING_POS.y - BALL_RADIUS * (2 * order + 1)), BALL_COLOR, BALL_MASS, balls.size());
        }
        launchSpring.compress(LAUNCH_SPRING_COMPRESS_SPEED);
        leftFlipper.update();
        rightFlipper.update();
        for (auto& ball: balls)
            ball.update(launchSpring, launchWall, roof, leftFlipper, rightFlipper, balls);
    }

    void draw(sf::RenderWindow& window) {
        window.draw(launchWall.getShape());
        window.draw(launchSpring.getShape());
        for (auto& ball: balls)
            window.draw(ball.getShape());
        window.draw(leftFlipper.getShape());
        window.draw(rightFlipper.getShape());
        window.draw(roof.getLeftSide());
        window.draw(roof.getRightSide());
    }
};