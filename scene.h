#pragma once

#include "objects.h"

class Scene {
public:
    LaunchWall launchWall;
    LaunchSpring launchSpring;
    Ball ball;
    LeftFlipper leftFlipper;
    RightFlipper rightFlipper;
    Roof roof;
    Obstacle_Circle obstacle_circle;
    Scene() :
        leftFlipper(FLIPPER_SIZE, LEFT_FLIPPER_POS, FLIPPER_COLOR, FLIPPER_INIT_ANGLE),
        rightFlipper(FLIPPER_SIZE, RIGHT_FLIPPER_POS, FLIPPER_COLOR, FLIPPER_INIT_ANGLE),
        launchWall(LAUNCH_WALL_POS, LAUNCH_WALL_SIZE, LAUNCH_WALL_COLOR),
        launchSpring(LAUNCH_SPRING_POS, LAUNCH_SPRING_SIZE, LAUNCH_SPRING_COLOR),
        roof(ROOF_POINTS, ROOF_COLOR),
        obstacle_circle(OB_BALL_RADIUS, OB_BALL_COLOR, OB_POS),
        ball(BALL_RADIUS, BALL_COLOR, launchSpring) {}
    
    void update() {
        launchSpring.compress(LAUNCH_SPRING_COMPRESS_SPEED);
        leftFlipper.rotate();
        rightFlipper.rotate();
        ball.update(launchSpring, launchWall, roof, leftFlipper, rightFlipper, obstacle_circle);
    }

    void draw(sf::RenderWindow& window) {
        window.draw(launchWall.getShape());
        window.draw(launchSpring.getShape());
        window.draw(ball.getShape());
        window.draw(leftFlipper.getShape());
        window.draw(rightFlipper.getShape());
        window.draw(roof.getLeftSide());
        window.draw(roof.getRightSide());
        window.draw(obstacle_circle.getShape());
    }
};