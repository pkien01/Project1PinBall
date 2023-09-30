#pragma once

#include "objects.h"

class Scene {
public:
    LaunchWall launchWall;
    LaunchSpring launchSpring;
    Ball ball;
    Ball ball_2;
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
        obstacle_circle(OB_BALL_RADIUS, OB_BALL_COLOR, OB_POS, OB_MASS),
        ball(BALL_RADIUS, BALL_COLOR, launchSpring, BALL_MASS, 0),
        ball_2(BALL_RADIUS, BALL_COLOR_2, launchSpring, BALL_MASS, 1){}
    
    void update() {
        launchSpring.compress(LAUNCH_SPRING_COMPRESS_SPEED);
        leftFlipper.update();
        rightFlipper.update();
        ball.update(launchSpring, launchWall, roof, leftFlipper, rightFlipper, obstacle_circle, ball_2);
        ball_2.update(launchSpring, launchWall, roof, leftFlipper, rightFlipper, obstacle_circle, ball);
    }

    void draw(sf::RenderWindow& window) {
        window.draw(launchWall.getShape());
        window.draw(launchSpring.getShape());
        window.draw(ball.getShape());
         window.draw(ball_2.getShape());
        window.draw(leftFlipper.getShape());
        window.draw(rightFlipper.getShape());
        window.draw(roof.getLeftSide());
        window.draw(roof.getRightSide());
        window.draw(obstacle_circle.getShape());
    }
};