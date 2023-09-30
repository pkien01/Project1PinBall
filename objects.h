#pragma once

#include <SFML/Graphics.hpp>
#include "constants.h"
#include <array>
#include "scene.h"

inline float vector2fLengthSquare(const sf::Vector2f& vec) {
    return vec.x * vec.x + vec.y * vec.y;
}
inline float vector2fLength(const sf::Vector2f& vec) {
    return sqrtf(vector2fLengthSquare(vec));
}
inline float vector2fDot(const sf::Vector2f& vec1, const sf::Vector2f& vec2) {
    return vec1.x * vec2.x + vec1.y * vec2.y;
}
inline sf::Vector2f vector2fNormalize(const sf::Vector2f& vec, const float newLength = 1.f) {
    return vec*newLength / vector2fLength(vec);
}
inline float clip(float val, float l, float r) {
    if (val < l) return l;
    if (val > r) return r;
    return val;
}
inline sf::Vector2f rotatePoint(const sf::Vector2f& curPoint, float angle) {
    float sin_angle = sinf(angle), cos_angle = cosf(angle);
    return {cos_angle * curPoint.x - sin_angle * curPoint.y, sin_angle * curPoint.x + cos_angle * curPoint.y };
}
template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec) {
    os << "[";
    bool first = true;
    for (auto& item : vec) {
        if (first) first = false;
        else os << ", ";
        os << item;
    }
    os << "]" << std::endl;
    return os;
}
std::ostream& operator<<(std::ostream& os, const sf::Vector2f vec) {
    os << "Vector2f(" << vec.x << ", " << vec.y << ")";
    return os;
}

sf::ConvexShape rectangleToConvex(const sf::RectangleShape& rectangle)
{
    sf::ConvexShape convexShape;
    convexShape.setPointCount(4);

    const sf::Vector2f& position = rectangle.getPosition();
    const sf::Vector2f& size = rectangle.getSize();

    convexShape.setPoint(0, position);
    convexShape.setPoint(1, position + sf::Vector2f(size.x, 0));
    convexShape.setPoint(2, position + size);
    convexShape.setPoint(3, position + sf::Vector2f(0, size.y));

    convexShape.setFillColor(rectangle.getFillColor());

    return convexShape;
}


class Flipper {
protected:    
    sf::ConvexShape shape;
    float initAngle, curAngle;
    float angleVelocity;
    float pivot_x, pivot_y;
    Flipper(sf::Vector2f pivot, sf::Color color, float angle): initAngle(angle), pivot_x(pivot.x), pivot_y(pivot.y), curAngle(0.) {
        shape.setOrigin(pivot);
        shape.setPosition(pivot);
        shape.setFillColor(color);
        shape.setPointCount(5);
    }
    void rotateShape(float angle) {
        curAngle += angle;
        while (curAngle < 0) curAngle += 360.;
        while (curAngle >= 360.) curAngle -= 360.;
        sf::Transform transform;
        transform.rotate(angle, pivot_x, pivot_y);
        for (int i = 0; i < 5; i++)
            shape.setPoint(i, transform.transformPoint(shape.getPoint(i)));
    }

    float getRotationShape() const {
        return curAngle;
    }
public:
    virtual void rotate() = 0;
    sf::ConvexShape getShape() const {
        return shape;
    }
    sf::Vector2f getPivot() const {
        return sf::Vector2f(pivot_x, pivot_y);
    }
    void update() {
        rotate();
        rotateShape(angleVelocity);
    }
    float getAngleVelocity() const {
        return angleVelocity;
    }
};

class LeftFlipper: public Flipper {
public:
    LeftFlipper(sf::Vector2f size, sf::Vector2f pivot, sf::Color color, float angle): Flipper(pivot, color, angle) {
        shape.setPoint(0, pivot + sf::Vector2f(-size.y, 0));
        shape.setPoint(1, pivot + sf::Vector2f(0, -size.y));
        shape.setPoint(2, pivot + sf::Vector2f(size.x, -size.y / 2));
        shape.setPoint(3, pivot + sf::Vector2f(size.x, size.y / 2));
        shape.setPoint(4, pivot + sf::Vector2f(0, size.y));
        rotateShape(angle);
    }

    void rotate() {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
            float nextAngle = getRotationShape() - FLIPPER_ANGLE_SPEED;
            if (nextAngle <= initAngle || nextAngle >= 360.f - initAngle) {
                angleVelocity = -FLIPPER_ANGLE_SPEED;
            }
            else angleVelocity = 0.f;
        }
        else {
            float nextAngle = getRotationShape() + FLIPPER_ANGLE_SPEED;
            if (nextAngle <= initAngle || nextAngle >= 360.f - initAngle) {
                angleVelocity = FLIPPER_ANGLE_SPEED;
            }
            else angleVelocity = 0.f;
        }
    }
};

class RightFlipper : public Flipper {
public:
    RightFlipper(sf::Vector2f size, sf::Vector2f pivot, sf::Color color, float angle) : Flipper(pivot, color, angle) {
        shape.setPoint(0, pivot + sf::Vector2f(-size.x, size.y / 2));
        shape.setPoint(1, pivot + sf::Vector2f(-size.x, -size.y / 2));
        shape.setPoint(2, pivot + sf::Vector2f(0, -size.y));
        shape.setPoint(3, pivot + sf::Vector2f(size.y, 0));
        shape.setPoint(4, pivot + sf::Vector2f(0, size.y));
        rotateShape(-angle);
    }

    void rotate() {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
            float nextAngle = getRotationShape() + FLIPPER_ANGLE_SPEED;
            if (nextAngle <= initAngle || nextAngle >= 360.f - initAngle) {
                angleVelocity = FLIPPER_ANGLE_SPEED;
            }
            else angleVelocity = 0.f;
        }
        else{
            float nextAngle = getRotationShape() - FLIPPER_ANGLE_SPEED;
            if (nextAngle <= initAngle || nextAngle >= 360.f - initAngle) {
                angleVelocity = -FLIPPER_ANGLE_SPEED;
            }
            else angleVelocity = 0.f;
        }
    }
};


class LaunchWall {
    sf::ConvexShape shape;
public:
    LaunchWall(sf::Vector2f pos, sf::Vector2f size, sf::Color color) {
        shape.setPointCount(4);
        shape.setPoint(0, pos);
        shape.setPoint(1, sf::Vector2f(pos.x, pos.y + size.y));
        shape.setPoint(2, sf::Vector2f(pos.x + size.x, pos.y + size.y));
        shape.setPoint(3, sf::Vector2f(pos.x + size.x, pos.y + size.x));
        shape.setFillColor(color);
    }
    sf::ConvexShape getShape() const {
        return shape;
    }
};

class Roof {
private:
    sf::ConvexShape leftTriangle;
    sf::ConvexShape rightTriangle;
public:
    Roof(const std::array<sf::Vector2f, 6>& points, sf::Color color) {
        leftTriangle.setPointCount(3);
        for (int i = 0; i < 3; i++) leftTriangle.setPoint(i, points[i]);
        leftTriangle.setFillColor(color);
        rightTriangle.setPointCount(3);
        for (int i = 0; i < 3; i++) rightTriangle.setPoint(i, points[i+3]);
        rightTriangle.setFillColor(color);
    }
    sf::ConvexShape getLeftSide() const {
        return leftTriangle;
    }
    sf::ConvexShape getRightSide() const {
        return rightTriangle;
    }
};

class LaunchSpring {
private:
    sf::RectangleShape shape;
    sf::Vector2f initSize, initPos;
    float maxCompress;
    float hit;
public:
    LaunchSpring(sf::Vector2f pos, sf::Vector2f size, sf::Color color) {
        shape.setPosition(pos);
        shape.setSize(size);
        shape.setFillColor(color);
        initSize = size;
        initPos = pos;
        maxCompress = 0.f;
        hit = false;
    }
    void compress(float speed) {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
            float nextHeight = std::max(initSize.y * MIN_SHRINK_PERCENT, shape.getSize().y - speed);
            shape.setSize(sf::Vector2f(initSize.x, nextHeight));
            maxCompress = std::max(maxCompress, initSize.y - nextHeight);
        }
        else {
            float nextHeight = shape.getSize().y + speed;
            if (nextHeight > initSize.y) {
                nextHeight = initSize.y;
                if (!hit) reset();
            }
            shape.setSize(sf::Vector2f(initSize.x, nextHeight));
        }
        shape.setPosition(initPos.x, initPos.y + initSize.y - shape.getSize().y);
    }
    sf::RectangleShape getShape() const {
        return shape;
    }
    float maxCompression() {
        return maxCompress;
    }
    void reset() {
        maxCompress = 0.f;
    }
    void setHit(bool hitVal) {
        hit = hitVal;
    }
    float getSpeed() {
        return sqrtf(2.f * SPRING_CONSTANT * maxCompress * maxCompress / SPRING_MASS);
    }
};

class Ball {
private:
    sf::CircleShape shape;
    sf::Vector2f velocity;
    float radius;
    float mass;
    int id;

public:
    Ball(float inputRadius, sf::Vector2f inputCenter, sf::Color color, const float ballMass, int inputId) {
        shape.setRadius(inputRadius);
        shape.setFillColor(color);
        velocity = sf::Vector2f(0.f, 0.f); // Initial velocity
        radius = inputRadius;
        mass = ballMass;
        id = inputId;
        shape.setPosition(inputCenter.x - radius, inputCenter.y - radius);
    }
    sf::Vector2f getCenter() const {
        return shape.getPosition() + sf::Vector2f(radius, radius);
    }

    void update(LaunchSpring &spring, const LaunchWall &launchWall, const Roof &roof, const LeftFlipper& leftFlipper, const RightFlipper &rightFlipper, 
        const std::vector<Ball> &balls) {
        shape.move(velocity);
        //std::cout << velocity << std::endl;
        if (collideAndReflectPolygon(rectangleToConvex(spring.getShape()), SPRING_MASS, { 0.f, -spring.getSpeed() })) {
            //std::cout << "Collided with spring." << std::endl;
            spring.setHit(true);
            spring.reset();
        }
        else {
            spring.setHit(false);
        }
        if (collideAndReflectPolygon(roof.getLeftSide())) {
            //std::cout << "Collided with left side roof." << std::endl;
        }
        if (collideAndReflectPolygon(roof.getRightSide())) {
           // std::cout << "Collided with right side roof." <<  std::endl;
        }
        if (collideAndReflectPolygon(launchWall.getShape())) {
          //  std::cout << "Collided with launch wall." << std::endl;
        }
        if (collideAndReflectPolygon(leftFlipper.getShape(), FLIPPER_MASS, leftFlipper.getAngleVelocity()*(getCenter() - leftFlipper.getPivot()))) {
          //  std::cout << velocity << " " << leftFlipper.getAngleVelocity() * vector2fNormalize(getCenter() - leftFlipper.getPivot()) << std::endl;
           // std::cout << "Collided with left flipper." << std::endl;
        }
        if (collideAndReflectPolygon(rightFlipper.getShape(), FLIPPER_MASS, rightFlipper.getAngleVelocity() * vector2fNormalize(getCenter() - rightFlipper.getPivot()))) {
          //  std::cout << velocity << " " << rightFlipper.getAngleVelocity() * vector2fNormalize(getCenter() - rightFlipper.getPivot()) << std::endl;
           // std::cout << "Collided with right flipper." << std::endl;
        }
        if (collideAndReflectScreen()) {
           // std::cout << "Collided with the screen." << std::endl;
        }
        for (const auto& ball : balls) {
            if (ball.id != id && collideAndReflectCircle(ball.getCenter(), ball.radius, ball.mass, ball.velocity)) {
                std::cout << "Ball " << id << " collide with ball " << ball.id << std::endl;
            }
        }
        velocity.y += GRAVITY_ACC;
       
    }
    void reset(const LaunchSpring &spring) {
        sf::Vector2f springSize = spring.getShape().getSize();
        sf::Vector2f springPos = spring.getShape().getPosition();
        shape.setPosition(sf::Vector2f(springPos.x + springSize.x / 2 - radius, springPos.y - radius * 2.f));
        velocity = sf::Vector2f(0.f, 0.f);
    }
    bool collideWithSpring(const LaunchSpring& spring) {
        return shape.getGlobalBounds().intersects(spring.getShape().getGlobalBounds());
    }
    bool collideAndReflectCircle(const sf::Vector2f otherCenter, const float otherRadius, const float otherMass = 0., const sf::Vector2f& otherVelocity = sf::Vector2f(0., 0.)) {
        float sumRadius = radius + otherRadius;
        sf::Vector2f delta = getCenter() - otherCenter;
        float distSquare = vector2fLengthSquare(delta);
        //std::cout << sqrtf(distSquare) << " " << sumRadius << std::endl;
        if (distSquare > sumRadius * sumRadius) return false;
      
        sf::Vector2f dir = vector2fNormalize(delta);
        float v1Dir = vector2fDot(velocity, dir), v2Dir = vector2fDot(otherVelocity, dir);
        float momentumVelocity = (mass * v1Dir + otherMass * v2Dir - otherMass * (v1Dir - v2Dir) * RESTITUTION) / (mass + otherMass);
        velocity += vector2fNormalize(dir, momentumVelocity);
        return true;
    }

    bool collideAndReflectLine(const sf::Vector2f from, const sf::Vector2f to, const float shapeMass = 0., const sf::Vector2f &shapeVelocity = sf::Vector2f(0., 0.)) {
        sf::Vector2f center = getCenter();
        sf::Vector2f closestVec;

        float centerDotFrom = vector2fDot(center - from, to - from), centerDotTo = vector2fDot(center - to, from - to);
        if (centerDotFrom < 0) {
            closestVec = from - center;
        }
        else if (centerDotTo < 0) {
            closestVec = to - center;
        }
        else {
            sf::Vector2f projPoint = from + (to - from) * (centerDotFrom / vector2fLengthSquare(to - from));
            closestVec = projPoint - center;
        }

        float distToClosest = std::sqrtf(vector2fLengthSquare(closestVec));
        if (distToClosest > radius) return false;
        shape.setPosition(shape.getPosition() - vector2fNormalize(closestVec, radius - distToClosest));

        sf::Vector2f momentumVelocity(0.f, 0.f);
        if (shapeMass > 0.f) momentumVelocity = (mass * 2.f * velocity + (shapeMass - mass) * shapeVelocity) / (mass + shapeMass);
        velocity = vector2fNormalize(velocity - closestVec * 2.f, vector2fLength(velocity) * RESTITUTION) + momentumVelocity;
        return true;
    }
    bool collideAndReflectScreen() {
        sf::Vector2f center = getCenter();
        bool returnVal = false;
        if (center.x - radius < 0) {
            velocity = vector2fNormalize(velocity - sf::Vector2f(-center.x, 0) * 2.f, vector2fLength(velocity) * RESTITUTION);
            returnVal = true;
        }
        else if (center.y - radius < 0) {
            velocity = vector2fNormalize(velocity - sf::Vector2f(0, -center.y) * 2.f, vector2fLength(velocity) * RESTITUTION);
            returnVal = true;
        }
        else if (center.x + radius > WINDOW_WIDTH) {
            velocity = vector2fNormalize(velocity - sf::Vector2f(WINDOW_WIDTH-center.x, 0) * 2.f, vector2fLength(velocity) * RESTITUTION);
            returnVal = true;
        }
        else if (center.y + radius > WINDOW_HEIGHT) {
            velocity = vector2fNormalize(velocity - sf::Vector2f(0, WINDOW_HEIGHT - center.y) * 2.f, vector2fLength(velocity) * RESTITUTION);
            returnVal = true;
        }

        shape.setPosition(clip(center.x, radius, WINDOW_WIDTH - radius) - radius, clip(center.y, radius, WINDOW_HEIGHT - radius) - radius);
        return returnVal;
    }
    bool collideAndReflectPolygon(const sf::ConvexShape& convex, const float shapeMass = 0., const sf::Vector2f& shapeVelocity = sf::Vector2f(0., 0.)) {
        size_t numVertices = convex.getPointCount();
        for (int i = 0; i < numVertices - 1; i++)
            if (collideAndReflectLine(convex.getPoint(i), convex.getPoint(i + 1), shapeMass, shapeVelocity)) return true;
        if (collideAndReflectLine(convex.getPoint(numVertices - 1), convex.getPoint(0), shapeMass, shapeVelocity)) return true;
        return false;
    }
    sf::CircleShape getShape() const {
        return shape;
    }
};

