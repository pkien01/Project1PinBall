#pragma once

#include <SFML/Graphics.hpp>
#include "constants.h"
#include <array>
#include "scene.h"

inline float vector2fLengthSquare(const sf::Vector2f& vec) {
    return vec.x * vec.x + vec.y * vec.y;
}
inline float vector2fDot(const sf::Vector2f& vec1, const sf::Vector2f& vec2) {
    return vec1.x * vec2.x + vec1.y * vec2.y;
}
inline sf::Vector2f vector2fNormalize(const sf::Vector2f& vec, const float newLengthSquare = 1.f) {
    return vec*std::sqrtf(newLengthSquare / vector2fLengthSquare(vec));
}
inline float clip(float val, float l, float r) {
    if (val < l) return l;
    if (val > r) return r;
    return val;
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
    float initAngle;
    Flipper(sf::Vector2f pivot, sf::Color color, float angle): initAngle(angle) {
        shape.setOrigin(pivot);
        shape.setPosition(pivot);
        shape.setFillColor(color);
        shape.setPointCount(4);
    }
public:
    virtual void rotate(const float) = 0;
    sf::ConvexShape getShape() const {
        return shape;
    }
};

class LeftFlipper: public Flipper {
public:
    LeftFlipper(sf::Vector2f size, sf::Vector2f pivot, sf::Color color, float angle): Flipper(pivot, color, angle) {
        shape.setPoint(0, pivot + sf::Vector2f(-size.y, 0));
        shape.setPoint(1, pivot + sf::Vector2f(0, -size.y));
        shape.setPoint(2, pivot + sf::Vector2f(size.x, 0));
        shape.setPoint(3, pivot + sf::Vector2f(0, size.y));
        shape.setRotation(angle);
    }

    void rotate(const float angleSpeed) {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
            float nextAngle = shape.getRotation() - angleSpeed;
            if (nextAngle <= initAngle || nextAngle >= 360.f - initAngle) {
                shape.rotate(-angleSpeed);
            }
        }
        else {
            float nextAngle = shape.getRotation() + angleSpeed;
            if (nextAngle <= initAngle || nextAngle >= 360.f - initAngle) {
                shape.rotate(angleSpeed);
            }
        }
    }
};

class RightFlipper : public Flipper {
public:
    RightFlipper(sf::Vector2f size, sf::Vector2f pivot, sf::Color color, float angle) : Flipper(pivot, color, angle) {
        shape.setPoint(0, pivot + sf::Vector2f(-size.x, 0));
        shape.setPoint(1, pivot + sf::Vector2f(0, -size.y));
        shape.setPoint(2, pivot + sf::Vector2f(size.y, 0));
        shape.setPoint(3, pivot + sf::Vector2f(0, size.y));
        shape.setRotation(-angle);
    }

    void rotate(const float angleSpeed) {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
            float nextAngle = shape.getRotation() + angleSpeed;
            if (nextAngle <= initAngle || nextAngle >= 360.f - initAngle) {
                shape.rotate(angleSpeed);
            }
        }
        else {
            float nextAngle = shape.getRotation() - angleSpeed;
            if (nextAngle <= initAngle || nextAngle >= 360.f - initAngle) {
                shape.rotate(-angleSpeed);
            }
        }
    }
};

class LaunchWall {
private: 
    sf::RectangleShape shape;
public:
    LaunchWall(sf::Vector2f pos, sf::Vector2f size, sf::Color color) {
        shape.setPosition(pos);
        shape.setSize(size);
        shape.setFillColor(color);
    }
    sf::RectangleShape getShape() const {
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
    bool hasReleased;
    float maxCompress;
public:
    LaunchSpring(sf::Vector2f pos, sf::Vector2f size, sf::Color color) {
        shape.setPosition(pos);
        shape.setSize(size);
        shape.setFillColor(color);
        initSize = size;
        initPos = pos;
        maxCompress = 0.f;
        hasReleased = false;
    }
    void compress(float speed) {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
            float nextHeight = std::max(initSize.y / 4, shape.getSize().y - speed);
            shape.setSize(sf::Vector2f(initSize.x, nextHeight));
            maxCompress = std::max(maxCompress, initSize.y - nextHeight);
        }
        else if (maxCompress > 0.f) {
            float nextHeight = shape.getSize().y + speed;
            if (nextHeight >= initSize.y) {
                hasReleased = true;
                nextHeight = initSize.y;
            }
            shape.setSize(sf::Vector2f(initSize.x, nextHeight));
        }
        shape.setPosition(initPos.x, initPos.y + initSize.y - shape.getSize().y);
    }
    sf::RectangleShape getShape() const {
        return shape;
    }
    bool released() const {
        return hasReleased;
    }
    float maxCompression() const {
        return maxCompress;
    }
    void relaunch() {
        maxCompress = 0.f;
        hasReleased = false;
    }
};

class Ball {
private:
    sf::CircleShape shape;
    sf::Vector2f velocity;
    bool hasLaunched;
    float radius;

public:
    Ball(float inputRadius, sf::Color color) {
        shape.setRadius(inputRadius);
        shape.setFillColor(color);
        velocity = sf::Vector2f(0.f, 0.f); // Initial velocity
        radius = inputRadius;
        hasLaunched = false;
    }

    void update(LaunchSpring &spring, const Roof &roof, const LeftFlipper& leftFlipper, const RightFlipper &rightFlipper) {
        sf::CircleShape old_shape = shape;
        shape.move(velocity);
        if (hasLaunched) {
            // if collide, need to find a way to display it just as when it touches the shape
            if (collide(spring)) {
                shape = old_shape;
                reset(spring);
                hasLaunched = false;
                spring.relaunch();
            }
            if (collideAndReflectPolygon(roof.getLeftSide())) {
                std::cout << "Collided with left side roof." << std::endl;
            }
            else if (collideAndReflectPolygon(roof.getRightSide())) {
                std::cout << "Collided with right side roof." <<  std::endl;
            }
            else if (collideAndReflectPolygon(leftFlipper.getShape())) {
                std::cout << "Collided with left flipper." << std::endl;
            }
            else if (collideAndReflectPolygon(rightFlipper.getShape())) {
                std::cout << "Collided with right flipper." << std::endl;
            }
            else if (collideAndReflectScreen()) {
                std::cout << "Collided with the screen." << std::endl;
            }

        }
        else launch(spring);
        velocity.y += GRAVITY_ACC;
    }
    void reset(const LaunchSpring &spring) {
        sf::Vector2f springSize = spring.getShape().getSize();
        sf::Vector2f springPos = spring.getShape().getPosition();
        shape.setPosition(sf::Vector2f(springPos.x + springSize.x / 2 - radius, springPos.y - radius * 2.f));
        velocity = sf::Vector2f(0.f, 0.f);
    }
    bool collide(const LaunchSpring& spring) {
        return shape.getGlobalBounds().intersects(spring.getShape().getGlobalBounds());
    }

    bool collideAndReflectLine(const sf::Vector2f from, const sf::Vector2f to) {
        sf::Vector2f center = shape.getPosition();
        sf::Vector2f closestVec;

        float centerDotFrom = vector2fDot(center - from, to - from), centerDotTo = vector2fDot(center - to, from - to);
        if (centerDotFrom < 0) closestVec = from - center;
        else if (centerDotTo < 0) closestVec = to - center;
        else {
            sf::Vector2f projPoint = from + (to - from)*(centerDotFrom / vector2fLengthSquare(to - from));
            closestVec = projPoint - center;
        }
        
        float distToClosest = std::sqrtf(vector2fLengthSquare(closestVec));
        if (distToClosest > radius) return false;
        shape.setPosition(center - vector2fNormalize(velocity, (radius - distToClosest)*(radius - distToClosest)));
        velocity = vector2fNormalize(velocity - closestVec * 2.f, vector2fLengthSquare(velocity)* RESTITUTION);
        return true;
    }
    bool collideAndReflectScreen() {
        sf::Vector2f center = shape.getPosition();
        bool returnVal = false;
        if (center.x - radius < 0) {
            velocity = vector2fNormalize(velocity - sf::Vector2f(-center.x, 0) * 2.f, vector2fLengthSquare(velocity) * RESTITUTION);
            returnVal = true;
        }
        else if (center.y - radius < 0) {
            velocity = vector2fNormalize(velocity - sf::Vector2f(0, -center.y) * 2.f, vector2fLengthSquare(velocity) * RESTITUTION);
            returnVal = true;
        }
        else if (center.x + radius > WINDOW_WIDTH) {
            velocity = vector2fNormalize(velocity - sf::Vector2f(WINDOW_WIDTH-center.x, 0) * 2.f, vector2fLengthSquare(velocity) * RESTITUTION);
            returnVal = true;
        }
        else if (center.y + radius > WINDOW_HEIGHT) {
            velocity = vector2fNormalize(velocity - sf::Vector2f(0, WINDOW_HEIGHT - center.y) * 2.f, vector2fLengthSquare(velocity) * RESTITUTION);
            returnVal = true;
        }

        shape.setPosition(clip(center.x, 0, WINDOW_WIDTH), clip(center.y, 0, WINDOW_HEIGHT));
        return returnVal;
    }
    bool collideAndReflectPolygon(const sf::ConvexShape& convex) {
        int numVertices = convex.getPointCount();
        for (int i = 0; i < numVertices - 1; i++)
            if (collideAndReflectLine(convex.getPoint(i), convex.getPoint(i + 1))) return true;
        if (collideAndReflectLine(convex.getPoint(numVertices - 1), convex.getPoint(0))) return true;
        return false;
    }
    void launch(const LaunchSpring& spring) {
        if (hasLaunched) return;
        sf::Vector2f springSize = spring.getShape().getSize();
        sf::Vector2f springPos = spring.getShape().getPosition();
        if (!spring.released())
            shape.setPosition(sf::Vector2f(springPos.x + springSize.x / 2 - radius, springPos.y - radius * 2.f));
        else if (!hasLaunched) {
            //std::cout << "maximum compression: " << spring.maxCompression() << std::endl;
            velocity.y = -spring.maxCompression() / springSize.y * 1.25f;
            hasLaunched = true;
        }
    }
    sf::CircleShape getShape() const {
        return shape;
    }
};

