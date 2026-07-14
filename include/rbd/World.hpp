#pragma once
#include <vector>
#include <Eigen/Dense>
#include "RigidBody.hpp"

namespace rbd {

class World {
public:
    std::vector<RigidBody*> bodies;
    Eigen::Vector3d gravity;

    World();

    void addBody(RigidBody* body);
    void step(double dt);
};

} // namespace rbd