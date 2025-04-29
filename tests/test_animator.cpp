#include <iostream>
#include <cassert>
#include "Eigen/Dense"
#include "../src/animator.h"

using namespace std;

float epsilon = 1e-4f;

bool isClose(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return (a - b).norm() < epsilon;
}

bool isClose(const Eigen::Quaternionf& a, const Eigen::Quaternionf& b) {
    return a.angularDistance(b) < epsilon;
}

void test_findPositionIndex() {
    std::cout << "Running test_findPositionIndex()..." << std::endl;

    Bone bone;
    bone.positionTimes = {0.0, 1.0, 2.0};

    assert(bone.findPositionIndex(-0.5) == 0);
    assert(bone.findPositionIndex(0.0) == 0);
    assert(bone.findPositionIndex(0.5) == 0);
    assert(bone.findPositionIndex(1.5) == 1);
    assert(bone.findPositionIndex(3.0) == 1);

    std::cout << "[PASSED] test_findPositionIndex()" << std::endl;

}

void test_findPositionIndex_advanced() {
    std::cout << "Running test_findPositionIndex_advanced()..." << std::endl;

    Bone bone;
    bone.positionTimes = {0.0, 0.5, 1.5, 3.0, 5.0};

    // Before first keyframe — clamp to first
    assert(bone.findPositionIndex(-10.0) == 0);
    assert(bone.findPositionIndex(0.0) == 0);

    // In between intervals
    assert(bone.findPositionIndex(0.1) == 0);
    assert(bone.findPositionIndex(0.5) == 1);
    assert(bone.findPositionIndex(1.0) == 1);
    assert(bone.findPositionIndex(2.0) == 2);
    assert(bone.findPositionIndex(3.0) == 3);
    assert(bone.findPositionIndex(4.0) == 3);

    // At and beyond last keyframe — return second-to-last index
    assert(bone.findPositionIndex(5.0) == 3);
    assert(bone.findPositionIndex(10.0) == 3);

    std::cout << "[PASSED] test_findPositionIndex_advanced()" << std::endl;
}

void test_findPositionIndex_empty() {
    std::cout << "Running test_findPositionIndex_empty()..." << std::endl;

    Bone bone;
    bone.positionTimes = {};  // no keyframes

    int result = bone.findPositionIndex(1.0);
    assert(result == -1);  // your function prints an error and returns -1

    std::cout << "[PASSED] test_findPositionIndex_empty()" << std::endl;
}



void test_interpolatePosition() {
    std::cout << "Running test_interpolatePosition()..." << std::endl;

    Bone bone;
    bone.positionTimes = {0.0, 1.0};
    bone.positionKeys = {
        Eigen::Vector3f(0, 0, 0),
        Eigen::Vector3f(10, 0, 0)
    };

    Eigen::Vector3f pos = bone.interpolatePosition(0.5);
    assert(isClose(pos, Eigen::Vector3f(5, 0, 0)));
    std::cout << "[PASSED] test_interpolatePosition()" << std::endl;

}

void test_interpolateRotation() {
    std::cout << "Running test_interpolateRotation()..." << std::endl;

    Bone bone;
    bone.rotationTimes = {0.0, 1.0};
    bone.rotationKeys = std::vector<Eigen::Quaternionf>{
        Eigen::Quaternionf::Identity(),
        Eigen::Quaternionf(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()))
    };
    

    Eigen::Quaternionf rot = bone.interpolateRotation(0.5);
    Eigen::Quaternionf expected = Eigen::Quaternionf(Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY()));

    assert(isClose(rot, expected));

    std::cout << "[PASSED] test_interpolateRotation()" << std::endl;

}

void test_interpolateScaling() {
    std::cout << "Running test_interpolateScaling()..." << std::endl;

    Bone bone;
    bone.scalingTimes = {0.0, 1.0};
    bone.scalingKeys = {
        Eigen::Vector3f(1, 1, 1),
        Eigen::Vector3f(2, 2, 2)
    };

    Eigen::Vector3f scale = bone.interpolateScaling(0.5);
    assert(isClose(scale, Eigen::Vector3f(1.5, 1.5, 1.5)));

    std::cout << "[PASSED] test_interpolateScaling()" << std::endl;

}

void test_buildLocalTransform() {
    std::cout << "Running test_buildLocalTransform()..." << std::endl;

    Bone bone;
    bone.positionTimes = {0.0, 1.0};
    bone.positionKeys = {
        Eigen::Vector3f(0, 0, 0),
        Eigen::Vector3f(10, 0, 0)
    };

    bone.rotationTimes = {0.0, 1.0};
    bone.rotationKeys = {
        Eigen::Quaternionf::Identity(),
        Eigen::Quaternionf::Identity()
    };

    bone.scalingTimes = {0.0, 1.0};
    bone.scalingKeys = {
        Eigen::Vector3f(1, 1, 1),
        Eigen::Vector3f(1, 1, 1)
    };

    Matrix4f t = bone.buildLocalTransform(0.5);
    Eigen::Vector4f translated = t * Eigen::Vector4f(0, 0, 0, 1);
    assert(isClose(translated.head<3>(), Eigen::Vector3f(5, 0, 0)));

    std::cout << "[PASSED] test_buildLocalTransform()" << std::endl;

}

void test_interpolateAt() {
    std::cout << "Running test_interpolateAt()..." << std::endl;

    Bone parent;
    parent.positionTimes = {0.0, 1.0};
    parent.positionKeys = {Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(1, 0, 0)};
    parent.rotationTimes = {0.0, 1.0};
    parent.rotationKeys = {Eigen::Quaternionf::Identity(), Eigen::Quaternionf::Identity()};
    parent.scalingTimes = {0.0, 1.0};
    parent.scalingKeys = {Eigen::Vector3f(1, 1, 1), Eigen::Vector3f(1, 1, 1)};

    Bone child;
    child.positionTimes = {0.0, 1.0};
    child.positionKeys = {Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 1, 0)};
    child.rotationTimes = {0.0, 1.0};
    child.rotationKeys = {Eigen::Quaternionf::Identity(), Eigen::Quaternionf::Identity()};
    child.scalingTimes = {0.0, 1.0};
    child.scalingKeys = {Eigen::Vector3f(1, 1, 1), Eigen::Vector3f(1, 1, 1)};

    vector<Bone*> children;
    children.push_back(&child);
    parent.children = children;

    parent.interpolateAt(0.5, Matrix4f::Identity());

    // Expect child to be halfway moved up
    Eigen::Vector4f result = child.globalTransformation * Eigen::Vector4f(0, 0, 0, 1);
    assert(isClose(result.head<3>(), Eigen::Vector3f(0.5, 0.5, 0)));

    std::cout << "[PASSED] test_interpolateAt()" << std::endl;

}

int main() {
    cout << "Running Bone tests..." << endl;
    
    test_findPositionIndex();
    test_findPositionIndex_advanced();
    test_findPositionIndex_empty();

    test_interpolatePosition();
    test_interpolateRotation();
    test_interpolateScaling();
    test_buildLocalTransform();
    test_interpolateAt();

    cout << "✅ All Bone tests passed!" << endl;
    return 0;
}
