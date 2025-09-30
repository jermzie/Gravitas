#ifndef AABB_HPP
#define AABB_HPP

#include <glm/glm.hpp>

struct AABB{
	glm::vec3 upperBound;
	glm::vec3 lowerBound;
};

bool intersects(const AABB& a, const AABB& b) {
    return (
        a.lowerBound.x <= b.upperBound.x &&
        a.upperBound.x >= b.lowerBound.x &&
        a.lowerBound.y <= b.upperBound.y &&
        a.upperBound.y >= b.lowerBound.y &&
        a.lowerBound.z <= b.upperBound.z &&
        a.upperBound.z >= b.lowerBound.z
    );
}

AABB combineAABBs(const AABB& a, const AABB& b) {

    AABB c;
    c.lowerBound = glm::min(a.lowerBound, b.lowerBound);
    c.upperBound = glm::max(a.upperBound, b.upperBound);

    return c;
}

// cost of AABB -- want to minimize BVH node surface area
float surfaceArea(const AABB& a) {

    // sa of rectangular prism:
    // 2(lw + lh + wh)
    glm::vec3 d = a.upperBound - a.lowerBound;
    return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
}





#endif
