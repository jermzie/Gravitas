#ifndef AABB_HPP
#define AABB_HPP

#include <glm/glm.hpp>

struct AABB{
	glm::vec3 max;
	glm::vec3 min;
};

bool intersects(const AABB& a, const AABB& b) {
    return (
        a.min.x <= b.max.x &&
        a.max.x >= b.min.x &&
        a.min.y <= b.max.y &&
        a.max.y >= b.min.y &&
        a.min.z <= b.max.z &&
        a.max.z >= b.min.z
    );
}

AABB combineAABBs(const AABB& a, const AABB& b) {

    AABB c;
    c.min = glm::min(a.min, b.min);
    c.max = glm::max(a.max, b.max);

    return c;
}

// cost of AABB -- want to minimize BVH node surface area
float surfaceArea(const AABB& a) {

    // sa of rectangular prism:
    // 2(lw + lh + wh)
    glm::vec3 d = a.max - a.min;
    return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
}

float volume(const AABB& a) {

    // vol of rectangular prism:
    // l * w * h
    glm::vec3 d = a.max - a.min;
    return d.x * d.y * d.z;
}





#endif
