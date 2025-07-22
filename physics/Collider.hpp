#pragma once

class Collider {
public:

	Collider() = default;

	
	virtual bool RayIntersection() const {};
	virtual bool PlaneIntersection() const {};
	virtual bool ColliderIntersection() const {};

};