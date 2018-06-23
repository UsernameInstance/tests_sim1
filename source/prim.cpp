#include "prim.h"

prim::prim(double radius, double mass, double bounciness, Eigen::Vector2d position, Eigen::Vector2d velocity, enum type type, texture* p_texture) 
	: radius(radius), mass(mass), bounciness(bounciness), position(position), velocity(velocity), type(type), p_texture(p_texture) {}

prim::prim() : radius(0), mass(0), bounciness(0), position{Eigen::Vector2d(0,0)}, velocity{Eigen::Vector2d(0,0)}, type(type::interior) {} 

prim::~prim() {}

void prim::render()
{
	if(p_texture)
		p_texture->render(position(0) - radius, position(1) - radius); //no camera yet, offset depending on center of mass placement in texture coordinates
}
