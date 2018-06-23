#ifndef PRIM_H
#define PRIM_H
#include <Eigen/Dense>
#include "texture.h"
#include <vector>
#include <limits>
#include <cmath>

struct prim //primitive. circles (and circle complements) without rotation only for now.
{
	enum class type
	{
		interior, //circle interior outward pointing normal
		exterior, //complement of circle interior inward pointing normal
	};

	enum type type;
	double radius; 
	double mass; //do not set to std::numeric_limits<double>::infinity(), use max() instead
        double bounciness; //0<= bounciness <=1; 0 minimum bounciness, 1 maximum bounciness;
	//0 bounciness object colliding with 0 bounciness object gives (approximately) perfectly inelastic collision
	//1 bounciness object colliding with 1 bounciness object gives (approximately) perfectly elastic collision

	Eigen::Vector2d position, velocity; //center of mass

	class texture* p_texture; //using pointer instead of class to decrease texture duplicates if for example there are many prims using identical textures

	prim(double radius, double mass, double bounciness, Eigen::Vector2d position = Eigen::Vector2d(0,0), Eigen::Vector2d velocity = Eigen::Vector2d(0,0), enum type type = type::interior, texture* p_texture = nullptr);  
	prim();
	~prim(); //does not delete class texture* if on heap.

	void render();
};

/* linearly interpolate primitive trajectories to handle rotation collision checking
struct body //rigid body
{
	double mass, angular_mass;
	double angle, angular_velocity;
	Eigen::Vector2d position, velocity;
	std::vector<prim> prim_list;

	body();
	~body();

	void render();
}
*/
#endif
