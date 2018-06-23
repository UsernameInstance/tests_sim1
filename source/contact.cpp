#include "contact.h"
#include <iostream>

contact::contact(prim* p_prim_0, prim* p_prim_1, double time, bool is_collision, Eigen::Vector2d normal)
	: p_prim_0(p_prim_0), p_prim_1(p_prim_1), time(time), is_collision(is_collision), normal(normal) {};

contact::~contact() {};
//time of contact can equal time_limit
contact get_contact(prim & prim0, prim & prim1, double time_limit, bool collision_only, double tolerance)
{
	contact output(&prim0, &prim1);
	if(time_limit < 0)
		return output;

	double diff_rad_squared = std::pow(prim1.radius-prim0.radius, 2) - tolerance;
	double sum_rad_squared = std::pow(prim1.radius+prim0.radius, 2) + tolerance;
	Eigen::Vector2d rel_vel = (prim1.velocity-prim0.velocity);
	Eigen::Vector2d rel_pos = (prim1.position-prim0.position);
	double rel_vel_squared = rel_vel.squaredNorm();
	double rel_pos_squared = rel_pos.squaredNorm();
	double rel_vel_dot_rel_pos = rel_vel.dot(rel_pos);
	double rel_vel_cross_rel_pos_squared = rel_vel_squared*rel_pos_squared - std::pow(rel_vel_dot_rel_pos, 2);

	//if moving together and in range dist<=sum_rad then collision pushing away
	if( (prim0.type == prim::type::interior) && (prim1.type == prim::type::interior) )
	{
		if(rel_vel_squared == 0 || time_limit == 0)
		{
			output.normal = rel_pos.normalized();
			if((collision_only && rel_vel.dot(output.normal) >= 0) || (rel_pos_squared > sum_rad_squared))
				return output;

			output.time = 0;
			if(rel_vel.dot(output.normal) < 0)
				output.is_collision = true;

			return output;
		}

		double disc = rel_vel_squared*sum_rad_squared - rel_vel_cross_rel_pos_squared;

		if(rel_vel_squared*sum_rad_squared < rel_vel_cross_rel_pos_squared)
			return output;

		double upper_bound, lower_bound;
		double temp = std::sqrt(disc);
		upper_bound = (temp-rel_vel_dot_rel_pos)/rel_vel_squared;
		lower_bound = std::fmax(0, -temp-rel_vel_dot_rel_pos)/rel_vel_squared;

		if(temp-rel_vel_dot_rel_pos < std::fmax(0, (-temp-rel_vel_dot_rel_pos)) || time_limit*rel_vel_squared < std::fmax(0, (-temp-rel_vel_dot_rel_pos)))
			return output;

		if(collision_only)
		{
			if(-rel_vel_dot_rel_pos <= std::fmax(0, (-temp-rel_vel_dot_rel_pos)))
				return output;
		}

		output.time = lower_bound;
		output.normal = (rel_vel*lower_bound + rel_pos).normalized();
		if(rel_vel.dot(output.normal) < 0)
			output.is_collision = true;

		return output;
	}

	//if moving away and in range dist>=diff_rad then collision pushing together 
	else if((prim0.type == prim::type::exterior) || (prim1.type == prim::type::exterior))
	{
		if(rel_vel_squared == 0 || time_limit == 0)
		{
			output.normal = -rel_pos.normalized(); 
			if((collision_only && rel_vel.dot(output.normal) >= 0) || rel_pos_squared < diff_rad_squared)
				return output;

			output.time = 0;
			if(rel_vel.dot(output.normal) < 0)
				output.is_collision = true;

			return output;
		}

		double upper_bound, lower_bound; //disjunction for this one, i.e. lower_bound <= time OR time <= upper_bound, where time is time of contact.
		double temp = std::sqrt(std::fmax(0, rel_vel_squared*diff_rad_squared - rel_vel_cross_rel_pos_squared));
		upper_bound = (-temp-rel_vel_dot_rel_pos)/rel_vel_squared;
		lower_bound = std::fmax(0, (temp-rel_vel_dot_rel_pos)/rel_vel_squared);
		

		//case: time <= upper_bound
		if(upper_bound >= 0) //since time >= 0
		{
			if(collision_only) 
			{
				if(0 > -rel_vel_dot_rel_pos) //negative because type::exterior 
				{
					output.is_collision = true;
					output.time = 0;
					output.normal = -rel_pos.normalized(); //if rel_pos is 0 vector, just let the next physics loop handle it.

					return output; //cant do any better than output.time == 0;
				}
			}
			else
			{
				if(0 > -rel_vel_dot_rel_pos)
					output.is_collision = true;

				output.time = 0;
				output.normal = -rel_pos.normalized();

				return output;
			}
		}

		//case: lower_bound <= time
		if(lower_bound <= time_limit)
		{
			if(collision_only)
			{
				if(std::fmax(0, (temp-rel_vel_dot_rel_pos)) > -rel_vel_dot_rel_pos) //inequality backwards because type::exterior
				{
					output.is_collision = true;
					output.time = lower_bound;
					output.normal = -(rel_vel*lower_bound + rel_pos).normalized();
					
					return output;
				}
			}
			else
			{
				if(std::fmax(0, (temp-rel_vel_dot_rel_pos)) > -rel_vel_dot_rel_pos)
					output.is_collision = true;
					
				output.time = lower_bound;
				output.normal = -(rel_vel*lower_bound + rel_pos).normalized();

				return output;
			}
		}
	}

	return output;
}
