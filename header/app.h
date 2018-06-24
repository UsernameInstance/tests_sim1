#ifndef APP_H
#define APP_H
#include <SDL.h>
#include "texture.h"
#include "physics.h"

class app 
{
	bool running = true;

	SDL_Window* window = NULL;
	SDL_Surface* surface = NULL;
	class texture background_ball_texture;
	class texture foreground_ball_texture;
	class texture foreground_ball_2_texture;

	Uint32 last_ticks;
	Uint32 current_ticks;
	std::vector<prim> prim_list;

	app();
	
	//handle SDL events
	void event(SDL_Event* e);

	//initialize SDL etc...
	bool init();

	//logic loop
	void loop();

	//render loop
	void render();

	//free up resources
	void close();

	int time_step_ticks;
	double time_step; 
	int step_limit;
	double tolerance;
	double resting_tolerance;
	int method;
	bool reset_sap; //if true reset sweep and prune boxes on loop

	public:
	static const int width = 900;
	static const int height = 900;

	static app instance; //static members zero initialized

	SDL_Renderer* renderer = NULL;

	void set_time_step(double time_step);

	int execute(int argc, char* argv[]);
};

#endif
