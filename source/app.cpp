#include "app.h"
#include "log.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>

app app::instance;

app::app() {}

void app::event(SDL_Event* e) {}

bool app::init()
{
	time_step_ticks = 8;
	time_step = .008; 
	step_limit = 256;
	tolerance = 0x1p-8;
	resting_tolerance = 0x1p-4;
	method = 0;
	reset_sap = true;

	if(SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		LOG("Unable to init SDL: %s", SDL_GetError());
		return false;
	}

	if(!SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1"))
	{
		LOG("Unable to init hinting: %s", SDL_GetError());
	}
	
	SDL_Rect display_bounds;
	if(SDL_GetDisplayBounds(0, &display_bounds) != 0)
	{
		LOG("SDL_GetDisplayBounds failed: %s", SDL_GetError());
		return false;
	}

	if((window = SDL_CreateWindow( "Window Name", (display_bounds.w)/2-100, 70, width, height, SDL_WINDOW_SHOWN)) == NULL)
	{
		LOG("Unable to create SDL Window: %s", SDL_GetError());
		return false;
	}

	surface = SDL_GetWindowSurface(window);

	if((renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED)) == NULL)
	{
		LOG("Unable to create renderer");
		return false;
	}

	SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0xFF);

	return true;
}

void app::loop()
{
	current_ticks = SDL_GetTicks();
	if(current_ticks >= time_step_ticks + last_ticks)
	{
		last_ticks = current_ticks;
		step_physics(prim_list, time_step, step_limit, tolerance, resting_tolerance, method, reset_sap); 
	}
}

void app::render()
{
	SDL_RenderClear(renderer);

	for(size_t i=0; i< prim_list.size(); ++i)
		prim_list.at(i).render();
	
	SDL_RenderPresent(renderer);
}

void app::close()
{
	if(renderer)
	{
		SDL_DestroyRenderer(renderer);
		renderer = NULL;
	}

	if(window)
	{
		SDL_DestroyWindow(window);
		window = NULL;
	}

	SDL_Quit();
}

void app::set_time_step(double time_step)
{
	if(time_step <= 0)
		return;
	this->time_step = time_step;
	time_step_ticks = 1000*time_step;
}

//full of miscellaneous tests atm.
int app::execute(int argc, char* argv[])
{
	if(!init()) return 0;

	last_ticks = SDL_GetTicks();
	current_ticks = last_ticks;
	
	double background_ball_radius = height/2.0;
	double foreground_ball_radius = 28;
	double x0 = width/2.0; //initial position of cue ball
	double y0 = height/2.0; //initial position of cue ball
	Eigen::Vector2d v0 = Eigen::Vector2d(256,0); //initial velocity of cue ball
	double bounce = 1; //material property corresponds to COR value
	double pi = 3.14159265358979323846;
	double foremass = (4.0/3.0)*pi*foreground_ball_radius*foreground_ball_radius*foreground_ball_radius; //mass of foreground balls
	int mod_num = 1; //higher values give random initial velocities to balls in tests 3 and 4
	double mult = 1; //scales large ball in tests 3 and 4. Don't change this line.
	std::srand(std::time(NULL)); //used with mod_num
	double eps = 0; //minimum separation of balls in initial configuration in test 2 pool break (0 is touching).
	int L0 = 6; //number of balls along width of rectangle of balls in tests 3 and 4.
	int L1 = 6; //number of balls along height of rectangle of balls in tests 3 and 4.

	bool show_parameters = false;
	std::cout << "\n***************************************************************\n";
	std::cout << "method == " << method << "\n"; 
	std::cout << "tolerance == " << tolerance << "\n"; 
	std::cout << "resting_tolerance == " << resting_tolerance << "\n"; 
	std::cout << "COR == " << bounce << "\n";
	std::cout << "***************************************************************" << std::endl;

	SDL_Event Event;
	while(running)
	{
		while(SDL_PollEvent(&Event) != 0)
		{
			event(&Event);
			if(Event.type == SDL_QUIT) running = false;
			if(Event.type == SDL_KEYUP)
			{
				switch(Event.key.keysym.sym)
				{
					case SDLK_UP:
						tolerance += .05;
						show_parameters = true;
						break;

					case SDLK_DOWN:
						tolerance += -.05;
						if(tolerance < 0) tolerance = 0;
						show_parameters = true;
						break;

					case SDLK_RIGHT:
						resting_tolerance += .05;
						show_parameters = true;
						break;

					case SDLK_LEFT:
						resting_tolerance += -.05;
						if(resting_tolerance < 0) resting_tolerance = 0;
						show_parameters = true;
						break;

					case SDLK_m:
						method = (1 + method)%2;
						show_parameters = true;
						break;

					case SDLK_t:
						if(tolerance == 0)
							tolerance = 1;
						else
							tolerance = 0;
						show_parameters = true;
						break;

					case SDLK_r:
						if(resting_tolerance == 0)
							resting_tolerance = 1;
						else
							resting_tolerance = 0;
						show_parameters = true;
						break;

					case SDLK_a:
						tolerance = 1;
						resting_tolerance = 1;
						show_parameters = true;
						break;
						
					case SDLK_z:
						tolerance = 0;
						resting_tolerance = 0;
						show_parameters = true;
						break;

					case SDLK_b:
						bounce += .05;
						if(bounce > 1) bounce = 1;
						show_parameters = true;
						break;

					case SDLK_n:
						bounce += -.05;
						if(bounce < 0) bounce = 0;
						show_parameters = true;
						break;

					case SDLK_ESCAPE:
						prim_list.clear();
						for(int i=0; i<57; ++i)
							std::cout << "\n";
						std::cout << std::endl;
						reset_sap = true;
						break;

					case SDLK_1: //test 1
						prim_list.clear();
						background_ball_radius = height/2.0;
						foreground_ball_radius = 32;
						v0 = Eigen::Vector2d(512,0);
						mult = 1;
						reset_sap = true;

						background_ball_texture.make_circle(background_ball_radius, 0x00ffffFF, true, renderer);
						foreground_ball_texture.make_circle(foreground_ball_radius, 0xff00ffFF, true, renderer);
						prim_list.push_back(prim(background_ball_radius, std::numeric_limits<double>::max(), bounce, Eigen::Vector2d(width/2.0, height/2.0), Eigen::Vector2d(0,0), prim::type::exterior, &background_ball_texture));

						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(foreground_ball_radius, y0), v0, prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0-2*foreground_ball_radius, y0), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0, y0), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+2*foreground_ball_radius, y0), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+4*foreground_ball_radius, y0), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						break;

					case SDLK_2: //test 2
						prim_list.clear();
						background_ball_radius = height/2.0;
						foreground_ball_radius = 27;
						v0 = Eigen::Vector2d(256,0);
						mult = 1;
						reset_sap = true;

						background_ball_texture.make_circle(background_ball_radius, 0x00ffffFF, true, renderer);
						foreground_ball_texture.make_circle(foreground_ball_radius, 0xff00ffFF, true, renderer); //0x8f8a88FF
						prim_list.push_back(prim(background_ball_radius, std::numeric_limits<double>::max(), bounce, Eigen::Vector2d(width/2.0, height/2.0), Eigen::Vector2d(0,0), prim::type::exterior, &background_ball_texture));

						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(foreground_ball_radius, y0), v0, prim::type::interior, &foreground_ball_texture));

						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0, y0), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));


						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+std::sqrt(3)*foreground_ball_radius, y0-foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+std::sqrt(3)*foreground_ball_radius, y0+foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));

						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+2*std::sqrt(3)*foreground_ball_radius, y0-2*foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+2*std::sqrt(3)*foreground_ball_radius, y0), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+2*std::sqrt(3)*foreground_ball_radius, y0+2*foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));

						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+3*std::sqrt(3)*foreground_ball_radius, y0-3*foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+3*std::sqrt(3)*foreground_ball_radius, y0-1*foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+3*std::sqrt(3)*foreground_ball_radius, y0+1*foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+3*std::sqrt(3)*foreground_ball_radius, y0+3*foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));

						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+4*std::sqrt(3)*foreground_ball_radius, y0-4*foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+4*std::sqrt(3)*foreground_ball_radius, y0-2*foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+4*std::sqrt(3)*foreground_ball_radius, y0), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+4*std::sqrt(3)*foreground_ball_radius, y0+2*foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(x0+4*std::sqrt(3)*foreground_ball_radius, y0+4*foreground_ball_radius), Eigen::Vector2d(0,0), prim::type::interior, &foreground_ball_texture));
						break;

					case SDLK_3: //test 3
						prim_list.clear();
						background_ball_radius = height/2.0;
						foreground_ball_radius = 16;
						v0 = Eigen::Vector2d(0,-256);
						mult = 2;
						reset_sap = true;

						L0 = 6;
						L1 = 6;

						background_ball_texture.make_circle(background_ball_radius, 0x00ffffFF, true, renderer);
						foreground_ball_texture.make_circle(foreground_ball_radius, 0xff00ffFF, true, renderer);
						foreground_ball_2_texture.make_circle(mult*foreground_ball_radius, 0xff00ffFF, true, renderer);

						prim_list.push_back(prim(background_ball_radius, std::numeric_limits<double>::max(), bounce, Eigen::Vector2d(width/2.0, height/2.0), Eigen::Vector2d(0,0), prim::type::exterior, &background_ball_texture));

						prim_list.push_back(prim(mult*foreground_ball_radius, mult*mult*mult*foremass, bounce, Eigen::Vector2d(x0, y0+500), v0, prim::type::interior, &foreground_ball_2_texture));
						for(int i = 0; i<L0; ++i)
						{
							for(int j=0; j<L1; ++j)
							{
								prim_list.push_back(prim(foreground_ball_radius, foremass, 1*bounce, Eigen::Vector2d(width/2.0 + (-L0+1+i*2)*(foreground_ball_radius - eps), height/2.0 + (-L1+1+j*2)*(foreground_ball_radius - eps)), Eigen::Vector2d((rand()%mod_num)-(rand()%mod_num),(rand()%mod_num)-(rand()%mod_num)), prim::type::interior, &foreground_ball_texture));
							}
						}
						break;

					case SDLK_4: //test 4
						prim_list.clear();
						background_ball_radius = height/2.0;
						foreground_ball_radius = 4;
						v0 = Eigen::Vector2d(0,-256);
						mult = 4;
						reset_sap = true;

						L0 = 16;
						L1 = 16;

						background_ball_texture.make_circle(background_ball_radius, 0x00ffffFF, true, renderer);
						foreground_ball_texture.make_circle(foreground_ball_radius, 0xff00ffFF, true, renderer);
						foreground_ball_2_texture.make_circle(mult*foreground_ball_radius, 0xff00ffFF, true, renderer);

						prim_list.push_back(prim(background_ball_radius, std::numeric_limits<double>::max(), bounce, Eigen::Vector2d(width/2.0, height/2.0), Eigen::Vector2d(0,0), prim::type::exterior, &background_ball_texture));

						prim_list.push_back(prim(mult*foreground_ball_radius, mult*mult*mult*foremass, bounce, Eigen::Vector2d(x0, y0+500), v0, prim::type::interior, &foreground_ball_2_texture));
						for(int i = 0; i<L0; ++i)
						{
							for(int j=0; j<L1; ++j)
							{
								prim_list.push_back(prim(foreground_ball_radius, foremass, 1*bounce, Eigen::Vector2d(width/2.0 + (-L0+1+i*2)*(foreground_ball_radius - eps), height/2.0 + (-L1+1+j*2)*(foreground_ball_radius - eps)), Eigen::Vector2d((rand()%mod_num)-(rand()%mod_num),(rand()%mod_num)-(rand()%mod_num)), prim::type::interior, &foreground_ball_texture));
							}
						}
						break;

					case SDLK_5: //test 5
						prim_list.clear();
						foreground_ball_radius = 27;
						background_ball_radius = height/30.0;
						mult = 1;
						v0 = Eigen::Vector2d(0,-256);
						reset_sap = true;

						background_ball_texture.make_circle(background_ball_radius, 0x00ffffFF, true, renderer);
						foreground_ball_texture.make_circle(foreground_ball_radius, 0xff00ffFF, true, renderer);

						prim_list.push_back(prim(background_ball_radius, std::numeric_limits<double>::max(), bounce, Eigen::Vector2d(width/2.0, height/2.0), Eigen::Vector2d(0,0), prim::type::exterior, &background_ball_texture));

						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(2*foreground_ball_radius, y0), v0, prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(width-2*foreground_ball_radius, y0), Eigen::Vector2d(0, 0), prim::type::interior, &foreground_ball_texture));
						prim_list.push_back(prim(foreground_ball_radius, foremass, bounce, Eigen::Vector2d(width-6*foreground_ball_radius, y0), Eigen::Vector2d(200, 300), prim::type::interior, &foreground_ball_texture));
						break;

					default:
						break;
				}
			}
		}

		loop();
		render();
		reset_sap = false;
		if(show_parameters)
		{
			std::cout << "\n***************************************************************\n";
			std::cout << "method == " << method << "\n"; 
			std::cout << "tolerance == " << tolerance << "\n"; 
			std::cout << "resting_tolerance == " << resting_tolerance << "\n"; 
			std::cout << "COR == " << bounce << "\n";
			std::cout << "***************************************************************" << std::endl;
		}
		show_parameters = false;
	}

	close();

	return 1;
}
