//wrapper for SDL_Texture

#ifndef TEXTURE_H
#define TEXTURE_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <string>
#include <cmath>

class texture
{
	std::string filename;
	SDL_Renderer* renderer = nullptr;
	SDL_Texture* sdltexture = nullptr;
	int width = 0;
	int height = 0;

	void set_renderer(SDL_Renderer* renderer); 

	public:
	texture();
	~texture();

	int get_width();
	int get_height();
	
	//loads image at specified filename for use with specified renderer. if nullptr then current renderer used.
	bool load(std::string filename, SDL_Renderer* renderer = nullptr);

	//create circle texture. if success then destroys current texture.
	bool make_circle(int radius, Uint32 color, bool fill = true, SDL_Renderer* renderer = nullptr); 

	//deallocates texture
	void free();

	void render(int x, int y);
	void render(int x, int y, int width, int height);
	void render(int x, int y, int width, int height, int sx, int sy, int swidth, int sheight);
};

#endif
