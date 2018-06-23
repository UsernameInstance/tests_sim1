#include "texture.h"
#include "log.h"

texture::texture() {}

texture::~texture()
{
	free();
}

int texture::get_width() { return width; }
int texture::get_height() { return height; }
void texture::set_renderer(SDL_Renderer* renderer) { this->renderer = renderer; } //make sure this doesn't break existing texture before calling

bool texture::load(std::string filename, SDL_Renderer* renderer)
{
	if(renderer == NULL && this->renderer == NULL)
	{
		LOG("Error in bool load(std::string,SDL_Renderer): No renderer supplied or available.");
		return false;
	}
	
	if(renderer)
	{
		this->renderer = renderer;
	}
	this->filename = filename;

	SDL_Surface* tempsurface = IMG_Load(filename.c_str());
	if(tempsurface == NULL)
	{
		LOG("Unable to laod image : %s : %s", filename.c_str(), IMG_GetError());
		return false;
	}

	//convert SDL surface to texture
	if((sdltexture = SDL_CreateTextureFromSurface(renderer, tempsurface)) == NULL)
	{
		LOG("Unable to create SDL Texture : %s : %s", filename.c_str(), IMG_GetError());
		return false;
	}

	//get dimensions
	SDL_QueryTexture(sdltexture, NULL, NULL, &width, &height); 

	SDL_FreeSurface(tempsurface);

	return true;
}

//pixel format RGBA
bool texture::make_circle(int radius, Uint32 color, bool fill, SDL_Renderer* renderer)
{
	if(radius<0)
	{
		LOG("error in bool make_circle(int, Uint32, bool): radius<0.");
		return false;
	}
	if(!renderer && !(this->renderer))
	{
		LOG("error in bool make_circle(int, Uint32, bool): No renderer supplied or available.");
		return false;
	}
	if(renderer == nullptr)
		renderer = this->renderer;

	int width = 2*radius+1;
	int height = width;
	SDL_Surface * temp_surface;
	Uint32 rmask, gmask, bmask, amask; /* SDL interprets each pixel as a 32-bit number, so our masks must depend on the endianness (byte order) of the machine */
	#if SDL_BYTEORDER == SDL_BIG_ENDIAN
	rmask = 0xff000000;
	gmask = 0x00ff0000;
	bmask = 0x0000ff00;
	amask = 0x000000ff;
	#else
	rmask = 0x000000ff;
	gmask = 0x0000ff00;
	bmask = 0x00ff0000;
	amask = 0xff000000;
	color = ( (color << 24) | ( (color << 8) & 0x00ff0000 ) | ( (color >> 8) & 0x0000ff00 ) | (color >> 24) );
	#endif

	temp_surface = SDL_CreateRGBSurface(SDL_SWSURFACE, width, height, 32, rmask, gmask, bmask, amask);
	if(!temp_surface)
	{
		LOG("unable to create surface: %s", SDL_GetError());
		return false;
	}
	Uint32* pixels = (Uint32*)temp_surface->pixels;	
	int r2 = radius*radius;
	int h;
	int j;
	if(fill)
	{
		for(int i=-radius; i<=radius; ++i)
		{
			j = std::sqrt(r2-i*i);
			for(int h = -j; h<=j; ++h)
				pixels[((i+radius)*width)+h+radius] = color; 
		}
	}
	else
	{
		for(int i=-radius; i<=radius; ++i)
		{
			j = std::sqrt(r2-i*i);
			pixels[((i+radius)*width)-j+radius] = color;
			pixels[((i+radius)*width)+j+radius] = color;
		}
	}
	//Create texture from surface pixels
	SDL_Texture* temp_texture = SDL_CreateTextureFromSurface(renderer, temp_surface);
	if(!temp_texture)
	{
		LOG("Error in bool make_circle(int,Uint32,bool): %s", SDL_GetError());
		//delete circle surface
		SDL_FreeSurface( temp_surface );
		return false;
	}

	free();
	filename = "";

	this->renderer = renderer;
	sdltexture = temp_texture;
	this->width = width;
	this->height = height;

	return true;
}

void texture::free()
{
	if(sdltexture)
	{
		SDL_DestroyTexture(sdltexture);
		sdltexture = NULL;
		width = 0;
		height = 0;
	}
}

void texture::render(int x, int y)
{
	render(x,y, width, height);
}

void texture::render(int x, int y, int width, int height)
{
	SDL_Rect destination = {x,y,width,height};
	SDL_RenderCopy(renderer, sdltexture, NULL, &destination);
}

void texture::render(int x, int y, int width, int height, int sx, int sy, int swidth, int sheight)
{
	SDL_Rect source = {sx, sy, swidth, sheight};
	SDL_Rect destination = {x, y, width, height};
	SDL_RenderCopy(renderer, sdltexture, &source, &destination);
}
