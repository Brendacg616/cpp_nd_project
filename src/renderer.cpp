#include "renderer.h"

Renderer::Renderer(const std::size_t sim_width, const std::size_t info_width,
  const std::size_t screen_height, string pkg_path):
  sim_width(sim_width), info_width(info_width), screen_height(screen_height),
  package_path(pkg_path)
{
  std::cout << "Renderer constructor" << std::endl;
  // Initialize SDL
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::cerr << "SDL could not initialize.\n";
    std::cerr << "SDL_Error: " << SDL_GetError() << "\n";
  }

  //TTF initialization
  if( TTF_Init() < 0)
  {
    std::cerr << "Error initializing SDL_ttf: " << TTF_GetError() << "\n";
  }

  // Create Window
  sdl_window = SDL_CreateWindow("OdometrySimulation", SDL_WINDOWPOS_CENTERED,
                                SDL_WINDOWPOS_CENTERED, (sim_width + info_width),
                                screen_height, SDL_WINDOW_SHOWN);

  if (nullptr == sdl_window) {
    std::cerr << "Window could not be created.\n";
    std::cerr << " SDL_Error: " << SDL_GetError() << "\n";
  }

  // Create renderer
  sdl_renderer = SDL_CreateRenderer(sdl_window, -1, SDL_RENDERER_ACCELERATED);
  if (nullptr == sdl_renderer) {
    std::cerr << "Renderer could not be created.\n";
    std::cerr << "SDL_Error: " << SDL_GetError() << "\n";
  }

  // Load font
  std::string font_path = package_path + "/fonts/NotoSansMono-SemiBold.ttf";
	font = TTF_OpenFont(font_path.c_str(), 20);
	if ( !font ) {
		std::cerr << "Error loading font: " << TTF_GetError() << "\n";
	}
}

Renderer::~Renderer() {
  std::cout << "Renderer destructor" << std::endl;
  SDL_DestroyWindow(sdl_window);
  SDL_Quit();
}

void Renderer::Render(const Robot &robot, const std::deque<Position> *route)
{
  //std::cout << "Renderer::Render function" << std::endl;
  SDL_Rect block;
  //
  SDL_SetRenderDrawColor(sdl_renderer, 0x1E, 0x1E, 0x1E, 0xFF);
  SDL_RenderClear(sdl_renderer);
  // Render Speed and Position Info
  SDL_SetRenderDrawColor(sdl_renderer, 0xFF, 0xFF, 0xFF, 0xFF);
  block.x = 0;
  block.y = 0;
  block.w = info_width;
  block.h = screen_height;
  SDL_RenderFillRect(sdl_renderer, &block);
  Velocity vel;
  Position pos;
  robot.GetVelocity(vel);
  robot.GetPosition(pos);
  RenderSpeedDisplay(vel, pos);
  RenderOdometryDisplay(route);
  // Update Screen
  SDL_RenderPresent(sdl_renderer);
}
void Renderer::RenderSpeedDisplay(Velocity velocity, Position position)
{
  SDL_Color color = {0x00, 0x00, 0x00};
  std::string str = "VELOCITY ";
  SDL_Point text_position = {10, 10};
  RenderText(str,text_position,color);
  str = "Linear: " ;
  text_position = {10, 60};
  RenderText(str,text_position,color);
  str = to_string(velocity.linear) +  " m/s";
  text_position = {10, 110};
  RenderText(str,text_position,color);
  str = "Angular: " ;
  text_position = {10, 170};
  RenderText(str,text_position,color);
  str = to_string(velocity.angular) + " rad/s";
  text_position = {10, 220};
  RenderText(str,text_position,color);

  str = "POSITION ";
  text_position = {10, 300};
  RenderText(str,text_position,color);
  str = "X: " + to_string(position.x);
  text_position = {10, 360};
  RenderText(str,text_position,color);
  str = "Y: " + to_string(position.y);
  text_position = {10, 420};
  RenderText(str,text_position,color);
  str = "Z: " + to_string(position.z);
  text_position = {10, 480};
  RenderText(str,text_position,color);
}

void Renderer::RenderText(const string str, const SDL_Point text_position,
  const SDL_Color color)
{
  sdl_text_surface = TTF_RenderText_Solid( font, str.c_str() , color );
  if ( !sdl_text_surface ) {
	  std::cerr << "Failed to render text: " << TTF_GetError() << std::endl;
  }
  else
  {
    SDL_Rect  text_rect = {text_position.x, text_position.y,
      sdl_text_surface->w,sdl_text_surface->h};
    sdl_text_texture = SDL_CreateTextureFromSurface(sdl_renderer, sdl_text_surface);
    SDL_RenderCopy(sdl_renderer, sdl_text_texture, NULL, &text_rect);
  }
}

void Renderer::RenderOdometryDisplay(const std::deque<Position> *route)
{
  //Render X axis
  SDL_Color color = {0xFF, 0x00, 0x00};
  std::string str = "X";
  SDL_Point text_position = {info_width + sim_width/2 +10, 10};
  RenderText(str, text_position, color);
  SDL_SetRenderDrawColor(sdl_renderer, 0xFF, 0x00, 0x00, SDL_ALPHA_OPAQUE);
  SDL_RenderDrawLine(sdl_renderer, info_width + (sim_width/2), 0,info_width + (sim_width/2) , screen_height);
  //Render Y axis
  color = {0x00, 0xFF, 0x00};
  str = "Y";
  text_position = {info_width + 10, screen_height/2 -50};
  RenderText(str, text_position, color);
  SDL_SetRenderDrawColor(sdl_renderer, 0x00, 0xFF, 0x00, SDL_ALPHA_OPAQUE);
  SDL_RenderDrawLine(sdl_renderer, info_width +1, screen_height/2 ,info_width + sim_width , screen_height/2);
  SDL_SetRenderDrawColor(sdl_renderer, 0xFF, 0xFF, 0x00, SDL_ALPHA_OPAQUE);
  for (auto it = route->begin(); it !=route->end(); ++it)
  {
      int x = (int)(-(it->y * 100) + info_width + sim_width/2);
      int y = (int) (-(it->x *100) + screen_height /2);
      if (SDL_RenderDrawPoint(sdl_renderer, x, y) < 0 )
        cerr << "Failed to render point" << TTF_GetError() << std::endl;
  }

}

void Renderer::UpdateWindowTitle(int fps) {
  std::string title{"Odometry Simulation --  FPS: " + std::to_string(fps)};
  SDL_SetWindowTitle(sdl_window, title.c_str());
}
