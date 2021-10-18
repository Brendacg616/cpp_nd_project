#ifndef RENDERER_H
#define RENDERER_H

#include <vector>
#include "SDL.h"
#include "SDL_ttf.h"
#include <iostream>
#include <string>
#include "robot.h"
#include <deque>

using namespace std;
class Renderer {
 public:
  Renderer(const std::size_t sim_width, const std::size_t info_width,
    const std::size_t screen_height, string pkg_path);
  ~Renderer();

  void Render(const Robot &robot, const std::deque<Position> *route);
  void UpdateWindowTitle(int fps);
/*
  void UpdateWindowTitle(int score, int fps);
*/
 private:
  SDL_Window *sdl_window;
  SDL_Renderer *sdl_renderer;
  // SDL objects for text rendering
  SDL_Surface *sdl_text_surface;
  SDL_Texture *sdl_text_texture;
  TTF_Font *font;
  const std::size_t sim_width ;
  const std::size_t info_width ;
  const std::size_t screen_height;
  string package_path;

  void RenderSpeedDisplay(Velocity vel, Position position);
  void RenderText(const string str,const SDL_Point  text_position,const SDL_Color color);
  void RenderOdometryDisplay(const std::deque<Position> *route);
};

#endif
