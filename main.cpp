#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include "SDL2/SDL.h"
#include "SDL2/SDL2_gfxPrimitives.h"

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 600

#define MAX_BOIDS 500

#define SEPARATION_DIST \
  4  // distance away to look for neighbors for separation force
#define VISIBLE_DIST \
  30  // distance away to lok for neighbors for alignment + cohesion
#define SEPARATION_FORCE 0.07  // how strongly to repel away from neighbors
#define ALIGNMENT_FORCE 0.05
#define COHESION_FORCE 0.003

#define GRID_COLS (WINDOW_WIDTH / VISIBLE_DIST + 1)
#define GRID_ROWS (WINDOW_HEIGHT / VISIBLE_DIST + 1)

#define MAX_VELOCITY 3.0f
#define MIN_VELOCITY 2.0f

#define TURNING_FACTOR 0.1
// Turn near edge of screen
#define SCREEN_PADDING 100
#define RIGHT_MARGIN (WINDOW_WIDTH - SCREEN_PADDING)
#define LEFT_MARGIN SCREEN_PADDING
#define TOP_MARGIN SCREEN_PADDING
#define BOTTOM_MARGIN (WINDOW_HEIGHT - SCREEN_PADDING)

#define FOV_THETA 90
#define TO_DEGREES (180 / M_PI)

class Boid {
 public:
  int x, y;
  float dx, dy;
  Boid(int x, int y, int dx, int dy) : x(x), y(y), dx(dx), dy(dy) {}

  void move(const std::vector<std::unique_ptr<Boid>> &neighboring_boids) {
    // Separation vars
    float sep_dx = 0;
    float sep_dy = 0;

    // Neighbor counts (protected + visible ranges)
    int neighbors = 0;

    // Alignment vars (average velocity vector)
    float xvel_avg = 0;
    float yvel_avg = 0;

    // Cohesion vars (average position vector)
    float xpos_avg = 0;
    float ypos_avg = 0;

    for (const auto &other : neighboring_boids) {
      if (other.get() != this) {
        int dx = x - other->x;
        int dy = y - other->y;
        int dist_sq = dx * dx + dy * dy;

        if (dist_sq <
            SEPARATION_DIST * SEPARATION_DIST) {  // Check against square since
                                                  // it's faster than sq root
          // add separation force (stronger when the neighbor is closer)
          // float factor = 1.0f / (dist_sq + 1);  // avoid division by zero
          sep_dx += dx;
          sep_dy += dy;
        }

        // Other boid is in visible range, calculate alignment + cohesion
        if (dist_sq < VISIBLE_DIST * VISIBLE_DIST) {
          // Alignment
          xvel_avg += other->dx;
          yvel_avg += other->dy;

          // Cohesion
          xpos_avg += other->x;
          ypos_avg += other->y;
          neighbors += 1;
        }
      }
    }

    // Apply separation force
    dx += sep_dx * SEPARATION_FORCE;
    dy += sep_dy * SEPARATION_FORCE;

    // Apply cohesion and alignment forces
    if (neighbors > 0) {
      // Finish calculation average velocity and position of neighbors
      xvel_avg /= neighbors;
      yvel_avg /= neighbors;
      xpos_avg /= neighbors;
      ypos_avg /= neighbors;

      // Apply alignment force
      dx += (xvel_avg - dx) * ALIGNMENT_FORCE;
      dy += (yvel_avg - dy) * ALIGNMENT_FORCE;

      // Apply cohesion force
      dx += (xpos_avg - x) * COHESION_FORCE;
      dy += (ypos_avg - y) * COHESION_FORCE;
    }

    // Apply turning
    if (x < LEFT_MARGIN) {
      dx += TURNING_FACTOR;
    }
    if (x > RIGHT_MARGIN) {
      dx -= TURNING_FACTOR;
    }
    if (y < TOP_MARGIN) {
      dy += TURNING_FACTOR;
    }
    if (y > BOTTOM_MARGIN) {
      dy -= TURNING_FACTOR;
    }

    // Limit maximum velocity
    float speed = sqrt(dx * dx + dy * dy);
    if (speed > MAX_VELOCITY) {
      float scale = MAX_VELOCITY / speed;
      dx *= scale;
      dy *= scale;
    } else if (speed < MIN_VELOCITY) {
      // If moving too slow, scale up to minimum velocity
      float scale = MIN_VELOCITY / speed;
      dx *= scale;
      dy *= scale;
    }

    // Update boid's position
    x += dx;
    y += dy;
  }
};

class Game {
 private:
  SDL_Renderer *renderer;
  SDL_Window *window;
  std::vector<std::unique_ptr<Boid>> boids;
  std::vector<Boid *> boid_position_grid[GRID_ROWS][GRID_COLS];

  // FPS
  Uint32 lastTime;
  float fps;

  void updateGrid() {
    for (int i = 0; i < GRID_COLS; i++) {
      for (int j = 0; j < GRID_ROWS; j++) {
        boid_position_grid[i][j].clear();
      }
    }

    // Place boids in grid cells
    for (const auto &boid : boids) {
      int grid_x = boid->x / VISIBLE_DIST;
      int grid_y = boid->y / VISIBLE_DIST;
      if (grid_x >= 0 && grid_x < GRID_COLS && grid_y >= 0 &&
          grid_y < GRID_ROWS) {
        boid_position_grid[grid_y][grid_x].push_back(boid.get());
      }
    }
  }

  std::vector<Boid *> get_neighboring_boids(Boid &boid) {
    std::vector<Boid *> neighbors;
    // Return the boids within adjacent cells in grid
    int grid_x = boid.x / VISIBLE_DIST;
    int grid_y = boid.y / VISIBLE_DIST;

    // Check surrounding cells
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        int check_x = grid_x + i;
        int check_y = grid_y + j;

        if (check_x >= 0 && check_x < GRID_COLS && check_y >= 0 &&
            check_y < GRID_ROWS) {
          for (Boid *neighbor : boid_position_grid[check_y][check_x]) {
            neighbors.push_back(neighbor);
          }
        }
      }
    }

    return neighbors;
  }

  void render_boids() {
    // updateGrid();
    for (const auto &boid : boids) {
      // auto neighboring_boids = get_neighboring_boids(*boid);
      boid->move(boids);

      // Draw a rectangle for each boid
      SDL_Rect rect = {boid->x, boid->y, 4, 4};
      SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
      SDL_RenderFillRect(renderer, &rect);
    }
  }

 public:
  Game() : lastTime(SDL_GetTicks()), fps(0.0f) {
    // Create random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> x_dist(0, WINDOW_WIDTH);
    std::uniform_int_distribution<> y_dist(0, WINDOW_HEIGHT);

    std::uniform_int_distribution<> vel_dist(MIN_VELOCITY, MAX_VELOCITY);
    std::uniform_int_distribution<> sign_dist(0, 1);

    // Initialize boids to random position
    for (int i = 0; i < MAX_BOIDS; i++) {
      int x = x_dist(gen);
      int y = y_dist(gen);
      int dx = vel_dist(gen) * (sign_dist(gen) ? 1 : -1);
      int dy = vel_dist(gen) * (sign_dist(gen) ? 1 : -1);

      boids.push_back(std::make_unique<Boid>(x, y, dx, dy));
    }
  };

  bool init() {
    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
      std::cout << "SDL could not initialize! SDL_Error: " << SDL_GetError()
                << std::endl;
      return false;
    }

    // Create window
    window = SDL_CreateWindow("SDL Window", SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH,
                              WINDOW_HEIGHT, SDL_WINDOW_SHOWN);

    if (window == nullptr) {
      std::cout << "Window could not be created! SDL_Error: " << SDL_GetError()
                << std::endl;
      return false;
    }

    // Create renderer
    renderer = SDL_CreateRenderer(
        window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (renderer == nullptr) {
      std::cout << "Renderer could not be created! SDL_Error: "
                << SDL_GetError() << std::endl;
      return false;
    }

    return true;
  };

  void run() {
    // Main loop flag
    bool quit = false;

    // Event handler
    SDL_Event e;

    // While application is running
    while (!quit) {
      // Handle events on queue
      while (SDL_PollEvent(&e) != 0) {
        // User requests quit
        if (e.type == SDL_QUIT) {
          quit = true;
        }
      }

      // Clear screen
      SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
      SDL_RenderClear(renderer);

      render_boids();

      // Calculate FPS
      Uint32 currentTime = SDL_GetTicks();
      float deltaTime = (currentTime - lastTime) / 1000.0f;
      fps = 1.0f / deltaTime;
      lastTime = currentTime;

      // Render FPS
      char fps_text[32];
      snprintf(fps_text, sizeof(fps_text), "FPS: %.1f", fps);
      stringRGBA(renderer, WINDOW_WIDTH - 100, 10, fps_text, 255, 255, 255,
                 255);

      // Update screen
      SDL_RenderPresent(renderer);
    }
  }

  void close() {
    // Free resources and close SDL
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
  }
};

int main(int argc, char *argv[]) {
  Game game;
  if (!game.init()) {
    return 1;
  }

  game.run();
  game.close();

  return 0;
}