#pragma once

#include "hlt.hpp"
#include "networking.hpp"
#include "doodads.h"

#include <array>
#include <functional>
#include <map>
#include <unordered_map>

typedef unsigned char direction_t;
typedef unsigned short distance_t;

struct site_state {
    float score{};
    unsigned int potential{};
};

struct zzbot_config {
	std::string name = "zzbot";
    unsigned short production_move_scalar = 5;
    unsigned short score_region_radius = 3;
	float score_enemy_scalar = 1.1f;
    unsigned short wander_clobber_ceiling = 350;
};

class zzbot {
  private:
    zzbot_config config_{};
    unsigned char id_{};

    std::fstream log_;
    hlt::GameMap map_;

    std::map<hlt::Location, hlt::Move> orders_;
    std::set<hlt::Location> mine_;
    std::vector<hlt::Location> enemies_;
    std::vector<hlt::Location> neutral_;
    std::vector<std::vector<site_state>> state_;

  public:
    zzbot(zzbot_config cfg);
    ~zzbot();

    direction_t reverse_direction(direction_t direction) {
        switch (direction) {
        case NORTH:
            return SOUTH;
        case SOUTH:
            return NORTH;
        case EAST:
            return WEST;
        case WEST:
            return EAST;
        default:
            return STILL;
        };
    }

    void calc_state();
    void run();
    void behavior();

    bool should_idle(const hlt::Site& site);

    void do_attack(std::vector<hlt::Location>& targets);
    optional<hlt::Move> do_wander(const hlt::Location loc, const hlt::Site& site);

    std::vector<std::pair<direction_t, hlt::Site>> get_neighbors(hlt::Location loc);

    typedef std::function<void(const hlt::Location, const hlt::Site&)> range_fn;
    typedef optional<std::pair<hlt::Location, distance_t>> maybe_neighbor_t;

    std::array<maybe_neighbor_t, 5> distance_to_borders(const hlt::Location loc);

    direction_t find_nearest_border(const hlt::Location loc);
    float score_region(const hlt::Location& loc);
    void nearby_region(const hlt::Location& loc, distance_t radius, range_fn fn);
    void range_all(range_fn fn);
    void range_do(distance_t y_start, distance_t y_end, distance_t x_start, distance_t x_end, range_fn);
};