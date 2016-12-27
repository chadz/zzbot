#pragma once

#include "doodads.h"
#include "hlt.hpp"
#include "networking.hpp"

#include <array>
#include <functional>
#include <map>
#include <unordered_map>

typedef unsigned char direction_t;
typedef unsigned short distance_t;

// i feel bad, and so should you for reading this
#define LOGZ                                                                                                           \
    if (config_.should_log) log_

struct site_state {
    float score{};
    int potential{};
    bool visited = false;
    bool border = false;
};

struct player_state {
    unsigned int strength{};
    unsigned int production{};
    unsigned int population{};
};

struct game_state {
    int frame{};
    unsigned int population{};
    std::map<int, player_state> players;
};

struct site_info {
    const hlt::Location& loc;
    const hlt::Site& site;
    site_state& state;
};

struct zzbot_config {
    std::string name = "zzbot";
    unsigned short production_move_scalar = 5;
    unsigned short score_region_radius = 10;
    float score_enemy_scalar = 0.55f;
    int wander_clobber_ceiling = 375;
    unsigned short max_wait_for_attack = 1;
    bool should_log = false;

    int max_reinforce_depth = 5;
    int min_reinforce_depth = 3;
};

class zzbot
{
  private:
    zzbot_config config_{};
    unsigned char id_{};

    std::fstream log_;
    hlt::GameMap map_;

    std::map<hlt::Location, hlt::Move> orders_;
    std::set<hlt::Location> mine_;
    std::vector<hlt::Location> enemies_;

    std::vector<std::vector<site_state>> state_;

    game_state game_state_;

  public:
    zzbot(zzbot_config cfg);
    ~zzbot();

    direction_t reverse_direction(direction_t direction)
    {
        int rdir[] = {STILL, SOUTH, WEST, NORTH, EAST};
        return rdir[direction];
    }

    site_state& get_state(const hlt::Location& loc)
    {
        return state_[loc.y][loc.x];
    }

    site_info get_info(const hlt::Location& location)
    {
        return site_info{location, map_.getSite(location), get_state(location)};
    }

    void mark_visited(const hlt::Location& loc)
    {
        auto& state = get_state(loc);
        state.visited = true;
    }

    bool has_visited(const hlt::Location& loc)
    {
        const auto& state = get_state(loc);
        return state.visited;
    }

    void calc_state();
    void run();
    void behavior();

    direction_t get_angle(const hlt::Location& from, const hlt::Location& to);

    bool should_idle(const hlt::Site& site);
    bool assigned_move(const hlt::Location& loc) const;

    bool assign_move(const hlt::Move& move, bool erase = true);

    void do_attack();
    void do_wander();

    int do_try_reinforce(const hlt::Location& root_target, const hlt::Location& target, int depth_limit, int depth,
                         int power, int production);
    void do_try_attack(hlt::Location target);
    void do_try_retreat(hlt::Location target);

    std::vector<std::pair<direction_t, hlt::Location>> get_neighbors(const hlt::Location& location);

    std::vector<std::pair<direction_t, hlt::Location>> get_neighbors(const hlt::Location& location,
                                                                     std::function<bool(site_info)> fn);

    float score_region(const hlt::Location& loc);

    typedef std::function<void(site_info)> range_fn;
    void nearby_region(const hlt::Location& loc, distance_t radius, range_fn fn);
    void range_all(range_fn fn);
    void range_do(distance_t y_start, distance_t y_end, distance_t x_start, distance_t x_end, range_fn fn);
};