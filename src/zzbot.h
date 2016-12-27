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
    int wander_clobber_ceiling = 350;
    unsigned short max_wait_for_attack = 1;
    bool should_log = true;

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

    void calc_state();
    void run();
    void behavior();

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

    template <class P>
    std::vector<std::pair<direction_t, hlt::Location>> get_neighbors(const hlt::Location& location, P fn)
    {
        auto neighbors = get_neighbors(location);

        if (get_state(location).score > 0) {
            LOGZ << "score  (" << location.x << "," << location.y << ")"
                 << ": " << get_state(location).score << std::endl;
        }

        filter(neighbors,
               [&](std::pair<direction_t, hlt::Location> neighbor) { return fn(get_info(neighbor.second)); });

        return neighbors;
    }

    // gotta be a cleaner way to do this. don't forget the bug in hlt::getAngle either (y needs be negated in atan2)
    direction_t get_angle(const hlt::Location& from, const hlt::Location& to)
    {
        float angle = map_.getAngle(from, to) * 180.0 / 3.14159265;
        float roll = rand() % 90;
        direction_t dir = EAST;

        if (angle > 0.0f && angle <= 90.0f) {
            dir = roll > angle ? EAST : NORTH;

        } else if (angle < 0.0f && angle >= -90.0f) {
            float a = abs(angle);
            dir = roll > a ? EAST : SOUTH;

        } else if (angle < -90.0f && angle >= -180.0f) {
            float a = abs(angle) - 90.0f;
            dir = roll > a ? SOUTH : WEST;

        } else if (angle > 90.0f && angle <= 180.0f) {
            float a = angle - 90.0f;
            dir = roll > a ? NORTH : WEST;
        }

        LOGZ << "getAngle from (" << from.x << "," << from.y << ") to (" << to.x << "," << to.y << ") roll:" << roll
             << " angle:" << angle << " direction:" << (int)dir << std::endl;

        return dir;
    }

    bool should_idle(const hlt::Site& site);
    bool assigned_move(const hlt::Location& loc) const;

    bool assign_move(const hlt::Move& move, bool erase = true)
    {
        auto loc = move.loc;
        const auto& current_site = map_.getSite(loc);

        auto& current_state = get_state(loc);

        auto future_loc = map_.getLocation(loc, move.dir);
        auto& future_state = get_state(future_loc);
        const auto& future_site = map_.getSite(future_loc);

        // this occurs due to reinforce being able to traverse through 0str cells that may or may not be ours
        if (current_site.owner != id_) {
            return false;
        }

        if (should_idle(current_site) && future_site.owner == id_) {
            LOGZ << "rejecting premature reinforce from (" << loc.x << "," << loc.y << ")" << std::endl;
            return false;
        }

        // reject illegal moves
        if (future_state.potential + current_site.strength > config_.wander_clobber_ceiling) {
            LOGZ << "rejecting clobber from (" << loc.x << "," << loc.y << ")" << std::endl;
            return false;
        }

        future_state.potential += current_site.strength;
        current_state.potential -= current_site.strength;

        if (erase) {
            mine_.erase(loc);
        }

        orders_[loc] = move;
        return true;
    }

    void do_attack(std::vector<hlt::Location>& targets);
    int do_try_reinforce(const hlt::Location& root_target, const hlt::Location& target, int depth_limit, int depth,
                         int power, int production);
    void do_try_attack(hlt::Location target);
    void do_try_retreat(hlt::Location target);
    void do_wander(const hlt::Location loc);

    std::vector<std::pair<direction_t, hlt::Location>> get_neighbors(hlt::Location loc);

    typedef std::function<void(site_info)> range_fn;

    float score_region(const hlt::Location& loc);

    void nearby_region(const hlt::Location& loc, distance_t radius, range_fn fn);
    void range_all(range_fn fn);
    void range_do(distance_t y_start, distance_t y_end, distance_t x_start, distance_t x_end, range_fn);
};