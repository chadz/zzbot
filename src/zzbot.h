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

struct zzbot_config {
    std::string name = "zzbot";
    unsigned short production_move_scalar = 5;
    unsigned short score_region_radius = 10;
    float score_enemy_scalar = 0.55f;
    int wander_clobber_ceiling = 350;
    unsigned short max_wait_for_attack = 1;
    bool should_log = true;

    int max_reinforce_depth = 5;
    int min_reinforce_depth = 1;
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

    std::vector<std::vector<site_state>> state_;

    game_state game_state_;

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

    void mark_visited(const hlt::Location& loc) {
        auto& state = get_state(loc);
        state.visited = true;
    }

    bool has_visited(const hlt::Location& loc) {
        const auto& state = get_state(loc);
        return state.visited;
    }

    template <class P>
    std::vector<std::pair<direction_t, hlt::Location>> get_neighbors(const hlt::Location& location, P fn) {

        auto neighbors = get_neighbors(location);

        if (get_state(location).score > 0) {
            LOGZ << "score  (" << location.x << "," << location.y << ")"
                 << ": " << get_state(location).score << std::endl;
        }

        filter(neighbors, [&](std::pair<direction_t, hlt::Location> neighbor) { return fn(neighbor.second); });

        return neighbors;
    }

    direction_t get_angle(const hlt::Location& from, const hlt::Location& to) {

        float angle = map_.getAngle(from, to) * 180.0 / 3.14159265;
        auto roll = rand() % 90;

        if (angle > 0 && angle <= 90) {
            return roll % 90 > angle ? NORTH : EAST;
        }
        if (angle > 90 && angle <= 180) {
            return 90 + roll % 90 > angle ? NORTH : WEST;
        }

        angle = abs(angle);

        if (angle > 0 && angle <= 90) {
            return roll % 90 > angle ? SOUTH : EAST;
        }
        if (angle > 90 && angle <= 180) {
            return 90 + roll % 90 > angle ? SOUTH : WEST;
        }

        return EAST;
    }

    site_state& get_state(const hlt::Location& loc) {
        return state_[loc.y][loc.x];
    }

    bool should_idle(const hlt::Site& site);
    bool assigned_move(const hlt::Location& loc) const;

    bool assign_move(const hlt::Move& move, bool erase = true) {

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
    void do_old_wander(const hlt::Location loc);

    std::vector<std::pair<direction_t, hlt::Location>> get_neighbors(hlt::Location loc);

    typedef std::function<void(const hlt::Location, const hlt::Site&)> range_fn;
    typedef optional<std::pair<hlt::Location, distance_t>> maybe_neighbor_t;

    std::array<maybe_neighbor_t, 5> distance_to_borders(const hlt::Location loc);

    direction_t find_nearest_border(const hlt::Location loc);
    float score_region(const hlt::Location& loc);
    void nearby_region(const hlt::Location& loc, distance_t radius, range_fn fn);
    void range_all(range_fn fn);
    void range_do(distance_t y_start, distance_t y_end, distance_t x_start, distance_t x_end, range_fn);
};