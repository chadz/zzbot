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
};

struct zzbot_config {
    std::string name = "zzbot";
    unsigned short production_move_scalar = 5;
    unsigned short score_region_radius = 1;
    float score_enemy_scalar = 1.0f;
    int wander_clobber_ceiling = 350;
    unsigned short max_wait_for_attack = 1;
    bool should_log = true;
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

    template <class P>
    std::vector<std::pair<direction_t, hlt::Location>> get_neighbors(const hlt::Location& location, P fn) {

        auto neighbors = get_neighbors(location);

        filter(neighbors, [&](std::pair<direction_t, hlt::Location> neighbor) { return fn(neighbor.second); });

        if (!neighbors.empty()) {
            LOGZ << "score  (" << location.x << "," << location.y << ")"
                 << ": " << state_[location.y][location.x].score << std::endl;
        }

        return neighbors;
    }

    site_state& get_state(const hlt::Location& loc) {
        return state_[loc.y][loc.x];
    }

    bool should_idle(const hlt::Site& site);
    bool assigned_move(const hlt::Location& loc) const;

    void assign_move(const hlt::Location& loc, const hlt::Move& move, bool erase = true) {

        auto future_loc = map_.getLocation(loc, move.dir);
        auto& future_state = get_state(future_loc);
        const auto& future_site = map_.getSite(future_loc);

        auto& current_state = get_state(loc);
        const auto& current_site = map_.getSite(loc);

        // reject illegal moves
        if (future_state.potential + current_site.strength > config_.wander_clobber_ceiling) {
            return;
        }

        future_state.potential += current_site.strength;
        current_state.potential -= current_site.strength;

        if (erase) {
            mine_.erase(loc);
        }

        orders_[loc] = move;
    }

    void do_attack(std::vector<hlt::Location>& targets);
    void do_try_reinforce(hlt::Location target);
    void do_try_attack(hlt::Location target);
    void do_wander(const hlt::Location loc);

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