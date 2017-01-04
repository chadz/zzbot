#pragma once

#include "doodads.h"
#include "hlt.hpp"
#include "networking.hpp"

#include <array>
#include <functional>
#include <map>
#include <unordered_map>

#include <queue>

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
    bool poisoned = false;
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

    float avg_str;
    float avg_prod;
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
    int wander_clobber_ceiling = 275;
    unsigned short max_wait_for_attack = 4;
    bool should_log = false;

    float score_enemy_scalar = 2.0f;
    int bonus_turns = 20;
    int max_reinforce_depth = 6;
    int min_reinforce_depth = 1;
};

class zzbot
{
  private:
    zzbot_config config_{};
    unsigned char id_{};

    std::fstream log_;
    hlt::GameMap map_;
    std::vector<std::vector<hlt::Site>> orig_map_;

    std::map<hlt::Location, hlt::Move> orders_;
    std::set<hlt::Location> mine_;
    std::vector<hlt::Location> enemies_;
    std::vector<std::vector<site_state>> state_;

    game_state game_state_;

  public:
    void run();
    zzbot(zzbot_config cfg);
    ~zzbot();

  private:
    template <class P>
    void traverse(const hlt::Location& location, P fn)
    {
        auto info = get_info(location);

        if (info.state.visited || info.state.border) return;

        info.state.visited = true;

        auto neighbors = get_neighbors(location, [&](site_info si) { return si.site.owner == id_; });

        for (const auto& neighbor : neighbors) {
            traverse(neighbor.second, fn);
            fn(neighbor.second);
        }
    }
    direction_t get_direction(const hlt::Location& from, const hlt::Location& to, bool wait = true);

    std::list<hlt::Location> get_path(const hlt::Location& from, const hlt::Location& to);

    std::list<hlt::Location> reconstruct_path(std::map<hlt::Location, hlt::Location>& came_from,
                                              const hlt::Location& current);

    void poison_area(const hlt::Location& location);

    direction_t reverse_direction(direction_t direction);

    site_state& get_state(const hlt::Location& loc);
    site_info get_info(const hlt::Location& location);

    void mark_visited(const hlt::Location& loc);
    bool has_visited(const hlt::Location& loc);

    void calc_state();
    float score_region(const hlt::Location& loc);
    void behavior();
    bool should_idle(const hlt::Site& site) const;

    bool assigned_move(const hlt::Location& loc) const;
    bool assign_move(const hlt::Move& move, bool force);
    bool force_move(const hlt::Move& move)
    {
        return assign_move(move, true);
    }

    void do_attack(float avg_score);
    void do_try_wander(const hlt::Location& location);
    void do_wander();

    void do_try_expand(hlt::Location target);
    void do_try_tactics(hlt::Location target);

    std::vector<std::pair<direction_t, hlt::Location>> get_neighbors(const hlt::Location& location);
    std::vector<std::pair<direction_t, hlt::Location>> get_neighbors(const hlt::Location& location,
                                                                     std::function<bool(site_info)> fn);

    typedef std::function<void(site_info)> range_fn;

    void nearby_region(const hlt::Location& loc, distance_t radius, range_fn fn);
    void range_all(range_fn fn);
    void range_do(distance_t y_start, distance_t y_end, distance_t x_start, distance_t x_end, range_fn fn);
};