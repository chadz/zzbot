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
    float score_enemy_scalar = 2.0f;
    int wander_clobber_ceiling = 300;
    unsigned short max_wait_for_attack = 2;
    bool should_log = true;

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
    bool are_facing(const hlt::Location& a, const hlt::Location& b)
    {
        return a.x == b.x || a.y == b.y;
    }

    direction_t get_direction(const hlt::Location& from, const hlt::Location& to)
    {
        auto path = get_path(from, to);
        if (path.empty() || path.size() == 1) return STILL;

        LOGZ << "path: ";
        path.pop_front();

        for (auto p : path) {
            LOGZ << "(" << p.x << ", " << p.y << ") -> ";
        }
        LOGZ << std::endl;

        auto next = path.front();

        for (auto dir : CARDINALS) {
            if (map_.getLocation(from, dir) == next) {
                return dir;
            }
        }

        return STILL;
    }

    std::list<hlt::Location> get_path(const hlt::Location& from, const hlt::Location& to)
    {
        struct score {
            int g;
            float f;
        };

        std::map<hlt::Location, score> scores;
        std::map<hlt::Location, hlt::Location> came_from;
        std::set<hlt::Location> open;
        std::set<hlt::Location> closed;

        open.emplace(from);
        scores[from] = {0, map_.getDistance(from, to)};

        while (!open.empty()) {

            auto current = *std::min_element(open.begin(), open.end());

            if (current == to) {

                return reconstruct_path(came_from, current);
            }

            open.erase(current);
            closed.emplace(current);

            auto neighbors = get_neighbors(current, [&](site_info ni) {
                return !((!ni.state.poisoned && (ni.site.owner == id_ || ni.site.strength == 0 || ni.loc == to)));
            });

            for (const auto& neighbor_pair : neighbors) {
                auto neighbor = neighbor_pair.second;

                if (closed.find(neighbor) != closed.end()) {
                    continue;
                }

                auto gscore = scores[current].g + (int)map_.getDistance(current, neighbor);

                if (open.find(neighbor) == open.end()) {
                    open.emplace(neighbor);
                } else if (gscore > scores[neighbor].g) {
                    continue;
                }

                scores[neighbor] = {gscore, map_.getDistance(current, neighbor)};
                came_from[neighbor] = current;
            }
        }
        return std::list<hlt::Location>();
    }

    std::list<hlt::Location> reconstruct_path(std::map<hlt::Location, hlt::Location>& came_from,
                                              const hlt::Location& current)
    {
        std::list<hlt::Location> path;
        hlt::Location cur = current;

        path.insert(path.begin(), cur);

        while (came_from.find(cur) != came_from.end()) {
            cur = came_from[cur];
            path.insert(path.begin(), cur);
        }

        return path;
    }

    void poison_area(const hlt::Location& location)
    {
        get_state(location).poisoned = true;
        for (auto dir : CARDINALS) {
            get_state(map_.getLocation(location, dir)).poisoned = true;
        }
    }

    direction_t reverse_direction(direction_t direction);

    site_state& get_state(const hlt::Location& loc);
    site_info get_info(const hlt::Location& location);
    direction_t get_angle(const hlt::Location& from, const hlt::Location& to);

    void mark_visited(const hlt::Location& loc);
    bool has_visited(const hlt::Location& loc);

    void calc_state();
    float score_region(const hlt::Location& loc);
    void behavior();
    bool should_idle(const hlt::Site& site) const;

    bool assigned_move(const hlt::Location& loc) const;
    bool assign_move(const hlt::Move& move, bool force, bool erase = true);
    bool force_move(const hlt::Move& move)
    {
        return assign_move(move, true, false);
    }

    void do_attack(float avg_score);
    void do_wander();

    int do_try_reinforce(const hlt::Location& root_target, const hlt::Location& target, int depth_limit, int depth,
                         int power, int production);
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