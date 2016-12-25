#include "zzbot.h"

#include <algorithm>
#include <limits>
#include <numeric>
// todo: optimize decision making between wandering and waiting to attack
// maybe a retreat if no overkill is really going to occur
// figure out how moves are actually executed... can strength be moved from a neighbor and attack the same turn? iono!

zzbot::zzbot(zzbot_config cfg) : config_(cfg) {
    getInit(id_, map_);
    sendInit(config_.name);

    log_.open("log." + std::to_string(id_) + ".txt", std::fstream::in | std::fstream::out | std::fstream::app);
    LOGZ << "id: " << (int)id_ << std::endl;
}

zzbot::~zzbot() {
    log_.close();
}

void zzbot::calc_state() {

    mine_.clear();
    enemies_.clear();
    game_state_.population = 0u;
    game_state_.players.clear();

    state_ = std::vector<std::vector<site_state>>(map_.height, std::vector<site_state>(map_.width));

    range_all([&](const hlt::Location loc, const hlt::Site& site) {

        auto& state = get_state(loc);
        auto neighbors = get_neighbors(loc);
        bool border_site = false;

        if (site.owner == 0) {

            state.potential -= site.strength;

            auto enemies = get_neighbors(loc, [this](hlt::Location location) {
                const auto& site = map_.getSite(location);
                return !(site.owner != id_ && site.owner != 0);
            });

            auto others = get_neighbors(loc, [this](hlt::Location location) {
                const auto& site = map_.getSite(location);
                return !(site.owner == id_ || site.owner == 0);
            });

            unsigned int power =
                std::accumulate(enemies.cbegin(), enemies.cend(), 0u,
                                [this](unsigned int power, std::pair<direction_t, hlt::Location> enemy) {
                                    return power + map_.getSite(enemy.second).strength;
                                });

            state.potential -= power;

            // reduce neighboring sites potential by the enemy power
            for (const auto& other : others) {

                auto& other_state = get_state(other.second);
                other_state.potential -= power;

                if (map_.getSite(other.second).owner == id_) {
                    border_site = true;
                }
            }

            if (border_site) {
                enemies_.emplace_back(loc);
            }

        } else if (site.owner != id_) {

            game_state_.population++;
            auto& player = game_state_.players[site.owner];

            player.population++;
            player.strength += site.strength;
            player.production += site.production;
            state.potential -= site.strength;

            for (const auto& neighbor : neighbors) {
                auto s = map_.getSite(neighbor.second);
                if (s.owner != id_ && s.owner != 0) {
                    state.potential -= s.strength;
                }

                if (s.owner == id_) {
                    border_site = true;
                }
            }

            if (border_site) {
                enemies_.emplace_back(loc);
            }

        } else {

            game_state_.population++;
            auto& player = game_state_.players[site.owner];

            player.population++;
            player.strength += site.strength;
            player.production += site.production;

            state.potential = site.strength;
            mine_.emplace(loc);
        }
    });

    range_all([&](const hlt::Location loc, const hlt::Site& site) {
        auto& state = get_state(loc);
        state.score = score_region(loc);
    });

    std::sort(enemies_.begin(), enemies_.end(), [this](const hlt::Location& a, const hlt::Location& b) {
        auto a_potential = map_.getSite(a).owner == 0 ? 1 : abs(get_state(a).potential);
        auto b_potential = map_.getSite(b).owner == 0 ? 1 : abs(get_state(b).potential);

        // return get_state(a).score * a_potential > get_state(b).score * b_potential;
        return get_state(a).score > get_state(b).score;
    });
}

void zzbot::behavior() {

    calc_state();

    do_attack(enemies_);

    // wander with whatever wasn't assigned a move
    for (const auto& mine : mine_) {
        do_old_wander(mine);
    }
}

void zzbot::run() {

    std::set<hlt::Move> moves;
    for (int frame = 0;; ++frame) {

        LOGZ << "frame: " << frame << std::endl;
        for (const auto player : game_state_.players) {
            LOGZ << " player: " << player.first << " strength: " << player.second.strength
                 << " production: " << player.second.production << " population: " << player.second.population
                 << std::endl;
        }
        LOGZ << "================================================================ " << std::endl;
        moves.clear();
        orders_.clear();

        getFrame(map_);

        behavior();

        for (const auto& move : orders_) {
            moves.emplace(move.second);
        }

        sendFrame(moves);
    }
}

// find_nearest_border a subset of this
std::array<optional<std::pair<hlt::Location, distance_t>>, 5> zzbot::distance_to_borders(const hlt::Location loc) {

    std::array<optional<std::pair<hlt::Location, distance_t>>, 5> distances;

    for (direction_t dir : CARDINALS) {
        unsigned int distance = 1;
        hlt::Location runner = loc;

        for (; distance < (std::max)(map_.width, map_.height); ++distance) {
            runner = map_.getLocation(runner, dir);
            if (map_.getSite(runner).owner != id_) {
                distances[dir] = optional<std::pair<hlt::Location, distance_t>>({runner, distance});
                break;
            }
        }
    }

    return distances;
}

direction_t zzbot::find_nearest_border(const hlt::Location loc) {

    unsigned int min_distance = 0;
    direction_t direction = 1;

    for (direction_t dir : CARDINALS) {
        unsigned int distance = 1;
        hlt::Location runner = loc;

        for (; distance < (std::max)(map_.width, map_.height); ++distance) {
            runner = map_.getLocation(runner, dir);
            if (map_.getSite(runner).owner != id_) {
                if (min_distance == 0 || distance < min_distance) {
                    min_distance = distance;
                    direction = dir;
                    break;
                }
            }
        }
    }
    return direction;
}

bool zzbot::assigned_move(const hlt::Location& loc) const {
    return mine_.find(loc) == mine_.end();
}

void zzbot::do_attack(std::vector<hlt::Location>& targets) {
    int depth = config_.max_reinforce_depth;
    for (const auto& target : targets) {

        do_try_reinforce(target, target, (std::max)(config_.min_reinforce_depth, depth--), 0, 0, 0);
        // do_try_retreat(target);
        do_try_attack(target);
    }
}

int zzbot::do_try_reinforce(const hlt::Location& root_target, const hlt::Location& target, int depth_limit, int depth,
                            int power, int production) {

    const auto& target_site = map_.getSite(target);

    if (depth >= depth_limit || has_visited(target)) {
        return power;
    }

    mark_visited(target);

    auto supporters = get_neighbors(target, [this](hlt::Location location) {
        const auto& site = map_.getSite(location);
        const auto& state = get_state(location);
        return !((site.owner == id_ && !has_visited(location)) || (site.strength == 0 && !has_visited(location)));
    });

    if (supporters.empty()) {
        return power;
    }

    LOGZ << "looking for reinforcements for  (" << target.x << "," << target.y << ") depth " << depth << std::endl;

    int total_power{};

    for (const auto& supporter : supporters) {

        const auto& supporter_loc = supporter.second;
        const auto& supporter_site = map_.getSite(supporter.second);

        auto next_power = power + supporter_site.strength + production;
        auto next_production = production + supporter_site.production;

        total_power +=
            do_try_reinforce(root_target, supporter_loc, depth_limit, depth + 1, next_power, next_production);
    }

    for (const auto& supporter : supporters) {

        const auto& supporter_loc = supporter.second;

        LOGZ << "checking reinforcing (" << supporter_loc.x << "," << supporter_loc.y << ")"
             << "to (" << target.x << "," << target.y << ")" << std::endl;

        if (target_site.owner == id_) {

            const auto& root_site = map_.getSite(root_target);
            if (total_power > root_site.strength) {

                LOGZ << "reinforcing (" << supporter_loc.x << "," << supporter_loc.y << ")"
                     << "to (" << target.x << "," << target.y << ")" << std::endl;

                assign_move({supporter_loc, supporter.first});
            } else {

                LOGZ << " waiting for reinforcmeents at (" << supporter_loc.x << ", " << supporter_loc.y << ") "
                     << " for (" << target.x << "," << target.y << ")" << std::endl;

                assign_move({supporter_loc, STILL});
            }
        }
    }

    return power;
}

void zzbot::do_try_attack(hlt::Location target) {

    const auto& target_site = map_.getSite(target);

    auto attackers = get_neighbors(target, [this](hlt::Location location) {
        const auto& site = map_.getSite(location);
        return !(site.owner == id_ && !assigned_move(location));
    });

    // std::sort(attackers.begin(), attackers.end(),
    //           [this](const std::pair<direction_t, hlt::Location>& a, const std::pair<direction_t, hlt::Location>&
    //           b)
    //           {
    //               return map_.getSite(a.second).strength > map_.getSite(b.second).strength;
    //           });

    // sum current and future powers
    int total_power{};
    int future_power{};
    for (const auto& attacker : attackers) {
        const auto& site = map_.getSite(attacker.second);
        const auto& state = get_state(attacker.second);
        total_power += site.strength;
        future_power += state.potential + site.production * config_.max_wait_for_attack;
    }

    for (const auto& attacker : attackers) {

        auto attacker_loc = attacker.second;
        const auto& attacker_site = map_.getSite(attacker_loc);
        const auto& target_state = get_state(target);

        LOGZ << "trying to attack from (" << attacker_loc.x << "," << attacker_loc.y << ","
             << (int)attacker_site.strength << ")"
             << "to (" << target.x << "," << target.y << "," << (int)target_site.strength << ","
             << target_state.potential << ") combined power: " << total_power << std::endl;

        if (target_site.owner == 0 && total_power < target_site.strength && future_power > target_site.strength) {
            LOGZ << "waiting to attack from (" << attacker_loc.x << "," << attacker_loc.y << "," << future_power << ")"
                 << std::endl;
            assign_move({attacker_loc, STILL});
            continue;
        }

        if ((target_site.owner != 0 && total_power >= target_site.strength) ||
            (target_site.owner == 0 && total_power > target_site.strength)) {

            assign_move({attacker_loc, attacker.first});

            // // abort once we commited a single guy to a zero str node
            // if (target_site.strength == 0) {
            //     break;
            // }
        }
    }
}

void zzbot::do_try_retreat(hlt::Location target) {

    const auto& target_site = map_.getSite(target);

    LOGZ << "attempting retreat for  (" << target.x << "," << target.y << ")" << std::endl;

    auto enemies = get_neighbors(target, [this](hlt::Location location) {
        const auto& site = map_.getSite(location);
        return !(site.owner != id_ && site.owner != 0);
    });

    auto allies = get_neighbors(target, [this](hlt::Location location) {
        const auto& site = map_.getSite(location);
        return !(site.owner == id_ && !assigned_move(location));
    });

    LOGZ << "found " << enemies.size() << " enemies and " << allies.size() << " allies" << std::endl;

    if (!enemies.empty() && !allies.empty()) {

        std::sort(allies.begin(), allies.end(), [this](const std::pair<direction_t, hlt::Location>& a,
                                                       const std::pair<direction_t, hlt::Location>& b) {
            return map_.getSite(a.second).strength < map_.getSite(b.second).strength;
        });

        // leave our biggest to attack or chill, but we must retreat with the rest
        auto biggest_ally = allies.back();
        allies.pop_back();

        // poison this tile so nobody moves into it
        LOGZ << "poisoning (" << biggest_ally.second.x << "," << biggest_ally.second.y << ")" << std::endl;
        get_state(biggest_ally.second).potential = config_.wander_clobber_ceiling;

        for (const auto& ally : allies) {
            auto& state = get_state(ally.second);

            auto retreat_spots = get_neighbors(ally.second, [&](hlt::Location location) {
                const auto& retreat_site = map_.getSite(location);
                auto& retreat_state = get_state(ally.second);
                return !(retreat_site.owner == id_);
            });

            for (const auto& retreat_attempt : retreat_spots) {
                if (assign_move({ally.second, retreat_attempt.first})) {
                    LOGZ << "retreating (" << ally.second.x << "," << ally.second.y << ") to ("
                         << map_.getLocation(ally.second, retreat_attempt.first).x << ","
                         << map_.getLocation(ally.second, retreat_attempt.first).y << ")" << std::endl;
                    continue;
                }
            }
            // poison this tile so nobody moves into it
            LOGZ << "poisoning (" << ally.second.x << "," << ally.second.y << ")" << std::endl;
            state.potential = config_.wander_clobber_ceiling;
        }
    }
}

// xxx: i think this might be better dynamic based on map size
bool zzbot::should_idle(const hlt::Site& site) {
    return (site.strength < site.production * config_.production_move_scalar);
}

void zzbot::do_old_wander(const hlt::Location loc) {

    const auto& site = map_.getSite(loc);
    if (should_idle(site)) {
        return;
    }

    auto borders = distance_to_borders(loc);

    // both planes are taken; just move randomly SW
    if (std::none_of(borders.cbegin(), borders.cend(),
                     [](const optional<std::pair<hlt::Location, distance_t>>& border) { return border.has_value(); })) {

        // direction_t random_dir = 3 + (rand() & 1);
        // const auto& future_state = get_state(map_.getLocation(loc, random_dir));
        // assign_move(loc, {loc, random_dir}, false);

        // maybe this is a good use of the shitty new wander method
        do_wander(loc);
        return;
    }

    // move in most valuable direction
    float best_score = std::numeric_limits<float>::lowest();
    direction_t best_dir = STILL;

    for (direction_t dir : CARDINALS) {
        // this happens when we control the entire row or column
        if (!borders[dir].has_value()) {
            continue;
        }

        const auto& border_loc = borders[dir].value().first;
        auto border_distance = borders[dir].value().second;
        const auto& border_score = get_state(border_loc).score;
        auto score = border_score / (float)(border_distance * border_distance);

        // LOGZ << "attempting to wander score: " << border_score << " distance: "
        // << (float)(border_distance * 2)
        //     << " direction: " << (int)dir << std::endl;

        if (score > best_score) {
            best_dir = dir;
            best_score = score;
        }
    }
    LOGZ << "wandering score: " << best_score << " dir: " << (int)best_dir << std::endl;

    auto proposed_loc = map_.getLocation(loc, best_dir);
    const auto& proposed_site = map_.getSite(proposed_loc);
    const auto& proposed_state = get_state(proposed_loc);

    if (proposed_site.owner == id_) {

        LOGZ << "wandering from (" << loc.x << "," << loc.y << ")"
             << "to (" << proposed_loc.x << "," << proposed_loc.y << ")" << std::endl;

        assign_move({loc, best_dir}, false);
    }
}

void zzbot::do_wander(const hlt::Location loc) {

    const auto& site = map_.getSite(loc);
    if (should_idle(site)) {
        return;
    }

    hlt::Location best_loc = loc;
    float best_score = (std::numeric_limits<float>::min)();
    for (const auto& enemy : enemies_) {
        auto distance = map_.getDistance(loc, enemy);

        auto enemy_score = get_state(enemy).score / (distance * distance * distance);

        if (enemy_score > best_score) {
            best_score = enemy_score;
            best_loc = enemy;
        }
    }

    auto direction = get_angle(loc, best_loc);
    const auto& destination_site = map_.getSite(loc, direction);
    if (destination_site.owner == id_) {
        if (assign_move({loc, direction}, false)) {

            LOGZ << "wandering from (" << loc.x << "," << loc.y << " to (" << best_loc.x << "," << best_loc.y
                 << ") direction " << (int)direction << std::endl;
            return;
        }
    }
}

float zzbot::score_region(const hlt::Location& loc) {

    float score = 0.0f;

    nearby_region(loc, config_.score_region_radius, [&](const hlt::Location l, const hlt::Site& site) {

        float local_score = 0.0f;
        float distance = map_.getDistance(l, loc);
        float power = (std::max)(1.0f, (float)site.strength);
        const auto& state = get_state(l);

        if (site.owner == id_) {

            // const auto& orig_site = map_.getSite(loc);
            // if (orig_site.owner != id_) {
            //     local_score = (site.strength * 0.1f) / distance;
            //     // local_score = (float)(orig_site.production) / (std::max)(1.0f, (float)orig_site.strength);
            // }

        } else if (site.owner != 0) {

            // local_score = abs(state.potential) * (1.01f - 1.0f / (float)site.production);
            // const auto& player = game_state_.players[site.owner];
            // const auto& me = game_state_.players[id_];
            // local_score *= config_.score_enemy_scalar *
            //                ((std::max)(1.0f, (float)me.strength) / (std::max)(1.0f, (float)player.strength));

        } else {

            auto military_value = (std::max)(1.0f, (float)(abs(state.potential + site.strength)));
            local_score = (military_value * (float)(site.production * site.production)) / power;
        }

        local_score /= 1 + distance * distance * distance;

        score += local_score;
    });

    return score;
}

void zzbot::nearby_region(const hlt::Location& loc, distance_t radius, range_fn fn) {

    distance_t radius_y = (std::min)((distance_t)(radius * 2), (distance_t)(map_.height / 2));
    distance_t radius_x = (std::min)((distance_t)(radius * 2), (distance_t)(map_.width / 2));
    auto l = loc;

    // LOGZ << "nearby (" << loc.x << "," << loc.y << ") radius " << radius << std::endl;
    // wind to 'top-left'
    for (distance_t r = 0; r < radius_y; ++r) {
        l = map_.getLocation(l, NORTH);
    }

    for (distance_t r = 0; r < radius_x; ++r) {
        l = map_.getLocation(l, WEST);
    }

    // LOGZ << "start (" << l.x << "," << l.y << ")" << std::endl;

    for (distance_t y = 0; y < radius_y * 2; y++) {
        auto runner = l;

        for (distance_t x = 0; x < radius_x * 2; x++) {
            // LOGZ << "running (" << runner.x << "," << runner.y << ")" << std::endl;
            runner = map_.getLocation(runner, EAST);
            fn(runner, map_.getSite(runner));
        }

        l = map_.getLocation(l, SOUTH);
    }
}

void zzbot::range_all(range_fn fn) {
    range_do(0, map_.height, 0, map_.width, fn);
}

void zzbot::range_do(distance_t y_start, distance_t y_end, distance_t x_start, distance_t x_end, range_fn fn) {

    for (distance_t y = y_start; y < y_end; y++) {
        for (distance_t x = x_start; x < x_end; x++) {
            fn({x, y}, map_.getSite({x, y}));
        }
    }
}

std::vector<std::pair<direction_t, hlt::Location>> zzbot::get_neighbors(hlt::Location loc) {

    std::vector<std::pair<direction_t, hlt::Location>> neighbors;

    for (direction_t dir : CARDINALS) {
        neighbors.emplace_back(reverse_direction(dir), map_.getLocation(loc, dir));
    }

    return neighbors;
}
