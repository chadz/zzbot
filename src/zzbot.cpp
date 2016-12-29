#include "zzbot.h"

#include <algorithm>
#include <limits>
#include <numeric>

zzbot::zzbot(zzbot_config cfg) : config_(cfg)
{
    getInit(id_, map_);
    sendInit(config_.name);

    if (config_.should_log) {
        log_.open("log." + std::to_string(id_) + ".txt", std::fstream::in | std::fstream::out | std::fstream::app);
        LOGZ << "id: " << (int)id_ << std::endl;
    }
}

zzbot::~zzbot()
{
    if (config_.should_log) {
        log_.close();
    }
}

direction_t zzbot::reverse_direction(direction_t direction)
{
    int rdir[] = {STILL, SOUTH, WEST, NORTH, EAST};
    return rdir[direction];
}

site_state& zzbot::get_state(const hlt::Location& loc)
{
    return state_[loc.y][loc.x];
}

site_info zzbot::get_info(const hlt::Location& location)
{
    return site_info{location, map_.getSite(location), get_state(location)};
}

void zzbot::mark_visited(const hlt::Location& loc)
{
    auto& state = get_state(loc);
    state.visited = true;
}

bool zzbot::has_visited(const hlt::Location& loc)
{
    const auto& state = get_state(loc);
    return state.visited;
}

void zzbot::calc_state()
{
    mine_.clear();
    enemies_.clear();
    game_state_.population = 0u;
    game_state_.players.clear();

    state_ = std::vector<std::vector<site_state>>(map_.height, std::vector<site_state>(map_.width));

    range_all([&](site_info info) {

        auto neighbors = get_neighbors(info.loc);
        auto& player = game_state_.players[info.site.owner];
        player.population++;
        player.strength += info.site.strength;
        player.production += info.site.production;

        if (info.site.owner == 0) {

            info.state.potential -= info.site.strength;

            auto enemies =
                get_neighbors(info.loc, [this](site_info ni) { return !(ni.site.owner != id_ && ni.site.owner != 0); });

            auto others =
                get_neighbors(info.loc, [this](site_info ni) { return !(ni.site.owner == id_ || ni.site.owner == 0); });

            unsigned int power =
                std::accumulate(enemies.cbegin(), enemies.cend(), 0u,
                                [this](unsigned int power, std::pair<direction_t, hlt::Location> enemy) {
                                    return power + map_.getSite(enemy.second).strength;
                                });

            info.state.potential -= power;

            for (const auto& other : others) {

                // // reduce neighboring sites potential by the enemy power
                // auto& other_state = get_state(other.second);
                // other_state.potential -= power;

                if (map_.getSite(other.second).owner == id_) {
                    info.state.border = true;
                }
            }

            if (info.state.border) {
                enemies_.emplace_back(info.loc);
            }

        } else if (info.site.owner != id_) {

            game_state_.population++;

            for (const auto& neighbor : neighbors) {
                auto s = map_.getSite(neighbor.second);
                if (s.owner == 0) {
                    info.state.potential -= s.strength;
                }
            }

        } else {

            game_state_.population++;
            info.state.potential = info.site.strength;
            mine_.emplace(info.loc);
        }
    });

    range_all([&](site_info info) { info.state.score = score_region(info.loc); });

    std::sort(enemies_.begin(), enemies_.end(), [this](const hlt::Location& a, const hlt::Location& b) {
        auto a_potential = map_.getSite(a).owner == 0 ? 1 : abs(get_state(a).potential);
        auto b_potential = map_.getSite(b).owner == 0 ? 1 : abs(get_state(b).potential);

        // return get_state(a).score * a_potential > get_state(b).score * b_potential;
        return get_state(a).score > get_state(b).score;
    });
}

void zzbot::behavior()
{
    calc_state();
    do_attack();
    do_wander();
}

void zzbot::run()
{
    std::set<hlt::Move> moves;
    for (game_state_.frame = 0;; game_state_.frame++) {

        LOGZ << "frame: " << game_state_.frame << std::endl;
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

bool zzbot::assigned_move(const hlt::Location& loc) const
{
    return mine_.find(loc) == mine_.end();
}

void zzbot::do_attack()
{
    int depth = config_.max_reinforce_depth;
    for (const auto& target : enemies_) {
        do_try_tactics(target);
        do_try_attack(target);
        do_try_reinforce(target, target, (std::max)(config_.min_reinforce_depth, depth--), 0, 0, 0);
    }
}

int zzbot::do_try_reinforce(const hlt::Location& root_target, const hlt::Location& target, int depth_limit, int depth,
                            int power, int production)
{
    const auto& target_site = map_.getSite(target);

    if (depth >= depth_limit || has_visited(target)) {
        return power;
    }

    mark_visited(target);

    auto supporters = get_neighbors(target, [this](site_info ni) {
        return !((ni.site.owner == id_ && !has_visited(ni.loc)) || (ni.site.strength == 0 && !has_visited(ni.loc)));
    });

    if (supporters.empty()) {
        return power;
    }

    LOGZ << "looking for reinforcements for  (" << target.x << "," << target.y << ") depth " << depth << std::endl;

    // int immediate_power{};
    // for (const auto& supporter : supporters) {
    //     auto si = get_info(supporter.second);
    //     immediate_power += si.site.strength;
    //     total_power += do_try_reinforce(root_target, si.loc, depth_limit, depth + 1, next_power, next_production);
    // }

    int total_power{};
    for (const auto& supporter : supporters) {
        auto si = get_info(supporter.second);
        auto next_power = power + si.site.strength + production;
        auto next_production = production + si.site.production;
        total_power += do_try_reinforce(root_target, si.loc, depth_limit, depth + 1, next_power, next_production);
    }

    for (const auto& supporter : supporters) {
        auto si = get_info(supporter.second);

        LOGZ << "checking reinforcing (" << si.loc.x << "," << si.loc.y << ")"
             << "to (" << target.x << "," << target.y << ")" << std::endl;

        // if (target_site.owner == 0 && immediate_power > target_site.strength) {

        //     auto dest = map_.getLocation(si.loc, supporter.first);
        //     LOGZ << "attacking with (" << si.loc.x << "," << si.loc.y << ")"
        //          << "to (" << dest.x << "," << dest.y << ")" << std::endl;
        //     assign_move({si.loc, supporter.first});

        if (target_site.owner == id_) {

            const auto& root_site = map_.getSite(root_target);
            if (total_power > root_site.strength) {

                LOGZ << "reinforcing (" << si.loc.x << "," << si.loc.y << ")"
                     << "to (" << target.x << "," << target.y << ")" << std::endl;

                auto direction = get_angle(si.loc, root_target);
                const auto& direction_site = map_.getSite(si.loc, direction);
                if (direction_site.owner == id_ || direction_site.strength == 0) {
                    assign_move({si.loc, direction});
                } else {
                    assign_move({si.loc, supporter.first});
                }

            } else {

                LOGZ << " waiting for reinforcmeents at (" << si.loc.x << ", " << si.loc.y << ") "
                     << " for (" << target.x << "," << target.y << ")" << std::endl;

                assign_move({si.loc, STILL});
            }
        }
    }

    return power;
}

void zzbot::do_try_attack(hlt::Location target)
{
    const auto& target_site = map_.getSite(target);

    auto attackers =
        get_neighbors(target, [this](site_info ni) { return !(ni.site.owner == id_ && !assigned_move(ni.loc)); });

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

        // if (target_site.owner == 0 && total_power < target_site.strength && future_power > target_site.strength)
        //{
        //    LOGZ << "waiting to attack from (" << attacker_loc.x << "," << attacker_loc.y << "," << future_power
        //    <<
        //    ")"
        //         << std::endl;
        //    assign_move({attacker_loc, STILL});
        //    continue;
        //}

        if (total_power > target_site.strength) {

            assign_move({attacker_loc, attacker.first});

            // // abort once we commited a single guy to a zero str node
            // if (target_site.strength == 0) {
            //     break;
            // }
        }
    }
}

void zzbot::do_try_tactics(hlt::Location target)
{

    auto ti = get_info(target);
    LOGZ << "special tactics at (" << target.x << "," << target.y << ")" << std::endl;

    auto enemies =
        get_neighbors(target, [this](site_info ni) { return !(ni.site.owner != id_ && ni.site.owner != 0); });

    auto allies = get_neighbors(target, [this](site_info ni) {
        return !(ni.site.owner == id_ && ni.site.owner != 0 && !assigned_move(ni.loc));
    });

    auto neutrals = get_neighbors(target, [this](site_info ni) { return !(ni.site.owner == 0); });

    if (enemies.empty() || allies.empty()) {
        return;
    }

    int enemy_power = std::accumulate(enemies.cbegin(), enemies.cend(), 0u,
                                      [this](unsigned int power, std::pair<direction_t, hlt::Location> enemy) {
                                          return power + map_.getSite(enemy.second).strength;
                                      });


    // coming up heads on to a enemy, let them sacrifice the strength to take the dividing territory
    if (neutrals.size() == 2) {

        for (const auto& ally : allies) {

            if (enemy_power > ti.site.strength) {

                auto ai = get_info(ally.second);
                auto destination = map_.getLocation(ally.second, reverse_direction(ally.first));

                LOGZ << "retreating for surprise (" << ally.second.x << "," << ally.second.y << ") into ("
                     << destination.x << "," << destination.y << ")" << std::endl;
                assign_move({ally.second, reverse_direction(ally.first)});
                ai.state.potential = 1337;
            } else {
                assign_move({ally.second, STILL});
            }
        }

        return;
    }

    // merge all allies into the single cell to reduce overkill
    if (allies.size() > 1 && ti.site.strength == 0) {

        for (const auto& ally : allies) {
            auto destination = map_.getLocation(ally.second, ally.first);
            LOGZ << "merging (" << ally.second.x << "," << ally.second.y << ") into (" << destination.x << ","
                 << destination.y << ")" << std::endl;
            assign_move({ally.second, ally.first});
        }
		return;
    }

	// ensure nobody moves into cells that are attacking
    if (enemies.size() == 1 && allies.size() == 1) {
        auto west = get_info(map_.getLocation(ti.loc, WEST));
        auto east = get_info(map_.getLocation(ti.loc, EAST));
        auto north = get_info(map_.getLocation(ti.loc, NORTH));
        auto south = get_info(map_.getLocation(ti.loc, SOUTH));

        if (west.site.owner == id_) {
			LOGZ << "poison west (" << west.loc.x << "," << west.loc.y << ")" << std::endl;
            west.state.potential = 1337;
        } else if (east.site.owner == id_) {
			LOGZ << "poison west (" << east.loc.x << "," << east.loc.y << ")" << std::endl;
            east.state.potential = 1337;
        } else if (north.site.owner == id_) {
			LOGZ << "poison west (" << north.loc.x << "," << north.loc.y << ")" << std::endl;
            north.state.potential = 1337;
        } else if (south.site.owner == id_) {
			LOGZ << "poison west (" << south.loc.x << "," << south.loc.y << ")" << std::endl;
            south.state.potential = 1337;
        }
    }

    // std::sort(allies.begin(), allies.end(), [this](const std::pair<direction_t, hlt::Location>& a,
    //                                                const std::pair<direction_t, hlt::Location>& b) {
    //     return map_.getSite(a.second).strength < map_.getSite(b.second).strength;
    // });

    // // leave our biggest to attack or chill, but we must retreat with the rest
    // auto biggest_ally = allies.back();
    // allies.pop_back();

    // // poison this tile so nobody moves into it
    // LOGZ << "poisoning wake (" << biggest_ally.second.x << "," << biggest_ally.second.y << ")" << std::endl;
    // get_state(biggest_ally.second).potential = 1337;

    // for (const auto& ally : allies) {

    //     LOGZ << "retreat ally (" << ally.second.x << "," << ally.second.y << ")" << std::endl;
    //     auto& state = get_state(ally.second);

    //     auto retreat_spots = get_neighbors(ally.second, [&](hlt::Location location) {
    //         const auto& retreat_site = map_.getSite(location);
    //         auto& retreat_state = get_state(ally.second);
    //         return !(retreat_site.owner == id_);
    //     });

    //     for (const auto& retreat_attempt : retreat_spots) {
    //         if (assign_move({ally.second, retreat_attempt.first})) {
    //             LOGZ << "retreating (" << ally.second.x << "," << ally.second.y << ") to ("
    //                  << map_.getLocation(ally.second, retreat_attempt.first).x << ","
    //                  << map_.getLocation(ally.second, retreat_attempt.first).y << ")" << std::endl;
    //             continue;
    //         }
    //     }
    //     // poison this tile so nobody moves into it
    //     LOGZ << "poisoning (" << ally.second.x << "," << ally.second.y << ")" << std::endl;
    //     state.potential = 1337;
    // }
}

// xxx: i think this might be better dynamic based on map size
bool zzbot::should_idle(const hlt::Site& site)
{
    return (site.strength < site.production * config_.production_move_scalar);
}

bool zzbot::assign_move(const hlt::Move& move, bool erase)
{
    auto current = get_info(move.loc);
    auto future = get_info(map_.getLocation(move.loc, move.dir));

    // this occurs due to reinforce being able to traverse through 0str cells that may or may not be ours
    if (current.site.owner != id_) {
        return false;
    }

    if (should_idle(current.site) && future.site.owner == id_) {
        LOGZ << "rejecting premature reinforce from (" << current.loc.x << "," << current.loc.y << ")" << std::endl;
        return false;
    }

    // reject illegal moves
    if (future.state.potential + current.site.strength > config_.wander_clobber_ceiling) {
        LOGZ << "rejecting clobber from (" << current.loc.x << "," << current.loc.y << ")" << std::endl;
        return false;
    }

    future.state.potential += current.site.strength;
    current.state.potential -= current.site.strength;

    if (orders_.find(current.loc) != orders_.end()) {
        LOGZ << "rejecting repeated order from (" << current.loc.x << "," << current.loc.y << ")" << std::endl;
        return false;
    }
    if (erase) {
        mine_.erase(current.loc);
    }

    orders_[current.loc] = move;
    return true;
}

void zzbot::do_wander()
{
    for (const auto& loc : mine_) {

        if (should_idle(map_.getSite(loc))) {
            continue;
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
                continue;
            }
        }
    }
}

float zzbot::score_region(const hlt::Location& location)
{
    float score = 0.0f;

    nearby_region(location, config_.score_region_radius, [&](site_info info) {

        float local_score = 0.0f;
        float distance = map_.getDistance(info.loc, location);
        float power = (std::max)(1.0f, (float)info.site.strength);

        if (info.site.owner == id_) {

            // const auto& orig_site = map_.getSite(loc);
            // if (orig_site.owner != id_) {
            //     local_score = (site.strength * 0.1f) / distance;
            //     // local_score = (float)(orig_site.production) / (std::max)(1.0f, (float)orig_site.strength);
            // }

        } else if (info.site.owner != 0) {

            // local_score = abs(state.potential) * (1.01f - 1.0f / (float)site.production);
            // const auto& player = game_state_.players[site.owner];
            // const auto& me = game_state_.players[id_];
            // local_score *= config_.score_enemy_scalar *
            //                ((std::max)(1.0f, (float)me.strength) / (std::max)(1.0f, (float)player.strength));

        } else {

            float military_value = 1.0f;

            if (info.state.border) {
                military_value = (std::max)(1.0f, (float)(abs(info.state.potential + info.site.strength)));
            }

            local_score = (military_value * (float)(info.site.production)) / power;
            local_score /= 1 + distance * distance * distance;
            score += local_score;
        }
    });

    return score;
}

void zzbot::nearby_region(const hlt::Location& loc, distance_t radius, range_fn fn)
{
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
            fn(get_info(runner));
        }

        l = map_.getLocation(l, SOUTH);
    }
}

void zzbot::range_all(range_fn fn)
{
    range_do(0, map_.height, 0, map_.width, fn);
}

void zzbot::range_do(distance_t y_start, distance_t y_end, distance_t x_start, distance_t x_end, range_fn fn)
{
    for (distance_t y = y_start; y < y_end; y++) {
        for (distance_t x = x_start; x < x_end; x++) {
            fn(get_info({x, y}));
        }
    }
}

std::vector<std::pair<direction_t, hlt::Location>> zzbot::get_neighbors(const hlt::Location& loc)
{
    std::vector<std::pair<direction_t, hlt::Location>> neighbors;

    for (direction_t dir : CARDINALS) {
        neighbors.emplace_back(reverse_direction(dir), map_.getLocation(loc, dir));
    }

    return neighbors;
}

std::vector<std::pair<direction_t, hlt::Location>> zzbot::get_neighbors(const hlt::Location& location,
                                                                        std::function<bool(site_info)> fn)
{
    auto neighbors = get_neighbors(location);

    if (get_state(location).score > 0) {
        LOGZ << "score  (" << location.x << "," << location.y << ")"
             << ": " << get_state(location).score << std::endl;
    }

    filter(neighbors, [&](std::pair<direction_t, hlt::Location> neighbor) { return fn(get_info(neighbor.second)); });

    return neighbors;
}

// don't forget the bug in hlt::getAngle either (y needs be negated in atan2)
direction_t zzbot::get_angle(const hlt::Location& from, const hlt::Location& to)
{
    float angle = map_.getAngle(from, to) * 180.0f / 3.14159265f;
    float roll = (float)(rand() % 90);
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
