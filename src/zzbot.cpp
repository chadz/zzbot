#include "zzbot.h"

#include <algorithm>
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
    LOGZ.close();
}

void zzbot::calc_state() {

    mine_.clear();
    enemies_.clear();
    neutral_.clear();

    state_ = std::vector<std::vector<site_state>>(map_.height, std::vector<site_state>(map_.width));

    range_all([&](const hlt::Location loc, const hlt::Site& site) {
        if (site.owner == 0) {
            neutral_.emplace_back(loc);
        } else if (site.owner != id_) {
            enemies_.emplace_back(loc);
        } else {
            mine_.emplace(loc);
        }

        state_[loc.y][loc.x].score = score_region(loc);
    });

    // rank spots such that first priority is given to attacking what is most valuable
    std::sort(neutral_.begin(), neutral_.end(), [this](const hlt::Location& a, const hlt::Location& b) {
        return state_[a.y][a.x].score > state_[b.y][b.x].score;
    });

    std::sort(enemies_.begin(), enemies_.end(), [this](const hlt::Location& a, const hlt::Location& b) {
        return state_[a.y][a.x].score > state_[b.y][b.x].score;
    });
}

void zzbot::behavior() {

    calc_state();

    do_attack(enemies_);
    do_attack(neutral_);

    for (const auto& mine : mine_) {
        auto move = do_wander(mine, map_.getSite(mine));

        if (move) {
            orders_[mine] = move.value();
        }
    }
}

void zzbot::run() {

    std::set<hlt::Move> moves;
    for (int frame = 0;; ++frame) {
        LOGZ << "frame: " << frame << std::endl;
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

    for (const auto& target : targets) {

        do_try_attack(target);
        do_try_reinforce(target);
    }
}

void zzbot::do_try_reinforce(hlt::Location target) {
    const auto& target_site = map_.getSite(target);

    auto attackers = get_neighbors(target, [this](hlt::Location location) {
        return !(map_.getSite(location).owner == id_ && !assigned_move(location));
    });

    auto total_power = std::accumulate(attackers.cbegin(), attackers.cend(), 0u,
                                       [this](unsigned int total, std::pair<direction_t, hlt::Location> neighbor) {
                                           return total + map_.getSite(neighbor.second).strength;
                                       });

    // sort by strength asc (maybe make this a param)
    std::sort(attackers.begin(), attackers.end(),
              [this](const std::pair<direction_t, hlt::Location>& a, const std::pair<direction_t, hlt::Location>& b) {
                  return map_.getSite(a.second).strength < map_.getSite(b.second).strength;
              });

    // cant directly attack now, try to borrow reinforcements
    if (target_site.owner == 0) {

        for (const auto& attacker : attackers) {

            auto attacker_loc = map_.getLocation(target, attacker.first);
            const auto& attacker_site = map_.getSite(attacker_loc);

            if (should_idle(attacker_site)) {
                continue;
            }

            auto supporters = get_neighbors(target, [this](hlt::Location location) {
                const auto& site = map_.getSite(location);
                return !(site.owner == id_ && !assigned_move(location) && !should_idle(site));
            });

            if (supporters.empty()) {
                continue;
            }

            LOGZ << "trying reinforce to beat (" << target.x << "," << target.y << ") score "
                 << state_[target.y][target.x].score << " from  (" << attacker_loc.x << "," << attacker_loc.y << ")"
                 << std::endl;

            unsigned int total_power = attacker_site.strength + attacker_site.production;
            unsigned int future_power = attacker_site.strength;
            attacker_site.production* config_.max_wait_for_attack;

            for (const auto& supporter : supporters) {
                const auto& site = map_.getSite(supporter.second);
                total_power += site.strength;
                future_power += site.strength + site.production * config_.max_wait_for_attack;
            }

            if (total_power > target_site.strength) {

                LOGZ << "can reinforce to beat (" << target.x << "," << target.y << ") score "
                     << state_[target.y][target.x].score << std::endl;

                for (const auto& supporter : supporters) {

                    auto supporter_loc = map_.getLocation(attacker_loc, supporter.first);
                    const auto& supporter_site = map_.getSite(supporter_loc);
                    auto& attacker_state = state_[attacker_loc.y][attacker_loc.x];

                    // send supporters to the attacker
                    if (supporter_site.strength + attacker_state.potential < config_.wander_clobber_ceiling) {
                        if (mine_.find(supporter_loc) != mine_.end()) {

                            LOGZ << "moving (" << supporter_loc.x << "," << supporter_loc.y << ")"
                                 << "to (" << attacker_loc.x << "," << attacker_loc.y << ")" << std::endl;

                            mine_.erase(supporter_loc);
                            orders_[supporter_loc] = {supporter_loc, reverse_direction(supporter.first)};
                            attacker_state.potential += supporter_site.strength;
                        }
                    }
                }

                // make sure our attacker awaits the supporters if going against neutral
                if (target_site.owner == 0 && !assigned_move(attacker_loc)) {

                    LOGZ << "awaiting support at (" << attacker_loc.x << "," << attacker_loc.y << ")" << std::endl;
                    mine_.erase(attacker_loc);
                    orders_[attacker_loc] = {attacker_loc, STILL};
                }

            } else if (future_power > target_site.strength) {

                for (const auto& supporter : supporters) {

                    auto supporter_loc = map_.getLocation(attacker_loc, supporter.first);
                    const auto& supporter_site = map_.getSite(supporter_loc);
                    auto& supporter_state = state_[supporter_loc.y][supporter_loc.x];

                    if (!assigned_move(supporter_loc)) {

                        LOGZ << "waiting to support (" << supporter_loc.x << "," << supporter_loc.y << ")"
                             << "to (" << attacker_loc.x << "," << attacker_loc.y << ")" << std::endl;

                        mine_.erase(supporter_loc);
                        orders_[supporter_loc] = {supporter_loc, STILL};
                        supporter_state.potential += supporter_site.strength;
                    }
                }

                if (target_site.owner == 0 && !assigned_move(attacker_loc)) {

                    LOGZ << "awaiting wait support at (" << attacker_loc.x << "," << attacker_loc.y << ")" << std::endl;
                    mine_.erase(attacker_loc);
                    orders_[attacker_loc] = {attacker_loc, STILL};
                }
            }
        }
    }
}

void zzbot::do_try_attack(hlt::Location target) {
    const auto& target_site = map_.getSite(target);

    auto attackers = get_neighbors(target, [this](hlt::Location location) {
        return !(map_.getSite(location).owner == id_ && !assigned_move(location));
    });

    auto total_power = std::accumulate(attackers.cbegin(), attackers.cend(), 0u,
                                       [this](unsigned int total, std::pair<direction_t, hlt::Location> neighbor) {
                                           return total + map_.getSite(neighbor.second).strength;
                                       });

    // sort by strength asc (maybe make this a param)
    std::sort(attackers.begin(), attackers.end(),
              [this](const std::pair<direction_t, hlt::Location>& a, const std::pair<direction_t, hlt::Location>& b) {
                  return map_.getSite(a.second).strength < map_.getSite(b.second).strength;
              });

    if ((target_site.owner != 0 && total_power >= target_site.strength) ||
        (target_site.owner == 0 && total_power > target_site.strength)) {

        unsigned int provided_power = 0;
        for (const auto& attacker : attackers) {

            auto attacker_loc = attacker.second;
            const auto& attacker_site = map_.getSite(attacker_loc);

            provided_power += attacker_site.strength;

            // don't overcommit an attack if it will forfeit strength. this needs to suck less
            if (provided_power > target_site.strength && (provided_power - target_site.strength) > 255) {
                continue;
            }

            if (!assigned_move(attacker_loc)) {

                LOGZ << "attacking from (" << attacker_loc.x << "," << attacker_loc.y << ","
                     << (int)attacker_site.strength << ")"
                     << "to (" << target.x << "," << target.y << "," << (int)target_site.strength
                     << ") combined power: " << total_power << std::endl;

                mine_.erase(attacker_loc);
                orders_[attacker_loc] = {attacker_loc, reverse_direction(attacker.first)};

                // abort once we commited a single guy to a zero str node
                if (target_site.strength == 0) {
                    break;
                }
            }
        }
    }
}

// xxx: i think this might be better dynamic based on map size
bool zzbot::should_idle(const hlt::Site& site) {
    return (site.strength < site.production * config_.production_move_scalar);
}

optional<hlt::Move> zzbot::do_wander(const hlt::Location loc, const hlt::Site& site) {

    if (!should_idle(site)) {

        auto borders = distance_to_borders(loc);

        // both planes are taken; just move randomly SW
        if (std::none_of(
                borders.cbegin(), borders.cend(),
                [](const optional<std::pair<hlt::Location, distance_t>>& border) { return border.has_value(); })) {

            direction_t random_dir = 3 + rand() & 1;
            return optional<hlt::Move>({loc, random_dir});
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
            auto border_score = state_[border_loc.y][border_loc.x].score;
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
        auto& proposed_state = state_[proposed_loc.y][proposed_loc.x];
        if (proposed_site.owner == id_ && site.strength + proposed_state.potential <= config_.wander_clobber_ceiling) {

            LOGZ << "wandering from (" << loc.x << "," << loc.y << ")"
                 << "to (" << proposed_loc.x << "," << proposed_loc.y << ")" << std::endl;

            proposed_state.potential += site.strength;
            return optional<hlt::Move>({loc, best_dir});
        }
    }

    /// if the dir wasnt coupled with the index i can try the next best moves
    /// instead of idling
    return optional<hlt::Move>();
}

// xxx:this is made me realize my lambdas are becoming boilerplate.
// should probably wrap regions into an object and provide begin/end so i can
// accum/do whatever over the iterators

float zzbot::score_region(const hlt::Location& loc) {

    float score = 0.0f;

    nearby_region(loc, config_.score_region_radius, [&](const hlt::Location l, const hlt::Site& site) {

        if (site.owner != id_) {

            float local_score = (float)site.production / (std::max)(1.0f, (float)site.strength);
            local_score /= 1 + map_.getDistance(l, loc);

            // lets be militaristic
            if (site.owner != 0) {
                local_score *= config_.score_enemy_scalar;
            }

            score += local_score;
        }

    });

    return score;
}

void zzbot::nearby_region(const hlt::Location& loc, distance_t radius, range_fn fn) {

    auto l = loc;

    // wind to 'top-left'
    for (distance_t r = 0; r < radius; ++r) {
        l = map_.getLocation(l, NORTH);
        l = map_.getLocation(l, WEST);
    }

    distance_t width = 2 * radius + 1;

    for (distance_t y = 0; y < width; y++) {
        auto runner = l;
        for (distance_t x = 0; x < width; x++) {
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
        neighbors.emplace_back(dir, map_.getLocation(loc, dir));
    }

    return neighbors;
}
