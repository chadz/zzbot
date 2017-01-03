#include "zzbot.h"

#include <algorithm>
#include <cassert>
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

direction_t zzbot::get_direction(const hlt::Location& from, const hlt::Location& to, bool wait)
{
    auto path = get_path(from, to);
    if (path.empty() || path.size() == 1) return STILL;

    LOGZ << "path: ";

    int power = 0;
    int depth = 0;
    for (auto p : path) {
        const auto& site = map_.getSite(p);
        if (site.owner == id_) {
            power += site.strength + site.production * depth;
        }
        LOGZ << "(" << p.x << ", " << p.y << ") -> ";
    }

    LOGZ << "power: " << power << std::endl;

    path.pop_front();

    auto next = path.front();

    const auto& to_site = map_.getSite(to);
    if (to_site.owner != id_ && wait && power <= to_site.strength) {
        return STILL;
    }

    for (auto dir : CARDINALS) {
        if (map_.getLocation(from, dir) == next) {
            return dir;
        }
    }
    return STILL;
}

std::list<hlt::Location> zzbot::get_path(const hlt::Location& from, const hlt::Location& to)
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
            return ni.loc == to ||
                   !ni.state.poisoned && ni.state.potential < config_.wander_clobber_ceiling &&
                       (ni.site.owner == id_ || ni.site.strength == 0);
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

std::list<hlt::Location> zzbot::reconstruct_path(std::map<hlt::Location, hlt::Location>& came_from,
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

void zzbot::poison_area(const hlt::Location& location)
{
    get_state(location).poisoned = true;
    for (auto dir : CARDINALS) {
        get_state(map_.getLocation(location, dir)).poisoned = true;
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

            info.state.potential -= info.site.strength + info.site.production;

            auto enemies =
                get_neighbors(info.loc, [this](site_info ni) { return ni.site.owner != id_ && ni.site.owner != 0; });
            auto others = get_neighbors(info.loc, [this](site_info ni) { return ni.site.owner == 0; });
            auto allies = get_neighbors(info.loc, [this](site_info ni) { return ni.site.owner == id_; });

            unsigned int allied_power{};
            for (const auto& ally : allies) {
                auto ai = get_info(ally.second);
                allied_power += ai.site.strength;
            }

            // double the value of cells that we can wipe out
            for (const auto& enemy : enemies) {
                auto ei = get_info(enemy.second);
                int factor = (int)allied_power > (ei.site.strength + ei.site.production) ? 2 : 1;
                info.state.potential -= factor * (ei.site.strength + ei.site.production);
            }

            if (!allies.empty()) {
                info.state.border = true;
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
            assert(map_.inBounds(info.loc));
            mine_.emplace(info.loc);
        }
    });

    const auto& neutral = game_state_.players[0];
    auto population = (std::max)((float)neutral.population, 1.0f);
    game_state_.avg_prod = (float)neutral.production / population;
    game_state_.avg_str = (float)neutral.strength / population;

    range_all([&](site_info info) { info.state.score = score_region(info.loc); });

    std::sort(enemies_.begin(), enemies_.end(), [this](const hlt::Location& a, const hlt::Location& b) {
        return get_state(a).score > get_state(b).score;
    });

    // amplify the top 20%
    for (int i = 0; i < (std::max)(1, (int)enemies_.size() / 6); ++i) {
        LOGZ << "amplyifing (" << enemies_[i].x << "," << enemies_[i].y << ")" << std::endl;
        get_state(enemies_[i]).score *= 4;
    }

    // std::vector<hlt::Location> areas;
    // for (const auto& enemy : enemies_) {

    //    bool too_close = std::any_of(areas.begin(), areas.end(), [&](hlt::Location l) {
    //        auto distance = map_.getDistance(l, enemy);
    //        return distance < 10;
    //    });

    //    if (!too_close) {
    //        LOGZ << "amplyifing (" << enemy.x << "," << enemy.y << ")" << std::endl;
    //        areas.push_back(enemy);
    //        get_state(enemy).score *= 10;
    //    }
    //}
}

void zzbot::behavior()
{
    calc_state();

    float avg = std::accumulate(enemies_.begin(), enemies_.end(), 0.0f,
                                [this](float score, hlt::Location loc) { return score += get_state(loc).score; }) /
                enemies_.size();

    do_wander();
    do_attack(avg);
}

void zzbot::run()
{
    std::set<hlt::Move> moves;

    for (game_state_.frame = 0;; game_state_.frame++) {

        LOGZ << "frame: " << game_state_.frame << " as: " << game_state_.avg_str << " ap: " << game_state_.avg_prod
             << " asap: " << game_state_.avg_prod / game_state_.avg_str << std::endl;

        for (const auto player : game_state_.players) {
            LOGZ << " player: " << player.first << " strength: " << player.second.strength
                 << " production: " << player.second.production << " population: " << player.second.population
                 << std::endl;
        }
        LOGZ << "================================================================ " << std::endl;

        moves.clear();
        orders_.clear();

        getFrame(map_);

        if (game_state_.frame == 0) {
            orig_map_ = map_.contents;
        }

        behavior();

        for (const auto& order : orders_) {
            moves.insert(order.second);
        }

        sendFrame(moves);
    }
}

bool zzbot::assigned_move(const hlt::Location& loc) const
{
    return orders_.find(loc) != orders_.end();
}

void zzbot::do_attack(float avg)
{
    for (const auto& target : enemies_) {
        do_try_tactics(target);
        do_try_expand(target);
    }
}

void zzbot::do_try_expand(hlt::Location target)
{
    const auto& target_site = map_.getSite(target);
    auto ti = get_info(target);

    if (ti.state.poisoned) {
        return;
    }

    auto attackers =
        get_neighbors(target, [this](site_info ni) { return ni.site.owner == id_ && !assigned_move(ni.loc); });

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

        if (total_power > target_site.strength) {

            assign_move({attacker_loc, attacker.first}, false);

            // // abort once we commited a single guy to a zero str node
            // if (target_site.strength == 0) {
            //     break;
            // }
        } else {
            assign_move({attacker_loc, STILL}, false);
        }
    }
}

void zzbot::do_try_tactics(hlt::Location target)
{
    auto ti = get_info(target);

    auto enemies = get_neighbors(target, [this](site_info ni) { return ni.site.owner != id_ && ni.site.owner != 0; });

    auto allies = get_neighbors(
        target, [this](site_info ni) { return ni.site.owner == id_ && ni.site.owner != 0 && !assigned_move(ni.loc); });

    auto neutrals = get_neighbors(target, [this](site_info ni) { return ni.site.owner == 0; });

    if (enemies.empty() || allies.empty()) {
        return;
    }

    LOGZ << "special tactics at (" << target.x << "," << target.y << ")" << std::endl;

    int enemy_power = std::accumulate(enemies.cbegin(), enemies.cend(), 0u,
                                      [this](unsigned int power, std::pair<direction_t, hlt::Location> enemy) {
                                          return power + map_.getSite(enemy.second).strength;
                                      });

    int allies_power = std::accumulate(allies.cbegin(), allies.cend(), 0u,
                                       [this](unsigned int power, std::pair<direction_t, hlt::Location> ally) {
                                           return power + map_.getSite(ally.second).strength;
                                       });

    // coming up heads on to a enemy, let them sacrifice the strength to take the dividing territory
    if (neutrals.size() == 2 && ti.site.strength > 0 /*&& are_facing(enemies.front().second, allies.front().second)*/) {
        for (const auto& ally : allies) {

            if (enemy_power > ti.site.strength) {

                auto ai = get_info(ally.second);
                auto destination = map_.getLocation(ally.second, reverse_direction(ally.first));

                LOGZ << "retreating for surprise (" << ally.second.x << "," << ally.second.y << ") into ("
                     << destination.x << "," << destination.y << ")" << std::endl;
                force_move({ally.second, reverse_direction(ally.first)});
            } else {
                force_move({ally.second, STILL});
            }
        }

        return;
    }

    if (ti.site.strength == 0) {

        std::sort(allies.begin(), allies.end(), [this](const std::pair<direction_t, hlt::Location>& a,
                                                       const std::pair<direction_t, hlt::Location>& b) {
            return map_.getSite(a.second).strength > map_.getSite(b.second).strength;
        });

        unsigned int commited_power{};
        for (const auto& ally : allies) {
            auto ai = get_info(ally.second);
            commited_power += ai.site.strength;

            // don't clobber ourselves intentionally when we're fighting. this will open this piece up to other moves
            // maybe. perhaps i need to determine if another move exists here, then retreat otherwise
            /*          if (commited_power > 255) {
                          continue;
                      }*/

            auto destination = map_.getLocation(ally.second, ally.first);
            LOGZ << "merging (" << ally.second.x << "," << ally.second.y << ") into (" << destination.x << ","
                 << destination.y << ")" << std::endl;
            force_move({ally.second, ally.first});
            poison_area(ti.loc);
        }

        return;
    }

    //  //ensure nobody else moves into cells that are attacking
    // auto poison_cell = [this](const hlt::Location& loc, direction_t dir) {
    //     auto a = get_info(map_.getLocation(loc, dir));
    //     auto b = get_info(map_.getLocation(loc, reverse_direction(dir)));

    //     if (a.site.owner != b.site.owner && a.site.owner != 0 && b.site.owner != 0) {
    // poison_area(loc);
    //     }
    // };

    // poison_area(ti.loc);
}

// xxx: i think this might be better dynamic based on map size
bool zzbot::should_idle(const hlt::Site& site) const
{
    return (site.strength < site.production * config_.production_move_scalar);
}

bool zzbot::assign_move(const hlt::Move& move, bool force)
{
    auto current = get_info(move.loc);
    auto future = get_info(map_.getLocation(move.loc, move.dir));

    if (assigned_move(current.loc)) {
        LOGZ << "rejecting repeated order from (" << current.loc.x << "," << current.loc.y << ")" << std::endl;
        return false;
    }

    if (current.site.strength == 0) {
        return false;
    }
    // this occurs due to reinforce being able to traverse through 0str cells that may or may not be ours
    if (current.site.owner != id_) {
        LOGZ << "rejecting move from unowned cell (" << current.loc.x << "," << current.loc.y << ")" << std::endl;
        return false;
    }

    if (!force) {
        if (future.state.poisoned) {
            LOGZ << "rejecting moving into poisoned cell (" << current.loc.x << "," << current.loc.y << ")"
                 << std::endl;
            return false;
        }

        if (should_idle(current.site) && future.site.owner == id_) {
            LOGZ << "rejecting premature reinforce from (" << current.loc.x << "," << current.loc.y << ")" << std::endl;
            return false;
        }

        if (future.state.potential + current.site.strength > config_.wander_clobber_ceiling) {
            LOGZ << "rejecting clobber from (" << current.loc.x << "," << current.loc.y << ")" << std::endl;
            return false;
        }
    }

    future.state.potential += current.site.strength;
    current.state.potential -= current.site.strength;

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
        float best_distance = 0.0f;
        for (const auto& enemy : enemies_) {
            auto distance = map_.getDistance(loc, enemy);

            if (distance > (std::max)(map_.width, map_.height) / 3) continue;

            auto enemy_score = get_state(enemy).score / distance;

            if (enemy_score > best_score) {
                best_score = enemy_score;
                best_loc = enemy;
                best_distance = distance;
            }
        }

        LOGZ << "wandering from (" << loc.x << "," << loc.y << ") to (" << best_loc.x << "," << best_loc.y << ")"
             << " best score: " << best_score << " distance:" << best_distance << std::endl;

        auto direction = get_direction(loc, best_loc);
        auto site = map_.getSite(loc, direction);

        if (site.owner == id_) {
            assign_move({loc, direction}, false);
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

            // military grading
            if (info.site.strength == 0) {
                military_value = info.site.production + (std::max)(1.0f, (float)(abs(info.state.potential)));
            }

            if (game_state_.frame > 0 && false) {
                auto orig_site = orig_map_[info.loc.y][info.loc.x];
                auto orig_score = orig_site.production / (std::max)(1.0f, (float)orig_site.strength);
                local_score = orig_score;
            } else {
                local_score = info.site.production / (std::max)(1.0f, (float)info.site.strength);
            }

            // scale value based on our current strength; valuable early when we dont' want to wait forever for a
            // "better" spot.
            if (game_state_.frame < 10) {
                local_score /= (std::max)(1.0f, (float)info.site.strength / (float)game_state_.players[id_].strength);
            }

            if (info.site.strength == 0) {
                local_score = military_value / local_score;
            }

            local_score /= 1 + pow(distance, 2);

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
    return get_neighbors(loc, [](site_info) { return true; });
}

std::vector<std::pair<direction_t, hlt::Location>> zzbot::get_neighbors(const hlt::Location& location,
                                                                        std::function<bool(site_info)> fn)
{

    std::vector<std::pair<direction_t, hlt::Location>> neighbors;

    for (direction_t dir : CARDINALS) {
        auto loc = map_.getLocation(location, dir);
        if (fn(get_info(loc))) {
            neighbors.emplace_back(reverse_direction(dir), loc);
        }
    }

    return neighbors;
}