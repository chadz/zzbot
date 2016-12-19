#include "zzbot.h"

#include "hlt.hpp"
#include "networking.hpp"
#include <array>

int main(int argc, char** argv) {
    srand(time(NULL));
    std::cout.sync_with_stdio(0);

    if (argc == 2) {
        std::string names[] = {"a", "b", "c", "d"};

        int productions[] = {2, 5, 8, 10};
        float battle_factors[] = {0.75f, 1.0f, 1.25f, 1.50f};
        int radii[] = {1, 2, 4, 6};

        std::array<zzbot_config, 4> configs;

        for (int i = 0; i < 4; ++i) {
            configs[i].name = "zzbot_" + names[i];
            configs[i].production_move_scalar = productions[i];
            // configs[i].score_enemy_scalar = battle_factors[i];
            configs[i].score_region_radius = radii[i];
        }

        int bot_idx = atoi(argv[1]);

        zzbot bot(configs[bot_idx]);
        bot.run();
    } else {
        zzbot bot({});
        bot.run();
    }

    return 0;
}