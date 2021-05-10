#include "simulator.h"

int main(int argc, char **argv, char **envp) {
    FILE* config_fp = fopen("test_config.txt", "r");

    if (config_fp == NULL) {
        perror("fopen");
        return 1;
    }

    SimulationConfig* sim_confg = parse_config_from_file(config_fp);

    if (sim_confg == NULL) {
        perror("parse_config_from_file");
        return 1;
    }

    

    fclose(config_fp);
    free(sim_confg);

    return 0;
}