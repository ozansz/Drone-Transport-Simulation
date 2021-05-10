#include "simulator.h"

const char* CONFIG_FILE_NAME = "test_config.txt";

int main(int argc, char **argv, char **envp) {
    FILE* config_fp;

    if (CONFIG_FILE_NAME == NULL)
        config_fp = stdin;
    else {
        config_fp = fopen(CONFIG_FILE_NAME, "r");

        if (config_fp == NULL) {
            perror("fopen");
            return 1;
        }
    }

    SimulationConfig* sim_confg = parse_config_from_file(config_fp);

    if (CONFIG_FILE_NAME != NULL)
        fclose(config_fp);

    if (sim_confg == NULL) {
        perror("parse_config_from_file");
        return 1;
    }

    if (DEBUG_SIMULATOR)
        dump_config(sim_confg);
    
    free(sim_confg);

    return 0;
}