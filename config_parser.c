#include "config_parser.h"

SimulationConfig* parse_config_from_file(FILE* fp) {
    SimulationConfig* config = (SimulationConfig*) malloc(sizeof(SimulationConfig));

    fscanf("%d\n", &config->hubs_count);

    config->hubs = (HubConfig*) malloc(sizeof(HubConfig) * config->hubs_count);

    for (int i = 0; i < config->hubs_count; i++) {
        config->hubs[i].hub_id = i + 1;
        config->hubs[i].distance_to_other_hubs = (int*) malloc(sizeof(int) * config->hubs_count);
        
        fscanf("%d %d %d ", &config->hubs[i].incoming_storge_size, &config->hubs[i].outgoing_storge_size, &config->hubs[i].charging_space_count);

        for (int j = 0; j < config->hubs_count; j++)
            if (j == (config->hubs_count - 1))
                fscanf("%d\n", &config->hubs[i].distance_to_other_hubs[j]);
            else
                fscanf("%d ", &config->hubs[i].distance_to_other_hubs[j]);
    }

    for (int i = 0; i < config->hubs_count; i++) {
        int time_wait;
        int hub_id;
        int packages;

        fscanf("%d %d %d\n", &time_wait, &hub_id, &packages);

        if (hub_id > config->hubs_count) {
            perror("sender hub id out of bounds!");
            return NULL;
        }

        config->hubs[hub_id-1].sender.sender_id = i + 1;
        config->hubs[hub_id-1].sender.total_packages = packages;
        config->hubs[hub_id-1].sender.wait_time_between_packages = time_wait;
    }

    for (int i = 0; i < config->hubs_count; i++) {
        int time_wait;
        int hub_id;

        fscanf("%d %d\n", &time_wait, &hub_id);

        if (hub_id > config->hubs_count) {
            perror("receiver hub id out of bounds!");
            return NULL;
        }

        config->hubs[hub_id-1].receiver.receiver_id = i + 1;
        config->hubs[hub_id-1].receiver.wait_time_between_packages = time_wait;
    }

    fscanf("%d\n", &config->drones_count);

    config->drones = (DroneConfig*) malloc(sizeof(DroneConfig) * config->drones_count);

    for (int i = 0; i < config->drones_count; i++) {
        config->drones[i].drone_id = i + 1;
        fscanf("%d %d %d\n", &config->drones[i].travel_speed, &config->drones[i].starting_hub_id, &config->drones[i].maximum_range);
    }

    return config;
}