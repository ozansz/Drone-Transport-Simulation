#ifndef __CENG_CONFIG_PARSER_H
#define __CENG_CONFIG_PARSER_H

#include <stdio.h>
#include <stdlib.h>

typedef struct {
    int hub_id;
    int sender_id;
    int total_packages;
    int wait_time_between_packages;
} SenderConfig;

typedef struct {
    int hub_id;
    int receiver_id;
    int wait_time_between_packages;
} ReceiverConfig;

typedef struct {
    int hub_id;
    int incoming_storge_size;
    int outgoing_storge_size;
    int charging_space_count;
    int* distance_to_other_hubs;
    int* nearest_other_hubs_sorted;
    SenderConfig sender;
    ReceiverConfig receiver;
} HubConfig;

typedef struct {
    int drone_id;
    int travel_speed;
    int maximum_range;
    int starting_hub_id;
} DroneConfig;

typedef struct {
    HubConfig* hubs;
    DroneConfig* drones;
    int hubs_count;
    int drones_count;
} SimulationConfig;

SimulationConfig* parse_config_from_file(FILE* fp);
void dump_config(FILE* fp, SimulationConfig* conf);

#endif