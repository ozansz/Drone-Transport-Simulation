#ifndef __CENG_SIMULATOR_H
#define __CENG_SIMULATOR_H

#include <time.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>

#include "config_parser.h"

#include "writeOutput.h"
#include "helper.h"

#define DEBUG_SIMULATOR 1
#define SUPERLOGGING 0

typedef struct {
    SenderInfo* self;
    SenderConfig* self_config;
    SimulationConfig* simulation_config;
} SenderThreadConfig;

typedef struct {
    ReceiverInfo* self;
    ReceiverConfig* self_config;
    SimulationConfig* simulation_config;
} ReceiverThreadConfig;

typedef struct {
    HubInfo* self;
    HubConfig* self_config;
    SimulationConfig* simulation_config;
} HubThreadConfig;

typedef struct {
    DroneInfo* self;
    DroneConfig* self_config;
    SimulationConfig* simulation_config;
} DroneThreadConfig;

#endif