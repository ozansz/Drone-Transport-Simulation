#ifndef __CENG_SIMULATOR_H
#define __CENG_SIMULATOR_H

#include <time.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>

#include "config_parser.h"

#include "writeOutput.h"
#include "helper.h"

#define DEBUG_SIMULATOR 1

typedef struct {
    SenderInfo* self;
    SenderConfig* self_config;
    SimulationConfig* simulation_config;
} SenderThreadConfig;

#endif