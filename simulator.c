#include "simulator.h"

SimulationConfig* sim_config = NULL;
const char* CONFIG_FILE_NAME = "test_config.txt";

pthread_t* hub_threads = NULL;
pthread_t* drone_threads = NULL;
pthread_t* sender_threads = NULL;
pthread_t* receiver_threads = NULL;

pthread_mutex_t hub_info_mutex = PTHREAD_MUTEX_INITIALIZER;
int total_hubs_count;
int* hub_activity_registry = NULL; // int used as bo0l

pthread_mutex_t drone_info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sender_info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t receiver_info_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t* hub_incoming_storage_mutexes = NULL;
pthread_mutex_t* hub_outgoing_storage_mutexes = NULL;
pthread_mutex_t* hub_charging_spaces_mutexes = NULL;
int* incoming_storage_remaining = NULL;
PackageInfo*** incoming_storages = NULL; 
int* outgoing_storage_remaining = NULL;
PackageInfo*** outgoing_storages = NULL; 
int* charging_spaces_remaining = NULL;

// IMPORTANT NOTE: We need to lock this mutex before using WriteOutput() !!!
pthread_mutex_t debug_printf_mutex = PTHREAD_MUTEX_INITIALIZER;

#define DEBUG_MUTEX_LOCK if (DEBUG_SIMULATOR) { pthread_mutex_lock(&debug_printf_mutex); }
#define DEBUG_MUTEX_RELEASE if (DEBUG_SIMULATOR) { pthread_mutex_unlock(&debug_printf_mutex); }

void defer(void) {
    if (DEBUG_SIMULATOR)
        printf("[*] Called defer()\n");

    if (sim_config != NULL)
        free(sim_config);

    if (hub_threads != NULL)
        free(hub_threads);

    if (drone_threads != NULL)
        free(drone_threads);

    if (sender_threads != NULL)
        free(sender_threads);

    if (receiver_threads != NULL)
        free(receiver_threads);

    if (hub_incoming_storage_mutexes != NULL)
        free(hub_incoming_storage_mutexes);

    if (hub_outgoing_storage_mutexes != NULL)
        free(hub_outgoing_storage_mutexes);

    if (hub_charging_spaces_mutexes != NULL)
        free(hub_charging_spaces_mutexes);

    if (incoming_storage_remaining != NULL)
        free(incoming_storage_remaining);

    if (outgoing_storage_remaining != NULL)
        free(outgoing_storage_remaining);

    if (charging_spaces_remaining != NULL)
        free(charging_spaces_remaining);
    
    if (hub_activity_registry != NULL)
        free(hub_activity_registry);

    // TODO: Free "incoming_storages"
    // TODO: Free "outgoing_storages"

    if (DEBUG_SIMULATOR)
        printf("[*] defer(): Cleanup OK!\n");
}

void init_mutexes(int hub_count) {
    hub_incoming_storage_mutexes = (pthread_mutex_t*) malloc(sizeof(pthread_mutex_t) * hub_count);

    if (hub_incoming_storage_mutexes == NULL) {
        perror("malloc/hub_incoming_storage_mutexes");
        exit(1);
    }

    hub_outgoing_storage_mutexes = (pthread_mutex_t*) malloc(sizeof(pthread_mutex_t) * hub_count);

    if (hub_outgoing_storage_mutexes == NULL) {
        perror("malloc/hub_outgoing_storage_mutexes");
        exit(1);
    }

    hub_charging_spaces_mutexes = (pthread_mutex_t*) malloc(sizeof(pthread_mutex_t) * hub_count);

    if (hub_charging_spaces_mutexes == NULL) {
        perror("malloc/hub_charging_spaces_mutexes");
        exit(1);
    }

    for (int i = 0; i < hub_count; i++) {
        pthread_mutex_init(&hub_incoming_storage_mutexes[i], NULL);
        pthread_mutex_init(&hub_outgoing_storage_mutexes[i], NULL);
        pthread_mutex_init(&hub_charging_spaces_mutexes[i], NULL);
    }
}

void init_critical_structures(SimulationConfig* conf) {
    total_hubs_count = conf->hubs_count;

    incoming_storage_remaining = (int*) malloc(sizeof(int) * conf->hubs_count);
    
    if (incoming_storage_remaining == NULL) {
        perror("malloc/incoming_storage_remaining");
        exit(1);
    }
    
    outgoing_storage_remaining = (int*) malloc(sizeof(int) * conf->hubs_count);
    
    if (outgoing_storage_remaining == NULL) {
        perror("malloc/outgoing_storage_remaining");
        exit(1);
    }
    
    charging_spaces_remaining = (int*) malloc(sizeof(int) * conf->hubs_count);

    if (charging_spaces_remaining == NULL) {
        perror("malloc/charging_spaces_remaining");
        exit(1);
    }

    hub_activity_registry = (int*) malloc(sizeof(int) * conf->hubs_count);

    if (hub_activity_registry == NULL) {
        perror("malloc/hub_activity_registry");
        exit(1);
    }

    incoming_storages = (PackageInfo***) malloc(sizeof(PackageInfo**) * conf->hubs_count);

    if (incoming_storages == NULL) {
        perror("malloc/incoming_storages");
        exit(1);
    }

    outgoing_storages = (PackageInfo***) malloc(sizeof(PackageInfo**) * conf->hubs_count);

    if (outgoing_storages == NULL) {
        perror("malloc/outgoing_storages");
        exit(1);
    }

    for (int i = 0; i < conf->hubs_count; i++) {
        incoming_storage_remaining[i] = conf->hubs[i].incoming_storge_size;
        outgoing_storage_remaining[i] = conf->hubs[i].outgoing_storge_size;
        charging_spaces_remaining[i] = conf->hubs[i].charging_space_count;
        hub_activity_registry[i] = 1;

        incoming_storages[i] = (PackageInfo**) malloc(sizeof(PackageInfo*) * conf->hubs[i].incoming_storge_size);

        if (incoming_storages[i] == NULL) {
            perror("malloc/incoming_storages[i]");
            exit(1);
        }

        outgoing_storages[i] = (PackageInfo**) malloc(sizeof(PackageInfo*) * conf->hubs[i].outgoing_storge_size);
    
        if (outgoing_storages[i] == NULL) {
            perror("malloc/outgoing_storages[i]");
            exit(1);
        }
    }
}

void sender_thread(void* sender_info, void *sim_config) {
    SenderInfo* self = (SenderInfo*) sender_info;
    SimulationConfig* simulation_config = (SimulationConfig*) sim_config;

    if (DEBUG_SIMULATOR) {
        pthread_mutex_lock(&debug_printf_mutex);
        printf("Sender Thread awake (id: %d, hub: %d, packages: %d)\n", self->id, self->current_hub_id, self->remaining_package_count);
        pthread_mutex_unlock(&debug_printf_mutex);
    }    

    DEBUG_MUTEX_LOCK
    WriteOutput(self, NULL, NULL, NULL, SENDER_CREATED);
    DEBUG_MUTEX_RELEASE

    while (self->remaining_package_count > 0) {
        int active_hubs_count = 0;
        pthread_mutex_lock(&hub_info_mutex);

        for (int i = 0; i < total_hubs_count; i++)
            if (hub_activity_registry[i])
                active_hubs_count++;

        pthread_mutex_unlock(&hub_info_mutex);

        srand(time(NULL));
        
        int hub_index = rand() % active_hubs_count;
        int hub_id = -1;

        for (int i = 0; i < total_hubs_count; i++) 
            if (hub_activity_registry[i]) {
                if (hub_index == 0) {
                    hub_id = i + 1;
                    break;
                } else {
                    hub_index--;
                }
            }

        if (hub_id == -1) {
            perror("unexpected: hub_id = -1");
            exit(1);
        }

        // TODO: Here
    }
}

int main(int argc, char **argv, char **envp) {
    atexit(defer);

    FILE* config_fp;

    if (CONFIG_FILE_NAME == NULL)
        config_fp = stdin;
    else {
        config_fp = fopen(CONFIG_FILE_NAME, "r");

        if (config_fp == NULL) {
            perror("fopen");
            exit(1);
        }
    }

    sim_config = parse_config_from_file(config_fp);

    if (CONFIG_FILE_NAME != NULL)
        fclose(config_fp);

    if (sim_config == NULL) {
        perror("parse_config_from_file");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        dump_config(sim_config);
    
    init_critical_structures(sim_config);
    init_mutexes(sim_config->hubs_count);

    exit(0);
}