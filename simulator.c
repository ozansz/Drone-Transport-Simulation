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
pthread_mutex_t receiver_info_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t sender_info_mutex = PTHREAD_MUTEX_INITIALIZER;
int* sender_activity_registry = NULL; // int used as bo0l

pthread_mutex_t* hub_incoming_storage_mutexes = NULL;
pthread_mutex_t* hub_outgoing_storage_mutexes = NULL;
pthread_mutex_t* hub_charging_spaces_mutexes = NULL;
int* incoming_storage_remaining = NULL;
PackageInfo*** incoming_storages = NULL; 
int* outgoing_storage_remaining = NULL; // hub look at (outgoing_storage_remaining < self.outgoing_capacity)
PackageInfo*** outgoing_storages = NULL; 
int* charging_spaces_remaining = NULL;

// IMPORTANT NOTE: We need to lock this mutex before using WriteOutput() !!!
pthread_mutex_t debug_printf_mutex = PTHREAD_MUTEX_INITIALIZER;

#define DEBUG_MUTEX_LOCK if (DEBUG_SIMULATOR) { pthread_mutex_lock(&debug_printf_mutex); }
#define DEBUG_MUTEX_RELEASE if (DEBUG_SIMULATOR) { pthread_mutex_unlock(&debug_printf_mutex); }
#define DEBUG_LOG_SAFE(x) if (DEBUG_SIMULATOR) { pthread_mutex_lock(&debug_printf_mutex); x; pthread_mutex_unlock(&debug_printf_mutex); }

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

    if (sender_activity_registry != NULL)
        free(sender_activity_registry);

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

    sender_activity_registry = (int*) malloc(sizeof(int) * conf->hubs_count);

    if (sender_activity_registry == NULL) {
        perror("malloc/sender_activity_registry");
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
        sender_activity_registry[i] = 1;

        incoming_storages[i] = (PackageInfo**) malloc(sizeof(PackageInfo*) * conf->hubs[i].incoming_storge_size);

        if (incoming_storages[i] == NULL) {
            perror("malloc/incoming_storages[i]");
            exit(1);
        }

        for (int j = 0; j < conf->hubs[i].incoming_storge_size; j++)
            incoming_storages[i][j] = NULL;

        outgoing_storages[i] = (PackageInfo**) malloc(sizeof(PackageInfo*) * conf->hubs[i].outgoing_storge_size);
    
        if (outgoing_storages[i] == NULL) {
            perror("malloc/outgoing_storages[i]");
            exit(1);
        }

        for (int j = 0; j < conf->hubs[i].outgoing_storge_size; j++)
            outgoing_storages[i][j] = NULL;
    }
}

void sender_thread(void* sender_info, void *sim_config, void* self_conf) {
    PackageInfo* new_package = NULL;
    SenderInfo* self = (SenderInfo*) sender_info;
    SenderConfig* self_config = (SenderConfig*) self_conf;
    SimulationConfig* simulation_config = (SimulationConfig*) sim_config;

    DEBUG_LOG_SAFE(printf("Sender Thread awake (id: %d, hub: %d, packages: %d)\n", self->id, self->current_hub_id, self->remaining_package_count))

    // if (DEBUG_SIMULATOR) {
    //     pthread_mutex_lock(&debug_printf_mutex);
    //     printf("Sender Thread awake (id: %d, hub: %d, packages: %d)\n", self->id, self->current_hub_id, self->remaining_package_count);
    //     pthread_mutex_unlock(&debug_printf_mutex);
    // }    

    DEBUG_LOG_SAFE(WriteOutput(self, NULL, NULL, NULL, SENDER_CREATED))

    // DEBUG_MUTEX_LOCK
    // WriteOutput(self, NULL, NULL, NULL, SENDER_CREATED);
    // DEBUG_MUTEX_RELEASE

    for (int _packet_index = 0; self->remaining_package_count > 0; _packet_index++, self->remaining_package_count--) {
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

        int receiver_id = simulation_config->hubs[hub_id-1].receiver.receiver_id;

        DEBUG_LOG_SAFE(printf("Sender Thread %d: Selected Hub-%d:Rec-%d for package #%d\n", self->id, hub_id, receiver_id, _packet_index))
        
        if (new_package != NULL)
            free(new_package);

        new_package = (PackageInfo*) malloc(sizeof(PackageInfo));

        if (new_package == NULL) {
            perror("malloc/new_package");
            exit(1);
        }

        FillPacketInfo(new_package, self->id, self->current_hub_id, receiver_id, hub_id);
        FillSenderInfo(self, self->id, self->current_hub_id, self->remaining_package_count, new_package);

        // OLD TODO:
        // put package to self's hub's array on "outgoing_storages"
        // TODO: Change this to pthread_cond_t
        while (1) {
            pthread_mutex_lock(&hub_outgoing_storage_mutexes[self->current_hub_id-1]); // WaitCanDeposit
            
            if (outgoing_storage_remaining[self->current_hub_id-1] > 0) {
                int new_package_index_in_store = simulation_config->hubs[self->current_hub_id-1].outgoing_storge_size - outgoing_storage_remaining[hub_id-1];
                
                outgoing_storages[self->current_hub_id-1][new_package_index_in_store] = new_package; // SenderDeposit
                outgoing_storage_remaining[self->current_hub_id-1] -= 1;
                
                pthread_mutex_unlock(&hub_outgoing_storage_mutexes[self->current_hub_id-1]);
                
                break;
            }

            pthread_mutex_unlock(&hub_outgoing_storage_mutexes[self->current_hub_id-1]);
        }

        // FillPacketInfo && FillSenderInfo && WriteOutput
        DEBUG_LOG_SAFE(WriteOutput(self, NULL, NULL, NULL, SENDER_DEPOSITED))

        // sleep()
        // TODO: Change this to: wait()
        _wait(self_config->wait_time_between_packages);
    }

    pthread_mutex_lock(&sender_info_mutex);
    sender_activity_registry[self->id-1] = 0;
    pthread_mutex_unlock(&sender_info_mutex);

    FillSenderInfo(self, self->id, self->current_hub_id, self->remaining_package_count, NULL);
    DEBUG_LOG_SAFE(WriteOutput(self, NULL, NULL, NULL, SENDER_STOPPED))
}

void receiver_thread(void* receiver_info, void *sim_config, void* self_conf) {
    ReceiverInfo* self = (ReceiverInfo*) receiver_info;
    ReceiverConfig* self_config = (ReceiverConfig*) self_conf;
    SimulationConfig* simulation_config = (SimulationConfig*) sim_config;

    DEBUG_LOG_SAFE(printf("Receiver Thread awake (id: %d, hub: %d)\n", self->id, self->current_hub_id))
    DEBUG_LOG_SAFE(WriteOutput(NULL, self, NULL, NULL, RECEIVER_CREATED))

    while (1) {
        pthread_mutex_lock(&hub_info_mutex); // while CurrentHub is active do

        if (!hub_activity_registry[self->current_hub_id-1]) {
            pthread_mutex_unlock(&hub_info_mutex);
            break;
        }

        pthread_mutex_unlock(&hub_info_mutex);
        
        pthread_mutex_lock(&hub_incoming_storage_mutexes[self->current_hub_id-1]);

        for (int package_index = 0; package_index < simulation_config->hubs[self->current_hub_id-1].incoming_storge_size; package_index++)
            if (incoming_storages[self->current_hub_id-1][package_index] != NULL)
                if (incoming_storages[self->current_hub_id-1][package_index]->receiver_id == self->id) {
                    PackageInfo* incoming_package = incoming_storages[self->current_hub_id-1][package_index];
                    incoming_storages[self->current_hub_id-1][package_index] = NULL;

                    FillPacketInfo(incoming_package, incoming_package->sender_id, incoming_package->sending_hub_id, incoming_package->receiver_id, self->current_hub_id);
                    FillReceiverInfo(self, self->id, self->current_hub_id, incoming_package);
                    DEBUG_LOG_SAFE(WriteOutput(NULL, self, NULL, NULL, RECEIVER_PICKUP))

                    // TODO: Do this
                    // BUT: Be careful for double-freeing
                    // free(incoming_storages[self->current_hub_id-1][package_index])

                    // TODO: Change this to: wait()
                    _wait(self_config->wait_time_between_packages);
                }

        pthread_mutex_unlock(&hub_incoming_storage_mutexes[self->current_hub_id-1]);
    }

    FillReceiverInfo(self, self->id, self->current_hub_id, NULL);
    DEBUG_LOG_SAFE(WriteOutput(NULL, self, NULL, NULL, RECEIVER_STOPPED))
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