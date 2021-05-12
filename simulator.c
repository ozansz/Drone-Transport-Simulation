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

typedef enum {
    DRONE_ON_PACKAGE_TRANSFER,
    DRONE_ON_SELF_TRAVEL,
    DRONE_IN_HUB
} DroneStatus;

typedef struct {
    DroneStatus stat;
    int hub_id;
    // PackageInfo* pkg;
    DroneInfo* info;
} DynamicDroneInfo;

pthread_mutex_t drone_info_mutex = PTHREAD_MUTEX_INITIALIZER;
DynamicDroneInfo** drone_info_registry = NULL;

pthread_mutex_t receiver_info_mutex = PTHREAD_MUTEX_INITIALIZER;

pthread_mutex_t sender_info_mutex = PTHREAD_MUTEX_INITIALIZER;
int* sender_activity_registry = NULL; // int used as bo0l

pthread_mutex_t* hub_incoming_storage_mutexes = NULL;
pthread_mutex_t* hub_outgoing_storage_mutexes = NULL;

// IMPORTANT: Drones MUST update (using this) charging_spaces_remaining and drone_info_registry.
pthread_mutex_t* hub_charging_spaces_mutexes = NULL;

int* incoming_storage_remaining = NULL;
PackageInfo*** incoming_storages = NULL; 
int* outgoing_storage_remaining = NULL; // hub look at (outgoing_storage_remaining < self.outgoing_capacity)
PackageInfo*** outgoing_storages = NULL; 
int* charging_spaces_remaining = NULL; // hub look at (charging_spaces_remaining < self.charging_capacity)

// IMPORTANT NOTE: We need to lock this mutex before using WriteOutput() !!!
pthread_mutex_t debug_printf_mutex = PTHREAD_MUTEX_INITIALIZER;

#define DEBUG_MUTEX_LOCK if (DEBUG_SIMULATOR) { pthread_mutex_lock(&debug_printf_mutex); }
#define DEBUG_MUTEX_RELEASE if (DEBUG_SIMULATOR) { pthread_mutex_unlock(&debug_printf_mutex); }
#define LOG_SAFE(x) if (DEBUG_SIMULATOR) { pthread_mutex_lock(&debug_printf_mutex); x; pthread_mutex_unlock(&debug_printf_mutex); } else { x; }
#define DEBUG_LOG_SAFE(x) if (DEBUG_SIMULATOR) { pthread_mutex_lock(&debug_printf_mutex); x; pthread_mutex_unlock(&debug_printf_mutex); }

void defer(void) {
    if (DEBUG_SIMULATOR)
        printf("\n[*] Called defer()\n");

    if (incoming_storages != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing incoming_storages\n");

        if ((sim_config != NULL) && (sim_config->hubs != NULL)) {
            for (int i = 0; i < sim_config->hubs_count; i++) {
                if (incoming_storages[i] != NULL) {
                    if (DEBUG_SIMULATOR)
                        printf("    ++ incoming_storages[%d]\n", i);

                    for (int j = 0; j < sim_config->hubs[i].incoming_storge_size; j++)
                        if (incoming_storages[i][j] != NULL)
                            free(incoming_storages[i][j]);

                    free(incoming_storages[i]);
                }
            }
        } else {
            printf("   >!! sim_config is NULL, just freeing incoming_storages.\n");
            printf("   >!! There may be data loss in heap.\n");
        }

        free(incoming_storages);
    }

    if (outgoing_storages != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing outgoing_storages\n");

        if ((sim_config != NULL) && (sim_config->hubs != NULL)) {
            for (int i = 0; i < sim_config->hubs_count; i++) {
                if (outgoing_storages[i] != NULL) {
                    if (DEBUG_SIMULATOR)
                        printf("    ++ outgoing_storages[%d]\n", i);

                    for (int j = 0; j < sim_config->hubs[i].outgoing_storge_size; j++)
                        if (outgoing_storages[i][j] != NULL)
                            free(outgoing_storages[i][j]);

                    free(outgoing_storages[i]);
                }
            }
        } else {
            printf("   >!! sim_config is NULL, just freeing outgoing_storages.\n");
            printf("   >!! There may be data loss in heap.\n");
        }

        free(outgoing_storages);
    }

    if (drone_info_registry != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing drone_info_registry\n");

        if ((sim_config != NULL) && (sim_config->drones != NULL)) {
            for (int i = 0; i < sim_config->drones_count; i++) {
                if (drone_info_registry[i] != NULL) {
                    if (DEBUG_SIMULATOR)
                        printf("    ++ drone_info_registry[%d]\n", i);

                    if (drone_info_registry[i]->info != NULL)
                        free(drone_info_registry[i]->info);

                    free(drone_info_registry[i]);
                }
            }
        } else {
            printf("   >!! sim_config is NULL, just freeing drone_info_registry.\n");
            printf("   >!! There may be data loss in heap.\n");
        }

        free(drone_info_registry);
    }

    if (sim_config != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing sim_config\n");

        if (sim_config->drones != NULL)
            free(sim_config->drones);

        if (sim_config->hubs != NULL) {
            for (int i = 0; i < sim_config->hubs_count; i++) {
                if (sim_config->hubs[i].distance_to_other_hubs != NULL)
                    free(sim_config->hubs[i].distance_to_other_hubs);

                if (sim_config->hubs[i].nearest_other_hubs_sorted != NULL)
                    free(sim_config->hubs[i].nearest_other_hubs_sorted);
            }

            free(sim_config->hubs);
        }

        free(sim_config);
    }

    if (hub_threads != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing hub_threads\n");
        
        free(hub_threads);
    }

    if (drone_threads != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing drone_threads\n");
        
        free(drone_threads);
    }

    if (sender_threads != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing sender_threads\n");

        free(sender_threads);
    }

    if (receiver_threads != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing receiver_threads\n");

        free(receiver_threads);
    }

    if (hub_incoming_storage_mutexes != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing hub_incoming_storage_mutexes\n");

        free(hub_incoming_storage_mutexes);
    }

    if (hub_outgoing_storage_mutexes != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing hub_outgoing_storage_mutexes\n");

        free(hub_outgoing_storage_mutexes);
    }

    if (hub_charging_spaces_mutexes != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing hub_charging_spaces_mutexes\n");

        free(hub_charging_spaces_mutexes);
    }

    if (incoming_storage_remaining != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing incoming_storage_remaining\n");

        free(incoming_storage_remaining);
    }

    if (outgoing_storage_remaining != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing outgoing_storage_remaining\n");

        free(outgoing_storage_remaining);
    }

    if (charging_spaces_remaining != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing charging_spaces_remaining\n");

        free(charging_spaces_remaining);
    }
    
    if (hub_activity_registry != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing hub_activity_registry\n");

        free(hub_activity_registry);
    }

    if (sender_activity_registry != NULL) {
        if (DEBUG_SIMULATOR)
            printf(" ++ Freeing sender_activity_registry\n");

        free(sender_activity_registry);
    }

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

    if (DEBUG_SIMULATOR)
        printf("[*] Allocating incoming_storage_remaining\n");

    incoming_storage_remaining = (int*) malloc(sizeof(int) * conf->hubs_count);
    
    if (incoming_storage_remaining == NULL) {
        perror("malloc/incoming_storage_remaining");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        printf("[*] Allocating outgoing_storage_remaining\n");
    
    outgoing_storage_remaining = (int*) malloc(sizeof(int) * conf->hubs_count);
    
    if (outgoing_storage_remaining == NULL) {
        perror("malloc/outgoing_storage_remaining");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        printf("[*] Allocating charging_spaces_remaining\n");
    
    charging_spaces_remaining = (int*) malloc(sizeof(int) * conf->hubs_count);

    if (charging_spaces_remaining == NULL) {
        perror("malloc/charging_spaces_remaining");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        printf("[*] Allocating hub_activity_registry\n");

    hub_activity_registry = (int*) malloc(sizeof(int) * conf->hubs_count);

    if (hub_activity_registry == NULL) {
        perror("malloc/hub_activity_registry");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        printf("[*] Allocating sender_activity_registry\n");

    sender_activity_registry = (int*) malloc(sizeof(int) * conf->hubs_count);

    if (sender_activity_registry == NULL) {
        perror("malloc/sender_activity_registry");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        printf("[*] Allocating incoming_storages\n");

    incoming_storages = (PackageInfo***) malloc(sizeof(PackageInfo**) * conf->hubs_count);

    if (incoming_storages == NULL) {
        perror("malloc/incoming_storages");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        printf("[*] Allocating outgoing_storages\n");

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

        if (DEBUG_SIMULATOR)
            printf("[*] Allocating incoming_storages[%d]\n", i);

        incoming_storages[i] = (PackageInfo**) malloc(sizeof(PackageInfo*) * conf->hubs[i].incoming_storge_size);

        if (incoming_storages[i] == NULL) {
            perror("malloc/incoming_storages[i]");
            exit(1);
        }

        if (DEBUG_SIMULATOR)
            printf("[*] Initializing incoming_storages[%d][<size:%d>]\n", i, conf->hubs[i].incoming_storge_size);

        for (int j = 0; j < conf->hubs[i].incoming_storge_size; j++)
            incoming_storages[i][j] = NULL;

        if (DEBUG_SIMULATOR)
            printf("[*] Allocating outgoing_storages[%d]\n", i);

        outgoing_storages[i] = (PackageInfo**) malloc(sizeof(PackageInfo*) * conf->hubs[i].outgoing_storge_size);
    
        if (outgoing_storages[i] == NULL) {
            perror("malloc/outgoing_storages[i]");
            exit(1);
        }

        if (DEBUG_SIMULATOR)
            printf("[*] Initializing outgoing_storages[%d][<size:%d>]\n", i, conf->hubs[i].outgoing_storge_size);

        for (int j = 0; j < conf->hubs[i].outgoing_storge_size; j++)
            outgoing_storages[i][j] = NULL;
    }

    if (DEBUG_SIMULATOR)
        printf("[*] Allocating drone_info_registry\n");

    drone_info_registry = (DynamicDroneInfo**) malloc(sizeof(DynamicDroneInfo*) * conf->drones_count);

    if (drone_info_registry == NULL) {
        perror("malloc/drone_info_registry");
        exit(1);
    }

    for (int i = 0; i < conf->drones_count; i++) {
        if (DEBUG_SIMULATOR)
            printf("[*] Allocating drone_info_registry[%d]\n", i);

        drone_info_registry[i] = (DynamicDroneInfo*) malloc(sizeof(DynamicDroneInfo));

        if (drone_info_registry[i] == NULL) {
            perror("malloc/drone_info_registry[i]");
            exit(1);
        }

        if (DEBUG_SIMULATOR)
            printf("[*] Initializing drone_info_registry[%d]\n", i);

        // TODO: Check this.
        drone_info_registry[i]->hub_id = conf->drones[i].starting_hub_id;
        
        // IMPORTANT: drone_info_registry[i]->info will be allocated and set inside "main" function.
        // drone_info_registry[i]->info->current_hub_id = conf->drones[i].starting_hub_id;
        // drone_info_registry[i]->info->current_range = conf->drones[i].maximum_range;
        // drone_info_registry[i]->info->id = conf->drones[i].drone_id;
        // drone_info_registry[i]->info->next_hub_id = -1;
        // drone_info_registry[i]->info->packageInfo = NULL;
        drone_info_registry[i]->info = NULL;

        drone_info_registry[i]->stat = DRONE_IN_HUB;
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

    LOG_SAFE(WriteOutput(self, NULL, NULL, NULL, SENDER_CREATED))

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
        LOG_SAFE(WriteOutput(self, NULL, NULL, NULL, SENDER_DEPOSITED))

        // sleep()
        // TODO: Change this to: wait()
        _wait(self_config->wait_time_between_packages);
    }

    pthread_mutex_lock(&sender_info_mutex);
    sender_activity_registry[self->id-1] = 0;
    pthread_mutex_unlock(&sender_info_mutex);

    FillSenderInfo(self, self->id, self->current_hub_id, self->remaining_package_count, NULL);
    LOG_SAFE(WriteOutput(self, NULL, NULL, NULL, SENDER_STOPPED))
}

void receiver_thread(void* receiver_info, void *sim_config, void* self_conf) {
    ReceiverInfo* self = (ReceiverInfo*) receiver_info;
    ReceiverConfig* self_config = (ReceiverConfig*) self_conf;
    SimulationConfig* simulation_config = (SimulationConfig*) sim_config;

    DEBUG_LOG_SAFE(printf("Receiver Thread awake (id: %d, hub: %d)\n", self->id, self->current_hub_id))
    LOG_SAFE(WriteOutput(NULL, self, NULL, NULL, RECEIVER_CREATED))

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
                    LOG_SAFE(WriteOutput(NULL, self, NULL, NULL, RECEIVER_PICKUP))

                    // TODO: Do this
                    // BUT: Be careful for double-freeing
                    // free(incoming_storages[self->current_hub_id-1][package_index])

                    // TODO: Change this to: wait()
                    _wait(self_config->wait_time_between_packages);
                }

        pthread_mutex_unlock(&hub_incoming_storage_mutexes[self->current_hub_id-1]);
    }

    FillReceiverInfo(self, self->id, self->current_hub_id, NULL);
    LOG_SAFE(WriteOutput(NULL, self, NULL, NULL, RECEIVER_STOPPED))
}

void hub_thread(void* hub_info, void *sim_config, void* self_conf) {
    HubInfo* self = (HubInfo*) hub_info;
    HubConfig* self_config = (HubConfig*) self_conf;
    SimulationConfig* simulation_config = (SimulationConfig*) sim_config;

    DEBUG_LOG_SAFE(printf("Hub Thread awake (id: %d)\n", self->id))
    LOG_SAFE(WriteOutput(NULL, NULL, NULL, self, HUB_CREATED))

    while (1) { // while there are active senders or packages in either storage do
        int active_senders = 0;
        pthread_mutex_lock(&sender_info_mutex);

        for (int i = 0; i < simulation_config->hubs_count; i++) {
            // TODO: Do we count this hub's sender too??
            // int sender_id = i + 1;
            // if (sender_id == self_config->sender.sender_id)
            //     continue;

            if (sender_activity_registry[i])
                active_senders++;
        }

        pthread_mutex_unlock(&sender_info_mutex);

        pthread_mutex_lock(&hub_incoming_storage_mutexes[self->id-1]);
        int incoming_packages = self_config->incoming_storge_size - incoming_storage_remaining[self->id-1];
        pthread_mutex_unlock(&hub_incoming_storage_mutexes[self->id-1]);

        pthread_mutex_lock(&hub_outgoing_storage_mutexes[self->id-1]);
        int outgoing_packages = self_config->outgoing_storge_size - outgoing_storage_remaining[self->id-1];
        pthread_mutex_lock(&hub_outgoing_storage_mutexes[self->id-1]);
    
        if ((active_senders == 0) && (incoming_packages == 0) && (outgoing_packages == 0))
            break;

        if (outgoing_packages > 0) { // WaitUntilPackageDeposited ()
            // Select the package
            pthread_mutex_lock(&hub_outgoing_storage_mutexes[self->id-1]);

            PackageInfo* package_will_sent = NULL;

            for (int i = 0; i < self_config->outgoing_storge_size; i++)
                if (outgoing_storages[self->id-1][i] != NULL) {
                    package_will_sent = outgoing_storages[self->id-1][i];
                    outgoing_storages[self->id-1][i] = NULL;
                    outgoing_storage_remaining[self->id-1]++;
                    break;
                }

            pthread_mutex_unlock(&hub_outgoing_storage_mutexes[self->id-1]);

            if (package_will_sent == NULL) {
                perror("unexpected: could not select package: package_will_sent");
                exit(1);
            }

// TODO: This may be done in a better way...
select_drones_loop:
            // Find drones...
            pthread_mutex_lock(&hub_charging_spaces_mutexes[self->id-1]);

            // Check if there are drones in the hub
            if (charging_spaces_remaining[self->id-1] < self_config->charging_space_count) {
                int drone_count_on_me = self_config->charging_space_count - charging_spaces_remaining[self->id-1];
                pthread_mutex_unlock(&hub_charging_spaces_mutexes[self->id-1]);

                DynamicDroneInfo** drones_on_me = malloc(sizeof(DynamicDroneInfo*) * drone_count_on_me);

                if (drones_on_me == NULL) {
                    perror("malloc/drones_on_me");
                    exit(1);
                }

                pthread_mutex_lock(&drone_info_mutex);

                for (int i = 0, dom_index = 0; i < simulation_config->drones_count; i++)
                    if ((drone_info_registry[i]->stat == DRONE_IN_HUB) && drone_info_registry[i]->hub_id == self->id)
                        drones_on_me[dom_index++] = drone_info_registry[i];

                pthread_mutex_unlock(&drone_info_mutex);

                DynamicDroneInfo* drone_with_highest_range_on_me = NULL;

                for (int i = 0; i < drone_count_on_me; i++)
                    if ((drone_with_highest_range_on_me == NULL) || (drones_on_me[i]->info->current_range > drone_with_highest_range_on_me->info->current_range))
                        drone_with_highest_range_on_me = drones_on_me[i];

                if (drone_with_highest_range_on_me == NULL) {
                    perror("unexpected: could not select drone: drone_with_highest_range_on_me");
                    exit(1);
                }

                free(drones_on_me);

                // assign the package to the drone
                // AssignAndNotifyDrone (Package, Drone)
                drone_with_highest_range_on_me->stat = DRONE_ON_PACKAGE_TRANSFER;
                drone_with_highest_range_on_me->info->packageInfo = package_will_sent;
            } else {
                pthread_mutex_unlock(&hub_charging_spaces_mutexes[self->id-1]);

                // TODO:
                // CallDroneFromHubs ()
                // if No drone is found in other hubs then
                //     WaitTimeoutOrDrone ()
                //     goto start
                // end

                DynamicDroneInfo* found_drone = NULL;

                for (int i = 0; i < total_hubs_count - 1; i++) { 
                    int other_hub_id = self_config->nearest_other_hubs_sorted[i];

                    pthread_mutex_lock(&hub_charging_spaces_mutexes[other_hub_id-1]);
                    int drones_in_neighbor_hub = simulation_config->hubs[other_hub_id-1].charging_space_count - charging_spaces_remaining[other_hub_id-1];
                    pthread_mutex_unlock(&hub_charging_spaces_mutexes[other_hub_id-1]);
                    
                    if (drones_in_neighbor_hub > 0) {
                        pthread_mutex_lock(&drone_info_mutex); 

                        for (int i = 0; i < simulation_config->drones_count; i++)
                            if ((drone_info_registry[i]->stat == DRONE_IN_HUB) && drone_info_registry[i]->hub_id == other_hub_id) {
                                found_drone = drone_info_registry[i];
                                break;
                            }

                        pthread_mutex_unlock(&drone_info_mutex); 

                        if (found_drone != NULL)
                            break;
                    }

                }

                if (found_drone != NULL) {
                    pthread_mutex_lock(&drone_info_mutex); 

                    found_drone->stat = DRONE_ON_SELF_TRAVEL;
                    found_drone->info->next_hub_id = self->id;
                    found_drone->info->packageInfo = package_will_sent;

                    // NOTE: Should we submit the package to the drone right now, or should we enter the loop again and when the drone comes then we should do it?

                    pthread_mutex_unlock(&drone_info_mutex); 
                } else {
                    _wait(UNIT_TIME);
                    goto select_drones_loop;
                }
            }
        } 
    }

    // HubStopped ()
    pthread_mutex_lock(&hub_info_mutex);
    hub_activity_registry[self->id-1] = 0;
    pthread_mutex_unlock(&hub_info_mutex);

    FillHubInfo(self, self->id);
    LOG_SAFE(WriteOutput(NULL, NULL, NULL, self, HUB_STOPPED))
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

    // TODO: Initizlize first HubInfo, DroneInfo, SenderInfo, ReceiverInfo objects
    //       Start threads

    exit(0);
}