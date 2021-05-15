#include "simulator.h"

SimulationConfig* sim_config = NULL;
// const char* CONFIG_FILE_NAME = "test_config.txt";
// const char* CONFIG_FILE_NAME = "testinp2.txt";
const char* CONFIG_FILE_NAME = NULL;

FILE* __debug_file;
const char* DEBUG_LOG_FILE_NAME = "/dev/null";

pthread_t* hub_pthreads = NULL;
pthread_t* drone_pthreads = NULL;
pthread_t* sender_pthreads = NULL;
pthread_t* receiver_pthreads = NULL;

int __count_packages_waiting = 0;
pthread_mutex_t global_delivery_package_ctr_mutex;

int __simulation_alive_threads_ctr = 0;
pthread_mutex_t simulation_alive_threads_mutex;

pthread_mutex_t hub_info_mutex;
int total_hubs_count;
int* hub_activity_registry = NULL; // int used as bo0l

typedef enum {
    DRONE_ON_PACKAGE_TRANSFER,
    DRONE_ON_SELF_TRAVEL,
    DRONE_ON_HUB
} DroneStatus;

char* __drone_stat_chr(DroneStatus s) {
    switch (s) {
        case DRONE_ON_HUB: return "DRONE_ON_HUB";
        case DRONE_ON_SELF_TRAVEL: return "DRONE_ON_SELF_TRAVEL";
        case DRONE_ON_PACKAGE_TRANSFER: return "DRONE_ON_PACKAGE_TRANSFER";
        default: return "UNKNOWN!!!";
    }
}

typedef struct {
    DroneStatus stat;
    int hub_id;
    // PackageInfo* pkg;
    DroneInfo* info;
    int package_will_sent_index_in_outgoing;
} DynamicDroneInfo;

pthread_mutex_t drone_info_mutex;
DynamicDroneInfo** drone_info_registry = NULL;

pthread_mutex_t receiver_info_mutex;

pthread_mutex_t sender_info_mutex;
int* sender_activity_registry = NULL; // int used as bo0l

pthread_mutex_t* hub_incoming_storage_mutexes = NULL;
pthread_mutex_t* hub_outgoing_storage_mutexes = NULL;

// IMPORTANT: Drones MUST update (using this) charging_spaces_remaining and drone_info_registry.
pthread_mutex_t* hub_charging_spaces_mutexes = NULL;

int* incoming_storage_remaining = NULL;
PackageInfo*** incoming_storages = NULL; 
int* outgoing_storage_remaining = NULL; // hub look at (outgoing_storage_remaining < self.outgoing_capacity)
int* outgoing_waiting_for_drone_pickup = NULL; 
PackageInfo*** outgoing_storages = NULL; 
int* drones_reserved_place_on_hub = NULL;
int* charging_spaces_remaining = NULL; // hub look at (charging_spaces_remaining < self.charging_capacity)

// IMPORTANT NOTE: We need to lock this mutex before using WriteOutput() !!!
pthread_mutex_t debug_printf_mutex;

int __pth_lock_rv, __pth_unlock_rv;

int ___last_lock_line = 0;
int ___last_lock_ok = -1;
int ___last_lock_wid = -1;
long long ___last_lock_ts = -1;

int ___last_unlock_line = 0;
int ___last_unlock_ok = -1;
int ___last_unlock_wid = -1;
long long ___last_unlock_ts = -1;

#define DEBUG_MUTEX_LOCK if (DEBUG_SIMULATOR) { if (pthread_mutex_lock(&debug_printf_mutex) != 0) { perror("    \n !!! pthread_mutex_lock: in DEBUG_MUTEX_LOCK\n"); } }
#define DEBUG_MUTEX_RELEASE if (DEBUG_SIMULATOR) { if (pthread_mutex_unlock(&debug_printf_mutex) != 0) { perror("    \n !!! pthread_mutex_unlock: in DEBUG_MUTEX_RELEASE\n"); } }
#define LOG_SAFE(x) if (DEBUG_SIMULATOR) { if (pthread_mutex_lock(&debug_printf_mutex) != 0) { perror("    \n !!! pthread_mutex_lock: in LOG_SAFE\n"); } x; if (pthread_mutex_unlock(&debug_printf_mutex) != 0) { perror("    \n !!! pthread_mutex_unlock: in LOG_SAFE\n"); } } else { x; }
#define DEBUG_LOG_SAFE(x) if (DEBUG_SIMULATOR) { if (pthread_mutex_lock(&debug_printf_mutex) != 0) { perror("    \n !!! pthread_mutex_lock: in DEBUG_LOG_SAFE\n"); } x; fflush(__debug_file); if (pthread_mutex_unlock(&debug_printf_mutex) != 0) { perror("    \n !!! pthread_mutex_unlock: in DEBUG_LOG_SAFE\n"); } }

#define LOCK_AND_CHECK(x) { if (SUPERLOGGING) {___last_lock_line = __LINE__; ___last_lock_wid = self->id; ___last_lock_ok = 0; ___last_lock_ts = timeInMilliseconds();} if ((__pth_lock_rv = pthread_mutex_lock(&x)) != 0) { fprintf(stderr, "\n !!! pthread_mutex_lock: in LOCK_AND_CHECK(" #x "): %d in %s:%d\n", __pth_lock_rv, __FILE__, __LINE__); exit(1); } if (SUPERLOGGING) {___last_lock_ok = 1;} }
#define UNLOCK_AND_CHECK(x) { if (SUPERLOGGING) {___last_unlock_line = __LINE__; ___last_unlock_wid = self->id; ___last_unlock_ok = 0; ___last_unlock_ts = timeInMilliseconds();} if ((__pth_unlock_rv = pthread_mutex_unlock(&x)) != 0) { fprintf(stderr, "\n !!! pthread_mutex_unlock: in UNLOCK_AND_CHECK(" #x "): %d in %s:%d\n", __pth_unlock_rv, __FILE__, __LINE__); exit(1); } if (SUPERLOGGING) {___last_unlock_ok = 1;} }

#define MIN(a,b) (((a)<(b))?(a):(b))

// TODO: Try this later...
#define FREE_AND_NULL(x) {free(x); x = NULL;}

void defer(void) {
    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "\n[*] Called defer()\n");

    if (incoming_storages != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing incoming_storages\n");

        if ((sim_config != NULL) && (sim_config->hubs != NULL)) {
            for (int i = 0; i < sim_config->hubs_count; i++) {
                if (incoming_storages[i] != NULL) {
                    if (DEBUG_SIMULATOR)
                        fprintf(__debug_file, "    ++ incoming_storages[%d]\n", i);

                    for (int j = 0; j < sim_config->hubs[i].incoming_storge_size; j++)
                        if (incoming_storages[i][j] != NULL)
                            free(incoming_storages[i][j]);

                    free(incoming_storages[i]);
                }
            }
        } else {
            fprintf(__debug_file, "   >!! sim_config is NULL, just freeing incoming_storages.\n");
            fprintf(__debug_file, "   >!! There may be data loss in heap.\n");
        }

        free(incoming_storages);
    }

    if (outgoing_storages != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing outgoing_storages\n");

        if ((sim_config != NULL) && (sim_config->hubs != NULL)) {
            for (int i = 0; i < sim_config->hubs_count; i++) {
                if (outgoing_storages[i] != NULL) {
                    if (DEBUG_SIMULATOR)
                        fprintf(__debug_file, "    ++ outgoing_storages[%d]\n", i);

                    for (int j = 0; j < sim_config->hubs[i].outgoing_storge_size; j++)
                        if (outgoing_storages[i][j] != NULL)
                            free(outgoing_storages[i][j]);

                    free(outgoing_storages[i]);
                }
            }
        } else {
            fprintf(__debug_file, "   >!! sim_config is NULL, just freeing outgoing_storages.\n");
            fprintf(__debug_file, "   >!! There may be data loss in heap.\n");
        }

        free(outgoing_storages);
    }

    if (drone_info_registry != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing drone_info_registry\n");

        if ((sim_config != NULL) && (sim_config->drones != NULL)) {
            for (int i = 0; i < sim_config->drones_count; i++) {
                if (drone_info_registry[i] != NULL) {
                    if (DEBUG_SIMULATOR)
                        fprintf(__debug_file, "    ++ drone_info_registry[%d]\n", i);

                    if (drone_info_registry[i]->info != NULL)
                        free(drone_info_registry[i]->info);

                    free(drone_info_registry[i]);
                }
            }
        } else {
            fprintf(__debug_file, "   >!! sim_config is NULL, just freeing drone_info_registry.\n");
            fprintf(__debug_file, "   >!! There may be data loss in heap.\n");
        }

        free(drone_info_registry);
    }

    if (sim_config != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing sim_config\n");

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

    if (hub_pthreads != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing hub_pthreads\n");
        
        free(hub_pthreads);
    }

    if (drone_pthreads != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing drone_pthreads\n");
        
        free(drone_pthreads);
    }

    if (sender_pthreads != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing sender_pthreads\n");

        free(sender_pthreads);
    }

    if (receiver_pthreads != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing receiver_pthreads\n");

        free(receiver_pthreads);
    }

    if (hub_incoming_storage_mutexes != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing hub_incoming_storage_mutexes\n");

        free(hub_incoming_storage_mutexes);
    }

    if (hub_outgoing_storage_mutexes != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing hub_outgoing_storage_mutexes\n");

        free(hub_outgoing_storage_mutexes);
    }

    if (hub_charging_spaces_mutexes != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing hub_charging_spaces_mutexes\n");

        free(hub_charging_spaces_mutexes);
    }

    if (incoming_storage_remaining != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing incoming_storage_remaining\n");

        free(incoming_storage_remaining);
    }

    if (outgoing_storage_remaining != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing outgoing_storage_remaining\n");

        free(outgoing_storage_remaining);
    }

    if (outgoing_waiting_for_drone_pickup != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing outgoing_waiting_for_drone_pickup\n");

        free(outgoing_waiting_for_drone_pickup);
    }

    if (charging_spaces_remaining != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing charging_spaces_remaining\n");

        free(charging_spaces_remaining);
    }

    if (drones_reserved_place_on_hub != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing drones_reserved_place_on_hub\n");

        free(drones_reserved_place_on_hub);
    }
    
    if (hub_activity_registry != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing hub_activity_registry\n");

        free(hub_activity_registry);
    }

    if (sender_activity_registry != NULL) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, " ++ Freeing sender_activity_registry\n");

        free(sender_activity_registry);
    }

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] defer(): Cleanup OK!\n");
}

void init_mutexes(int hub_count) {
    hub_incoming_storage_mutexes = (pthread_mutex_t*) malloc(sizeof(pthread_mutex_t) * hub_count);

    if (hub_incoming_storage_mutexes == NULL) {
        perror("    malloc/hub_incoming_storage_mutexes");
        exit(1);
    }

    hub_outgoing_storage_mutexes = (pthread_mutex_t*) malloc(sizeof(pthread_mutex_t) * hub_count);

    if (hub_outgoing_storage_mutexes == NULL) {
        perror("    malloc/hub_outgoing_storage_mutexes");
        exit(1);
    }

    hub_charging_spaces_mutexes = (pthread_mutex_t*) malloc(sizeof(pthread_mutex_t) * hub_count);

    if (hub_charging_spaces_mutexes == NULL) {
        perror("    malloc/hub_charging_spaces_mutexes");
        exit(1);
    }

    pthread_mutexattr_t attr;
    pthread_mutexattr_init (&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK_NP);
    // pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE_NP);

    pthread_mutex_init(&simulation_alive_threads_mutex, &attr);
    pthread_mutex_init(&global_delivery_package_ctr_mutex, &attr);

    pthread_mutex_init(&hub_info_mutex, &attr);
    pthread_mutex_init(&drone_info_mutex, &attr);
    pthread_mutex_init(&receiver_info_mutex, &attr);
    pthread_mutex_init(&sender_info_mutex, &attr);
    pthread_mutex_init(&debug_printf_mutex, &attr);

    for (int i = 0; i < hub_count; i++) {
        pthread_mutex_init(&hub_incoming_storage_mutexes[i], &attr);
        pthread_mutex_init(&hub_outgoing_storage_mutexes[i], &attr);
        pthread_mutex_init(&hub_charging_spaces_mutexes[i], &attr);
    }
}

void init_critical_structures(SimulationConfig* conf) {
    total_hubs_count = conf->hubs_count;

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] Allocating incoming_storage_remaining\n");

    incoming_storage_remaining = (int*) malloc(sizeof(int) * conf->hubs_count);
    
    if (incoming_storage_remaining == NULL) {
        perror("    malloc/incoming_storage_remaining");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] Allocating outgoing_storage_remaining\n");
    
    outgoing_storage_remaining = (int*) malloc(sizeof(int) * conf->hubs_count);
    
    if (outgoing_storage_remaining == NULL) {
        perror("    malloc/outgoing_storage_remaining");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] Allocating outgoing_waiting_for_drone_pickup\n");
    
    outgoing_waiting_for_drone_pickup = (int*) malloc(sizeof(int) * conf->hubs_count);
    
    if (outgoing_waiting_for_drone_pickup == NULL) {
        perror("    malloc/outgoing_waiting_for_drone_pickup");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] Allocating charging_spaces_remaining\n");
    
    charging_spaces_remaining = (int*) malloc(sizeof(int) * conf->hubs_count);

    if (charging_spaces_remaining == NULL) {
        perror("    malloc/charging_spaces_remaining");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] Allocating drones_reserved_place_on_hub\n");
    
    drones_reserved_place_on_hub = (int*) malloc(sizeof(int) * conf->hubs_count);

    if (drones_reserved_place_on_hub == NULL) {
        perror("    malloc/drones_reserved_place_on_hub");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] Allocating hub_activity_registry\n");

    hub_activity_registry = (int*) malloc(sizeof(int) * conf->hubs_count);

    if (hub_activity_registry == NULL) {
        perror("    malloc/hub_activity_registry");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] Allocating sender_activity_registry\n");

    sender_activity_registry = (int*) malloc(sizeof(int) * conf->hubs_count);

    if (sender_activity_registry == NULL) {
        perror("    malloc/sender_activity_registry");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] Allocating incoming_storages\n");

    incoming_storages = (PackageInfo***) malloc(sizeof(PackageInfo**) * conf->hubs_count);

    if (incoming_storages == NULL) {
        perror("    malloc/incoming_storages");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] Allocating outgoing_storages\n");

    outgoing_storages = (PackageInfo***) malloc(sizeof(PackageInfo**) * conf->hubs_count);

    if (outgoing_storages == NULL) {
        perror("    malloc/outgoing_storages");
        exit(1);
    }

    for (int i = 0; i < conf->hubs_count; i++) {
        incoming_storage_remaining[i] = conf->hubs[i].incoming_storge_size;
        outgoing_storage_remaining[i] = conf->hubs[i].outgoing_storge_size;
        outgoing_waiting_for_drone_pickup[i] = 0;
        charging_spaces_remaining[i] = conf->hubs[i].charging_space_count;
        drones_reserved_place_on_hub[i] = 0;
        hub_activity_registry[i] = 1;
        sender_activity_registry[i] = 1;

        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, "[*] Allocating incoming_storages[%d]\n", i);

        incoming_storages[i] = (PackageInfo**) malloc(sizeof(PackageInfo*) * conf->hubs[i].incoming_storge_size);

        if (incoming_storages[i] == NULL) {
            perror("    malloc/incoming_storages[i]");
            exit(1);
        }

        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, "[*] Initializing incoming_storages[%d][<size:%d>]\n", i, conf->hubs[i].incoming_storge_size);

        for (int j = 0; j < conf->hubs[i].incoming_storge_size; j++)
            incoming_storages[i][j] = NULL;

        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, "[*] Allocating outgoing_storages[%d]\n", i);

        outgoing_storages[i] = (PackageInfo**) malloc(sizeof(PackageInfo*) * conf->hubs[i].outgoing_storge_size);
    
        if (outgoing_storages[i] == NULL) {
            perror("    malloc/outgoing_storages[i]");
            exit(1);
        }

        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, "[*] Initializing outgoing_storages[%d][<size:%d>]\n", i, conf->hubs[i].outgoing_storge_size);

        for (int j = 0; j < conf->hubs[i].outgoing_storge_size; j++)
            outgoing_storages[i][j] = NULL;
    }

    if (DEBUG_SIMULATOR)
        fprintf(__debug_file, "[*] Allocating drone_info_registry\n");

    drone_info_registry = (DynamicDroneInfo**) malloc(sizeof(DynamicDroneInfo*) * conf->drones_count);

    if (drone_info_registry == NULL) {
        perror("    malloc/drone_info_registry");
        exit(1);
    }

    for (int i = 0; i < conf->drones_count; i++) {
        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, "[*] Allocating drone_info_registry[%d]\n", i);

        drone_info_registry[i] = (DynamicDroneInfo*) malloc(sizeof(DynamicDroneInfo));

        if (drone_info_registry[i] == NULL) {
            perror("    malloc/drone_info_registry[i]");
            exit(1);
        }

        if (DEBUG_SIMULATOR)
            fprintf(__debug_file, "[*] Initializing drone_info_registry[%d]\n", i);

        // TODO: Check this.
        drone_info_registry[i]->hub_id = conf->drones[i].starting_hub_id;
        
        // IMPORTANT: drone_info_registry[i]->info will be allocated and set inside "main" function.
        // drone_info_registry[i]->info->current_hub_id = conf->drones[i].starting_hub_id;
        // drone_info_registry[i]->info->current_range = conf->drones[i].maximum_range;
        // drone_info_registry[i]->info->id = conf->drones[i].drone_id;
        // drone_info_registry[i]->info->next_hub_id = -1;
        // drone_info_registry[i]->info->packageInfo = NULL;
        drone_info_registry[i]->info = NULL;
        drone_info_registry[i]->package_will_sent_index_in_outgoing = -1;
        drone_info_registry[i]->stat = DRONE_ON_HUB;
    }

}

void* sender_thread(void* _sender_thread_config) {
    SenderThreadConfig* sender_thread_config = (SenderThreadConfig*) _sender_thread_config;
    PackageInfo* new_package = NULL;
    SenderInfo* self = (SenderInfo*) sender_thread_config->self;
    SenderConfig* self_config = (SenderConfig*) sender_thread_config->self_config;
    SimulationConfig* simulation_config = (SimulationConfig*) sender_thread_config->simulation_config;

    DEBUG_LOG_SAFE(fprintf(__debug_file, "Sender Thread awake (id: %d, hub: %d, packages: %d)\n", self->id, self->current_hub_id, self->remaining_package_count))

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
        LOCK_AND_CHECK(hub_info_mutex)

        for (int i = 0; i < total_hubs_count; i++)
            if (hub_activity_registry[i])
                active_hubs_count++;

        UNLOCK_AND_CHECK(hub_info_mutex)

        srand(time(NULL));
        
        int hub_index = self->current_hub_id - 1;

        while (hub_index == (self->current_hub_id - 1))
            hub_index  = rand() % active_hubs_count;

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
            perror("     !!! unexpected: hub_id = -1");
            exit(1);
        }

        int receiver_id = simulation_config->hubs[hub_id-1].receiver.receiver_id;

        DEBUG_LOG_SAFE(fprintf(__debug_file, "Sender Thread %d: Selected Hub-%d:Rec-%d for package #%d\n", self->id, hub_id, receiver_id, _packet_index))

        // CRITICAL ERROR: This will be done bty the receiver or defer()!!!
        //                  !!! DO NOT DO THIS AGAIN !!!        
        // if (new_package != NULL)
        //     free(new_package);

        new_package = (PackageInfo*) malloc(sizeof(PackageInfo));

        if (new_package == NULL) {
            perror("    malloc/new_package");
            exit(1);
        }

        FillPacketInfo(new_package, self->id, self->current_hub_id, receiver_id, hub_id);
        FillSenderInfo(self, self->id, self->current_hub_id, self->remaining_package_count, new_package);

        // OLD TODO:
        // put package to self's hub's array on "outgoing_storages"
        // TODO: Change this to pthread_cond_t

        DEBUG_LOG_SAFE(fprintf(__debug_file, "Sender Thread %d: Will put package #%d to my hub (%d)'s outgoing facility\n", self->id, _packet_index, self->current_hub_id))

        int __putwait_log_counter = 0;

        while (1) {
            LOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->current_hub_id-1]); // WaitCanDeposit
            // DEBUG_LOG_SAFE(fprintf(__debug_file, "Sender Thread %d: LOCK PASSED\n", self->id))

            if (outgoing_storage_remaining[self->current_hub_id-1] > 0) {
                int new_package_index_in_store = -1;

                for (int pi = 0; pi < simulation_config->hubs[self->current_hub_id-1].outgoing_storge_size; pi++)
                    if (outgoing_storages[self->current_hub_id-1][pi] == NULL) {
                        new_package_index_in_store = pi;
                        break;
                    }

                if (new_package_index_in_store < 0) {
                    perror("        unexpected: could not select place: new_package_index_in_store");
                    exit(1);
                }

                // int new_package_index_in_store = simulation_config->hubs[self->current_hub_id-1].outgoing_storge_size - outgoing_storage_remaining[self->current_hub_id-1];
                
                outgoing_storages[self->current_hub_id-1][new_package_index_in_store] = new_package; // SenderDeposit
                outgoing_storage_remaining[self->current_hub_id-1] -= 1;
                
                UNLOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->current_hub_id-1]);
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Sender Thread %d: PUT OK (package #%d -> my hub %d)\n", self->id, _packet_index, self->current_hub_id))   
                
                break;
            }

            UNLOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->current_hub_id-1]);

            if ((__putwait_log_counter++ % 10000) == 0)
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Sender Thread %d: PUT WAIT... (package #%d -> my hub %d)\n", self->id, _packet_index, self->current_hub_id))
        }

        // FillPacketInfo && FillSenderInfo && WriteOutput
        LOG_SAFE(WriteOutput(self, NULL, NULL, NULL, SENDER_DEPOSITED))

        // sleep()
        // TODO: Change this to: wait()
        wait(self_config->wait_time_between_packages * UNIT_TIME);
        // sleep(2); // DEBUG: DELETE THIS!
    }

    LOCK_AND_CHECK(sender_info_mutex);
    sender_activity_registry[self->id-1] = 0;
    UNLOCK_AND_CHECK(sender_info_mutex);

    FillSenderInfo(self, self->id, self->current_hub_id, self->remaining_package_count, NULL);
    LOG_SAFE(WriteOutput(self, NULL, NULL, NULL, SENDER_STOPPED))

    LOCK_AND_CHECK(simulation_alive_threads_mutex)
    __simulation_alive_threads_ctr--;
    UNLOCK_AND_CHECK(simulation_alive_threads_mutex)

    DEBUG_LOG_SAFE(fprintf(__debug_file, "Sender Thread %d: Exiting...\n", self->id))

    return NULL;
}

void* receiver_thread(void* _receiver_thread_config) {
    ReceiverThreadConfig* receiver_thread_config = (ReceiverThreadConfig*) _receiver_thread_config;
    ReceiverInfo* self = (ReceiverInfo*) receiver_thread_config->self;
    ReceiverConfig* self_config = (ReceiverConfig*) receiver_thread_config->self_config;
    SimulationConfig* simulation_config = (SimulationConfig*) receiver_thread_config->simulation_config;

    DEBUG_LOG_SAFE(fprintf(__debug_file, "Receiver Thread awake (id: %d, hub: %d)\n", self->id, self->current_hub_id))
    LOG_SAFE(WriteOutput(NULL, self, NULL, NULL, RECEIVER_CREATED))

    int __recv_pkgs_log_ctr = 0;

    while (1) {
        LOCK_AND_CHECK(hub_info_mutex); // while CurrentHub is active do

        if (!hub_activity_registry[self->current_hub_id-1]) {
            UNLOCK_AND_CHECK(hub_info_mutex);
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Receiver Thread %d: My hub is not active, exiting.\n", self->id))
            break;
        }

        UNLOCK_AND_CHECK(hub_info_mutex);
        
        int packages_got_this_round = 0;

        LOCK_AND_CHECK(hub_incoming_storage_mutexes[self->current_hub_id-1]);
        // DEBUG_LOG_SAFE(fprintf(__debug_file, "Receiver Thread %d: Checking incoming packages...\n", self->id))

        for (int package_index = 0; package_index < simulation_config->hubs[self->current_hub_id-1].incoming_storge_size; package_index++)
            if (incoming_storages[self->current_hub_id-1][package_index] != NULL)
                if (incoming_storages[self->current_hub_id-1][package_index]->receiver_id == self->id) {
                    PackageInfo* incoming_package = incoming_storages[self->current_hub_id-1][package_index];
                    incoming_storages[self->current_hub_id-1][package_index] = NULL;
                    incoming_storage_remaining[self->current_hub_id-1] += 1;

                    FillPacketInfo(incoming_package, incoming_package->sender_id, incoming_package->sending_hub_id, incoming_package->receiver_id, self->current_hub_id);
                    FillReceiverInfo(self, self->id, self->current_hub_id, incoming_package);
                    LOG_SAFE(WriteOutput(NULL, self, NULL, NULL, RECEIVER_PICKUP))

                    DEBUG_LOG_SAFE(fprintf(__debug_file, "Receiver Thread %d: Got package from sender-%d, hub-%d.\n", self->id, incoming_package->sender_id, incoming_package->sending_hub_id))

                    packages_got_this_round++;

                    // TODO: Do this
                    // BUT: Be careful for double-freeing
                    free(incoming_package);

                    // TODO: Change this to: wait()
                    wait(self_config->wait_time_between_packages * UNIT_TIME);
                }

        UNLOCK_AND_CHECK(hub_incoming_storage_mutexes[self->current_hub_id-1]);

        if (__recv_pkgs_log_ctr++ % 1000 == 0)
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Receiver Thread %d: Received %d packages this round...\n", self->id, packages_got_this_round))
        
        int remaining_packages_global;
        LOCK_AND_CHECK(global_delivery_package_ctr_mutex)
        __count_packages_waiting -= packages_got_this_round;
        remaining_packages_global = __count_packages_waiting;
        UNLOCK_AND_CHECK(global_delivery_package_ctr_mutex)
        
        if (__recv_pkgs_log_ctr++ % 1000 == 1)
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Receiver Thread %d: Packages left: %d\n", self->id, remaining_packages_global))
    }

    FillReceiverInfo(self, self->id, self->current_hub_id, NULL);
    LOG_SAFE(WriteOutput(NULL, self, NULL, NULL, RECEIVER_STOPPED))

    LOCK_AND_CHECK(simulation_alive_threads_mutex)
    __simulation_alive_threads_ctr--;
    UNLOCK_AND_CHECK(simulation_alive_threads_mutex)

    DEBUG_LOG_SAFE(fprintf(__debug_file, "Receiver Thread %d: Exiting...\n", self->id))

    return NULL;
}

void* hub_thread(void* _hub_thread_config) {
    HubThreadConfig* hub_thread_config = (HubThreadConfig*) _hub_thread_config;
    HubInfo* self = (HubInfo*) hub_thread_config->self;
    HubConfig* self_config = (HubConfig*) hub_thread_config->self_config;
    SimulationConfig* simulation_config = (SimulationConfig*) hub_thread_config->simulation_config;

    DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread awake (id: %d)\n", self->id))
    LOG_SAFE(WriteOutput(NULL, NULL, NULL, self, HUB_CREATED))

    int __o_ao_ctr = 0;

    while (1) { // while there are active senders or packages in either storage do
        int active_senders = 0;
        LOCK_AND_CHECK(sender_info_mutex);

        for (int i = 0; i < simulation_config->hubs_count; i++) {
            // TODO: Do we count this hub's sender too??
            // int sender_id = i + 1;
            // if (sender_id == self_config->sender.sender_id)
            //     continue;

            if (sender_activity_registry[i])
                active_senders++;
        }

        UNLOCK_AND_CHECK(sender_info_mutex);

        int active_packages_to_transfer = 0;

        for (int i = 0; i < simulation_config->hubs_count; i++) {
            LOCK_AND_CHECK(hub_incoming_storage_mutexes[i])

            for (int j = 0; j < simulation_config->hubs[i].incoming_storge_size; j++)
                if (incoming_storages[i][j] != NULL)
                    active_packages_to_transfer++;

            UNLOCK_AND_CHECK(hub_incoming_storage_mutexes[i])

            LOCK_AND_CHECK(hub_outgoing_storage_mutexes[i])

            for (int j = 0; j < simulation_config->hubs[i].outgoing_storge_size; j++)
                if (outgoing_storages[i][j] != NULL)
                    active_packages_to_transfer++;

            UNLOCK_AND_CHECK(hub_outgoing_storage_mutexes[i])
        }

        int drones_on_air = 0;
        int drones_charging_to_fly = 0;
        LOCK_AND_CHECK(drone_info_mutex)
        for (int i = 0; i < simulation_config->drones_count; i++) {
            if (drone_info_registry[i]->stat != DRONE_ON_HUB)
                drones_on_air++;
            else if (drone_info_registry[i]->info->packageInfo != NULL)
                drones_charging_to_fly++;
        }
        UNLOCK_AND_CHECK(drone_info_mutex)

        LOCK_AND_CHECK(hub_incoming_storage_mutexes[self->id-1]);
        int incoming_packages = self_config->incoming_storge_size - incoming_storage_remaining[self->id-1];
        UNLOCK_AND_CHECK(hub_incoming_storage_mutexes[self->id-1]);

        LOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->id-1]);
        int outgoing_packages = self_config->outgoing_storge_size - outgoing_storage_remaining[self->id-1];
        int active_outgoing_packages = outgoing_packages - outgoing_waiting_for_drone_pickup[self->id-1];
        UNLOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->id-1]);

        LOCK_AND_CHECK(global_delivery_package_ctr_mutex)
        int global_packages_left = __count_packages_waiting;
        UNLOCK_AND_CHECK(global_delivery_package_ctr_mutex)
    
        if ((global_packages_left == 0) && (drones_charging_to_fly == 0) && (drones_on_air == 0) && (active_packages_to_transfer == 0) && (active_senders == 0) && (incoming_packages == 0) && (outgoing_packages == 0)) {
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: O: %d, AO: %d, WDP: %d, ALL: %d, DOA: %d, DCF: %d, GPL: %d\n", self->id, outgoing_packages, active_outgoing_packages, outgoing_waiting_for_drone_pickup[self->id-1], active_packages_to_transfer, drones_on_air, drones_charging_to_fly, global_packages_left))
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: There are no active senders or incoming/outgoing packages.\n", self->id))
            break;
        }

        if (__o_ao_ctr++ % 300 == 0)
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: O: %d, AO: %d, WDP: %d, ALL: %d, DOA: %d, DCF: %d, GPL: %d\n", self->id, outgoing_packages, active_outgoing_packages, outgoing_waiting_for_drone_pickup[self->id-1], active_packages_to_transfer, drones_on_air, drones_charging_to_fly, global_packages_left))

        if (active_outgoing_packages < 0) {
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: O: %d, AO: %d\n", self->id, outgoing_packages, active_outgoing_packages))
            perror("    !!! active_outgoing_packages < 0!");
            exit(1);
        }

        if (active_outgoing_packages > 0) { // WaitUntilPackageDeposited ()
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: O: %d, AO: %d, WDP: %d, ALL: %d, DOA: %d, DCF: %d, GPL: %d\n", self->id, outgoing_packages, active_outgoing_packages, outgoing_waiting_for_drone_pickup[self->id-1], active_packages_to_transfer, drones_on_air, drones_charging_to_fly, global_packages_left))
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: There are %d outgoing, %d active packages.\n", self->id, outgoing_packages, active_outgoing_packages))
            // Select the package
            LOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->id-1]);

            PackageInfo* package_will_sent = NULL;
            int package_will_sent_index_in_outgoing = -1;

            for (int i = 0; i < self_config->outgoing_storge_size; i++)
                if (outgoing_storages[self->id-1][i] != NULL) {
                    package_will_sent = outgoing_storages[self->id-1][i];
                    package_will_sent_index_in_outgoing = i;
                    outgoing_storages[self->id-1][i] = NULL;
                    // outgoing_storage_remaining[self->id-1]--; // drone should do this.
                    outgoing_waiting_for_drone_pickup[self->id-1]++;
                    DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: outgoing_storage_remaining: %d, outgoing_waiting_for_drone_pickup: %d\n", self->id, outgoing_storage_remaining[self->id-1], outgoing_waiting_for_drone_pickup[self->id-1]))
                    break;
                }

            UNLOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->id-1]);
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: Package to send: (index: %d) %p (sender: %d, from hub: %d)\n", self->id, package_will_sent_index_in_outgoing, package_will_sent, package_will_sent == NULL ? -1 : package_will_sent->sender_id, package_will_sent == NULL ? -1 : package_will_sent->sending_hub_id))

            // A drone may be reserved a space to send a package!!
            // UPDATE: NOOOOO!!!!! ONLY SENDERS PUT PACKAGES TO OUTGOING STORAGE!!!!
            if (package_will_sent == NULL) {
                // perror("    unexpected: could not select package: package_will_sent");
                // exit(1);
                
                // continue; // FALSY!!

                perror("        unexpected: could not select package: package_will_sent");
                exit(1);
            }

// TODO: This may be done in a better way...
select_drones_loop:
            // Find drones...
            // Check if there are drones in the hub
            // LOCK_AND_CHECK(hub_charging_spaces_mutexes[self->id-1]);
            LOCK_AND_CHECK(drone_info_mutex)
            int drone_count_on_me = 0;
            for (int i = 0; i < simulation_config->drones_count; i++)
                if ((drone_info_registry[i]->stat == DRONE_ON_HUB) && (drone_info_registry[i]->hub_id == self->id))
                    drone_count_on_me++;
            // int drone_count_on_me = self_config->charging_space_count - (charging_spaces_remaining[self->id-1] - drones_reserved_place_on_hub[self->id-1]);
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: Drones on me: %d (CR: %d, DR: %d)\n", self->id, drone_count_on_me, charging_spaces_remaining[self->id-1], drones_reserved_place_on_hub[self->id-1]))
            // UNLOCK_AND_CHECK(hub_charging_spaces_mutexes[self->id-1]);

            if (drone_count_on_me > 0) {
                DynamicDroneInfo** drones_on_me = malloc(sizeof(DynamicDroneInfo*) * drone_count_on_me);

                if (drones_on_me == NULL) {
                    perror("    malloc/drones_on_me");
                    exit(1);
                }

                // LOCK_AND_CHECK(drone_info_mutex);

                for (int i = 0, dom_index = 0; i < simulation_config->drones_count; i++)
                    if ((drone_info_registry[i]->stat == DRONE_ON_HUB) && drone_info_registry[i]->hub_id == self->id)
                        drones_on_me[dom_index++] = drone_info_registry[i];

                // UNLOCK_AND_CHECK(drone_info_mutex);

                DynamicDroneInfo* drone_with_highest_range_on_me = NULL;

                for (int i = 0; i < drone_count_on_me; i++)
                    if ((drone_with_highest_range_on_me == NULL) || (drones_on_me[i]->info->current_range > drone_with_highest_range_on_me->info->current_range))
                        drone_with_highest_range_on_me = drones_on_me[i];

                // LOCK_AND_CHECK(drone_info_mutex); 
                
                if (drone_with_highest_range_on_me == NULL) {
                    UNLOCK_AND_CHECK(drone_info_mutex); 
                    perror("    unexpected: could not select drone: drone_with_highest_range_on_me");
                    // exit(1);
                    free(drones_on_me);
                    wait(UNIT_TIME);
                    goto select_drones_loop;
                }

                DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: best drone on me: %p\n", self->id, drone_with_highest_range_on_me))

                // assign the package to the drone
                // AssignAndNotifyDrone (Package, Drone)
                drone_with_highest_range_on_me->stat = DRONE_ON_PACKAGE_TRANSFER;
                drone_with_highest_range_on_me->package_will_sent_index_in_outgoing = package_will_sent_index_in_outgoing;
                drone_with_highest_range_on_me->info->packageInfo = package_will_sent;
                drone_with_highest_range_on_me->info->next_hub_id = package_will_sent->receiving_hub_id;
                UNLOCK_AND_CHECK(drone_info_mutex); 

                DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: Assigned package (H%d->H%d) to drone-%d!\n", self->id, package_will_sent->sending_hub_id, package_will_sent->receiving_hub_id, drone_with_highest_range_on_me->info->id))

                free(drones_on_me);
            } else {
                // UNLOCK_AND_CHECK(drone_info_mutex);          
                // CallDroneFromHubs ()
                // if No drone is found in other hubs then
                //     WaitTimeoutOrDrone ()
                //     goto start
                // end

                DynamicDroneInfo* found_drone = NULL;

                for (int i = 0; i < total_hubs_count - 1; i++) { 
                    int other_hub_id = self_config->nearest_other_hubs_sorted[i];

                    LOCK_AND_CHECK(hub_charging_spaces_mutexes[other_hub_id-1]);
                    int drones_in_neighbor_hub = simulation_config->hubs[other_hub_id-1].charging_space_count - (charging_spaces_remaining[other_hub_id-1] - drones_reserved_place_on_hub[other_hub_id-1]);
                    UNLOCK_AND_CHECK(hub_charging_spaces_mutexes[other_hub_id-1]);
                    
                    if (drones_in_neighbor_hub > 0) {
                        // LOCK_AND_CHECK(drone_info_mutex); 

                        for (int i = 0; i < simulation_config->drones_count; i++)
                            if ((drone_info_registry[i]->stat == DRONE_ON_HUB) && drone_info_registry[i]->hub_id == other_hub_id) {
                                found_drone = drone_info_registry[i];
                                break;
                            }

                        // UNLOCK_AND_CHECK(drone_info_mutex); 

                        if (found_drone != NULL)
                            break;
                    }

                }

                DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: Neighbor drone found: %p!\n", self->id, found_drone))

                if (found_drone != NULL) {
                    // LOCK_AND_CHECK(drone_info_mutex); 
                    int neighbor_hub_id = found_drone->hub_id;

                    found_drone->stat = DRONE_ON_SELF_TRAVEL;
                    found_drone->info->next_hub_id = self->id;
                    found_drone->info->packageInfo = package_will_sent;
                    found_drone->package_will_sent_index_in_outgoing = package_will_sent_index_in_outgoing;

                    // NOTE: Should we submit the package to the drone right now, or should we enter the loop again and when the drone comes then we should do it?

                    UNLOCK_AND_CHECK(drone_info_mutex); 

                    DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: Assigned package to neighbor drone-%d (%p: to hub %d)!\n", self->id, found_drone->info->id, package_will_sent, package_will_sent->receiving_hub_id))

                    // TODO: Do some logic here, wait until the drone comes and it will charge here and then go.
                    while (1) {
                        LOCK_AND_CHECK(drone_info_mutex);  
                        if (found_drone->info->current_hub_id == self->id) {
                            UNLOCK_AND_CHECK(drone_info_mutex); 
                            break;
                        }
                        UNLOCK_AND_CHECK(drone_info_mutex); 
                    }

                    DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: Neighbor(%d)'s drone(%d) came!!\n", self->id, neighbor_hub_id, found_drone->info->id))

                    LOCK_AND_CHECK(drone_info_mutex);  
                    found_drone->stat = DRONE_ON_PACKAGE_TRANSFER;
                    found_drone->info->next_hub_id = package_will_sent->receiving_hub_id;
                    UNLOCK_AND_CHECK(drone_info_mutex);  

                    // Wait until the drone takes the package... (it may charge...)
                    int __drone_waiting_ctr = 0;
                    while (1) {
                        LOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->id-1]);
                        if (outgoing_storages[self->id-1][package_will_sent_index_in_outgoing] == NULL) {
                            UNLOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->id-1]);
                            break;
                        }
                        UNLOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->id-1]);

                        if (__drone_waiting_ctr++ % 200 == 0)
                            DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: Still waiting new drone (%d) ...\n", self->id, found_drone->info->id))
                    }

                    DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: My new drone(%d) took the package.\n", self->id, found_drone->info->id))
                } else {
                    UNLOCK_AND_CHECK(drone_info_mutex); 
                    wait(UNIT_TIME);

                    // TODO: EDIT HERE!
                    // WaitTimeoutOrDrone: Wait a specific duration until a drone arrives. The specific duration is 1 units of time. Time units will be explained later.

                    goto select_drones_loop;
                }
            }
        } 
    }

    // HubStopped ()
    LOCK_AND_CHECK(hub_info_mutex);
    hub_activity_registry[self->id-1] = 0;
    UNLOCK_AND_CHECK(hub_info_mutex);

    FillHubInfo(self, self->id);
    LOG_SAFE(WriteOutput(NULL, NULL, NULL, self, HUB_STOPPED))

    LOCK_AND_CHECK(simulation_alive_threads_mutex)
    __simulation_alive_threads_ctr--;
    UNLOCK_AND_CHECK(simulation_alive_threads_mutex)

    DEBUG_LOG_SAFE(fprintf(__debug_file, "Hub Thread %d: Exiting...\n", self->id))

    return NULL;
}

void* drone_thread(void* _drone_thread_config) {
    DroneThreadConfig* drone_thread_config = (DroneThreadConfig*) _drone_thread_config;
    DroneInfo* self = (DroneInfo*) drone_thread_config->self;
    DroneConfig* self_config = (DroneConfig*) drone_thread_config->self_config;
    SimulationConfig* simulation_config = (SimulationConfig*) drone_thread_config->simulation_config;

    DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread awake (id: %d, on hub: %d)\n", self->id, self->current_hub_id))
    LOG_SAFE(WriteOutput(NULL, NULL, self, NULL, DRONE_CREATED))

    long long _last_charged_timestamp = timeInMilliseconds();

    int there_are_active_hubs;

    int __drone_act_hub_log_ctr = 0;

    while (1) {
        there_are_active_hubs = 0;

        LOCK_AND_CHECK(hub_info_mutex)

        for (int i = 0; i < total_hubs_count; i++)
            if (hub_activity_registry[i]) {
                there_are_active_hubs++;
            }

        UNLOCK_AND_CHECK(hub_info_mutex)

        if (__drone_act_hub_log_ctr++ % 1000 == 0)
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: There are %d active hubs!\n", self->id, there_are_active_hubs))

        if (!there_are_active_hubs) {
            DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: There are no active hubs, exiting...\n", self->id))
            break;
        }

        LOCK_AND_CHECK(drone_info_mutex)
        DynamicDroneInfo* dyn_info = drone_info_registry[self->id-1];
        UNLOCK_AND_CHECK(drone_info_mutex)

        int delivery_hub_id = dyn_info->info->next_hub_id;
        PackageInfo* delivery_package = dyn_info->info->packageInfo;

        switch (dyn_info->stat) {
            case DRONE_ON_PACKAGE_TRANSFER: 
                /////// DELETE LATER
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_PACKAGE_TRANSFER: %p\n", self->id, delivery_package))

                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_PACKAGE_TRANSFER: from sender-%d to receiver-%d\n", self->id, delivery_package->sender_id, delivery_package->receiver_id))

                // TODO: Change this to pthread_cond_t or semaphore
                while (1) {
                    int _done = 0;   

                    LOCK_AND_CHECK(hub_charging_spaces_mutexes[delivery_hub_id-1])
                    
                    if (charging_spaces_remaining[delivery_hub_id-1] > 0) {
                        charging_spaces_remaining[delivery_hub_id-1]--;
                        drones_reserved_place_on_hub[delivery_hub_id-1]++;

                        _done = 1;
                        DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_PACKAGE_TRANSFER: Reserved charging space\n", self->id))
                    }

                    UNLOCK_AND_CHECK(hub_charging_spaces_mutexes[delivery_hub_id-1])

                    if (_done)
                        break;
                }

                while (1) {
                    int _done = 0;   

                    LOCK_AND_CHECK(hub_incoming_storage_mutexes[delivery_hub_id-1])
                    
                    if (incoming_storage_remaining[delivery_hub_id-1] > 0) {
                        // Reserve package space for me!
                        incoming_storage_remaining[delivery_hub_id-1]--;

                        _done = 1;
                        DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_PACKAGE_TRANSFER: Reserved incoming package space\n", self->id))
                    }

                    UNLOCK_AND_CHECK(hub_incoming_storage_mutexes[delivery_hub_id-1])

                    if (_done)
                        break;
                }

                long long the_timestamp_on_start = timeInMilliseconds();

                int distance_between_hubs = simulation_config->hubs[self->current_hub_id-1].distance_to_other_hubs[delivery_hub_id-1];

                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_PACKAGE_TRANSFER: Current charge: %d\n", self->id, self->current_range))

                // WaitForRange ()
                // while (self->current_range < distance_between_hubs) {
                while (self->current_range < range_decrease(distance_between_hubs, self_config->travel_speed)) {
                    LOCK_AND_CHECK(drone_info_mutex)
                    self->current_range = calculate_drone_charge(timeInMilliseconds() - the_timestamp_on_start, self->current_range, self_config->maximum_range);
                    UNLOCK_AND_CHECK(drone_info_mutex)
                }

                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_PACKAGE_TRANSFER: After charging: %d\n", self->id, self->current_range))
                
                LOCK_AND_CHECK(drone_info_mutex)
                FillPacketInfo(delivery_package, delivery_package->sender_id, delivery_package->sending_hub_id, delivery_package->receiver_id, delivery_package->receiving_hub_id);
                FillDroneInfo(self, self->id, self->current_hub_id, self->current_range, delivery_package, delivery_hub_id);
                // UNLOCK_AND_CHECK(drone_info_mutex)
                LOG_SAFE(WriteOutput(NULL, NULL, self, NULL, DRONE_PICKUP))

                travel(distance_between_hubs, self_config->travel_speed); // Sleep the duration of travel
                // LOCK_AND_CHECK(drone_info_mutex)
                self->current_range = self->current_range - range_decrease(distance_between_hubs, self_config->travel_speed); // CurrentRange ← CurrentRange − (Distance/Speed)
                UNLOCK_AND_CHECK(drone_info_mutex)

                // DropPackageToHub (Package)
                LOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->current_hub_id-1])
                // outgoing_storages[self->current_hub_id-1][dyn_info->package_will_sent_index_in_outgoing] = NULL;
                outgoing_storage_remaining[self->current_hub_id-1]++;
                outgoing_waiting_for_drone_pickup[self->current_hub_id-1]--;
                UNLOCK_AND_CHECK(hub_outgoing_storage_mutexes[self->current_hub_id-1])

                int delivery_done = 0;

                LOCK_AND_CHECK(hub_incoming_storage_mutexes[delivery_hub_id-1])  
                for (int i = 0; i < simulation_config->hubs[delivery_hub_id-1].incoming_storge_size; i++)
                    if (incoming_storages[delivery_hub_id-1][i] == NULL) {
                        incoming_storages[delivery_hub_id-1][i] = delivery_package;
                        delivery_done = 1;
                        break;
                    }
                UNLOCK_AND_CHECK(hub_incoming_storage_mutexes[delivery_hub_id-1])

                if (!delivery_done) {
                    DEBUG_LOG_SAFE(fprintf(stderr, "   !!! Drone Thread %d: Could not find a plave tp put the incoming package!! (from hub: %d, to hub: %d)\n", self->id, self->current_hub_id, delivery_hub_id))
                    exit(1);
                }
                
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_PACKAGE_TRANSFER: Done delivery to hub: %d\n", self->id, delivery_hub_id))

                LOCK_AND_CHECK(hub_charging_spaces_mutexes[self->current_hub_id-1])
                charging_spaces_remaining[self->current_hub_id-1]++;
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_PACKAGE_TRANSFER: Incr charging space counter on old hub: %d\n", self->id, self->current_hub_id))
                UNLOCK_AND_CHECK(hub_charging_spaces_mutexes[self->current_hub_id-1])

                LOCK_AND_CHECK(hub_charging_spaces_mutexes[delivery_hub_id-1])
                drones_reserved_place_on_hub[delivery_hub_id-1]--;
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_PACKAGE_TRANSFER: Decr reserved charging space counter on new hub: %d\n", self->id, delivery_hub_id))
                UNLOCK_AND_CHECK(hub_charging_spaces_mutexes[delivery_hub_id-1])

                // CurrentHub ← DestinationHub
                LOCK_AND_CHECK(drone_info_mutex)
                self->current_hub_id = delivery_hub_id;
                dyn_info->hub_id = self->current_hub_id;
                dyn_info->info->current_hub_id = self->current_hub_id; // this is probably "gereksiz", as dyn_info->info == self, but anyways!
                dyn_info->info->current_range = self->current_range;
                dyn_info->package_will_sent_index_in_outgoing = -1;
                dyn_info->stat = DRONE_ON_HUB;
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_PACKAGE_TRANSFER: Updated dynamic info: new hub: %d, charge: %d\n", self->id, self->current_hub_id, self->current_range))
                UNLOCK_AND_CHECK(drone_info_mutex)
                
                // FillPacketInfo(PackageInfo, SenderID, SendingHub, ReceiverID,
                // ReceivingHub )
                // FillDroneInfo(DroneInfo, ID, CurrentHub, CurrentRange, PackageInfo, 0)
                // WriteOutput(NULL, NULL, DroneInfo, NULL, DRONE_DEPOSITED)

                LOCK_AND_CHECK(drone_info_mutex)
                FillPacketInfo(delivery_package, delivery_package->sender_id, delivery_package->sending_hub_id, delivery_package->receiver_id, delivery_package->receiving_hub_id);
                FillDroneInfo(self, self->id, self->current_hub_id, self->current_range, delivery_package, 0);
                LOG_SAFE(WriteOutput(NULL, NULL, self, NULL, DRONE_DEPOSITED))
                self->packageInfo = NULL; // after drop-off, we can null this.
                UNLOCK_AND_CHECK(drone_info_mutex)

                _last_charged_timestamp = timeInMilliseconds();

                break;
            case DRONE_ON_SELF_TRAVEL:
                /////// DELETE LATER
                DEBUG_LOG_SAFE(fprintf(__debug_file, "     Drone Thread %d: DRONE_ON_SELF_TRAVEL: %p %p \n", self->id, delivery_package, dyn_info->info->packageInfo))

                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_SELF_TRAVEL: from hub-%d to hub-%d\n", self->id, self->current_hub_id, delivery_hub_id))

                // TODO: Change this to pthread_cond_t or semaphore
                while (1) {
                    int _done = 0;   

                    LOCK_AND_CHECK(hub_charging_spaces_mutexes[delivery_hub_id-1])
                    
                    if (charging_spaces_remaining[delivery_hub_id-1] > 0) {
                        // Reserve charging space for me!
                        charging_spaces_remaining[delivery_hub_id-1]--;
                        drones_reserved_place_on_hub[delivery_hub_id-1]++;

                        _done = 1;
                        DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_SELF_TRAVEL: Reserved charging space from hub: %d\n", self->id, delivery_hub_id))
                    }

                    UNLOCK_AND_CHECK(hub_charging_spaces_mutexes[delivery_hub_id-1])

                    if (_done)
                        break;
                }

                long long travel__the_timestamp_on_start = timeInMilliseconds();

                int travel__distance_between_hubs = simulation_config->hubs[self->current_hub_id-1].distance_to_other_hubs[delivery_hub_id-1];

                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_SELF_TRAVEL: Current charge: %d\n", self->id, self->current_range))

                // WaitForRange ()
                // while (self->current_range < distance_between_hubs) {
                while (self->current_range < range_decrease(travel__distance_between_hubs, self_config->travel_speed)) {
                    LOCK_AND_CHECK(drone_info_mutex)
                    self->current_range = calculate_drone_charge(timeInMilliseconds() - travel__the_timestamp_on_start, self->current_range, self_config->maximum_range);
                    UNLOCK_AND_CHECK(drone_info_mutex)
                }

                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_SELF_TRAVEL: After charging: %d\n", self->id, self->current_range))

                LOCK_AND_CHECK(drone_info_mutex)
                PackageInfo* __tmp_pi = dyn_info->info->packageInfo;
                FillDroneInfo(self, self->id, self->current_hub_id, self->current_range, NULL, delivery_hub_id);
                UNLOCK_AND_CHECK(drone_info_mutex)

                LOG_SAFE(WriteOutput(NULL, NULL, self, NULL, DRONE_GOING))

                travel(travel__distance_between_hubs, self_config->travel_speed); // Sleep the duration of travel

                LOCK_AND_CHECK(hub_charging_spaces_mutexes[self->current_hub_id-1])
                charging_spaces_remaining[self->current_hub_id-1]++;
                UNLOCK_AND_CHECK(hub_charging_spaces_mutexes[self->current_hub_id-1])
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_SELF_TRAVEL: Incr charging space counter on old hub: %d\n", self->id, self->current_hub_id))

                LOCK_AND_CHECK(hub_charging_spaces_mutexes[delivery_hub_id-1])
                drones_reserved_place_on_hub[delivery_hub_id-1]--;
                UNLOCK_AND_CHECK(hub_charging_spaces_mutexes[delivery_hub_id-1])
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_SELF_TRAVEL: Decr reserved charging space counter on new hub: %d\n", self->id, delivery_hub_id))

                /////// DELETE LATER
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_SELF_TRAVEL: __tmp_pi 1: %p\n", self->id, __tmp_pi))

                LOCK_AND_CHECK(drone_info_mutex)
                self->current_range = self->current_range - range_decrease(travel__distance_between_hubs, self_config->travel_speed); // CurrentRange ← CurrentRange − (Distance/Speed)
                self->current_hub_id = delivery_hub_id;
                dyn_info->hub_id = self->current_hub_id;
                dyn_info->info->current_hub_id = self->current_hub_id; // this is probably "gereksiz", as dyn_info->info == self, but anyways!
                dyn_info->info->current_range = self->current_range;
                // dyn_info->package_will_sent_index_in_outgoing = -1;
                // dyn_info->stat = DRONE_ON_HUB; // NOOO!!! the hub will edit this!
                FillDroneInfo(self, self->id, self->current_hub_id, self->current_range, NULL, 0);
                UNLOCK_AND_CHECK(drone_info_mutex)
                
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_SELF_TRAVEL: Updated dynamic info: new hub: %d, charge: %d\n", self->id, self->current_hub_id, self->current_range))
                LOG_SAFE(WriteOutput(NULL, NULL, self, NULL, DRONE_ARRIVED))
        
                // When placed self to other hub, it's time to charge!!!
                _last_charged_timestamp = timeInMilliseconds();
                // When charge is enough, go fly!

                // IMPORTANT!!
                // We need to wait for self->stat = DRONE_ON_PACKAGE_TRANSFER;
                LOCK_AND_CHECK(drone_info_mutex)
                dyn_info->info->packageInfo = __tmp_pi;
                dyn_info->info->next_hub_id = delivery_hub_id;
                /////// DELETE LATER
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_SELF_TRAVEL: __tmp_pi 2: %p, TO HUB; %d (%d)\n", self->id, __tmp_pi, dyn_info->info->next_hub_id, delivery_hub_id))
                UNLOCK_AND_CHECK(drone_info_mutex)
                
                while (1) {
                    LOCK_AND_CHECK(drone_info_mutex)
                    if (dyn_info->stat == DRONE_ON_PACKAGE_TRANSFER) {
                        UNLOCK_AND_CHECK(drone_info_mutex)
                        break;
                    }
                    UNLOCK_AND_CHECK(drone_info_mutex)
                }

                break;
            case DRONE_ON_HUB:
                // No package assigned, not on travel.
                // Just wait to be assigned.

                // Charge self.
                LOCK_AND_CHECK(drone_info_mutex)

                long long curr_time = timeInMilliseconds();

                if (self->current_range == self_config->maximum_range) {
                    UNLOCK_AND_CHECK(drone_info_mutex)
                    // DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_HUB: Idle.\n", self->id))
                } else {
                    int old_charge = self->current_range;
                    self->current_range = calculate_drone_charge(curr_time - _last_charged_timestamp, self->current_range, self_config->maximum_range);
                    // DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: DRONE_ON_HUB: Charged by: %d -> %d (%d-%d)\n", self->id, old_charge, self->current_range, curr_time, _last_charged_timestamp))
                    _last_charged_timestamp = curr_time;
                    UNLOCK_AND_CHECK(drone_info_mutex)
                }

                break;
            default:
                DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: drone_info_registry[%d]->stat = %d, UNEXPECTED!!! OUT OF BOUNDS!!!\n", self->id, self->id-1, drone_info_registry[self->id-1]->stat))
                exit(1);
        }
    }

    FillDroneInfo(self, self->id, self->current_hub_id, self->current_range, NULL, 0);
    LOG_SAFE(WriteOutput(NULL, NULL, self, NULL, DRONE_STOPPED))

    LOCK_AND_CHECK(simulation_alive_threads_mutex)
    __simulation_alive_threads_ctr--;
    UNLOCK_AND_CHECK(simulation_alive_threads_mutex)

    DEBUG_LOG_SAFE(fprintf(__debug_file, "Drone Thread %d: Exiting...\n", self->id))

    return NULL;
}

void* debugger_thread(void* __v) {
    while (1) {
        pthread_mutex_lock(&debug_printf_mutex);
        pthread_mutex_lock(&simulation_alive_threads_mutex);
        if (__simulation_alive_threads_ctr == 0) {
            printf("====================[DEBUG DONE!!!]====================\n");
            pthread_mutex_unlock(&simulation_alive_threads_mutex);
            pthread_mutex_unlock(&debug_printf_mutex);
            return NULL;
        } 
        pthread_mutex_unlock(&simulation_alive_threads_mutex);
        long long now = timeInMilliseconds();
        printf("====================[DEBUG START]====================\n");
        printf("[%lld] AGO: %lld (ON: %lld) LLL: %d, LLO: %d, LLWID: %d\n", now, now - ___last_lock_ts, ___last_lock_ts,  ___last_lock_line, ___last_lock_ok, ___last_lock_wid);
        printf("[%lld] AGO: %lld (ON: %lld) LULL: %d, LULO: %d, LULWID: %d\n", now, now - ___last_lock_ts, ___last_lock_ts, ___last_unlock_line, ___last_unlock_ok, ___last_unlock_wid);
        // LOCK_AND_CHECK(hub_info_mutex)
        // for (int i = 0; i < sim_config->hubs_count; i++) {
        //     LOCK_AND_CHECK(hub_incoming_storage_mutexes[i])
        //     LOCK_AND_CHECK(hub_outgoing_storage_mutexes[i])
        //     printf("Hub-%d: %s: I [", i+1, hub_activity_registry[i] ? "ACTIVE" : "DEAD");
        //     for (int j = 0; j < sim_config->hubs[i].incoming_storge_size; j++)
        //         if (j == (sim_config->hubs[i].incoming_storge_size - 1)) {
        //             printf("%p]\n", incoming_storages[i][j]);
        //         } else {
        //             printf("%p, ", incoming_storages[i][j]);
        //         }
        //     printf("             O [");
        //     for (int j = 0; j < sim_config->hubs[i].outgoing_storge_size; j++)
        //         if (j == (sim_config->hubs[i].outgoing_storge_size - 1)) {
        //             printf("%p]\n", outgoing_storages[i][j]);
        //         } else {
        //             printf("%p, ", outgoing_storages[i][j]);
        //         }
        //     UNLOCK_AND_CHECK(hub_outgoing_storage_mutexes[i])
        //     UNLOCK_AND_CHECK(hub_incoming_storage_mutexes[i])
        // }
        // UNLOCK_AND_CHECK(hub_info_mutex)
        // LOCK_AND_CHECK(drone_info_mutex)
        // for (int i = 0; i < sim_config->drones_count; i++) {
        //     printf("Drone-%d: stat: %s, hub: %d, info: %p", i+1, __drone_stat_chr(drone_info_registry[i]->stat), drone_info_registry[i]->hub_id, drone_info_registry[i]->info);
        //     if (drone_info_registry[i]->info != NULL) {
        //         printf("[curr hub: %d, next hub: %d, range: %d, pkg: %p]\n", drone_info_registry[i]->info->current_hub_id, drone_info_registry[i]->info->next_hub_id, drone_info_registry[i]->info->current_range, drone_info_registry[i]->info->packageInfo);
        //     } else {
        //         printf("\n");
        //     }
        // }
        // UNLOCK_AND_CHECK(drone_info_mutex)
        printf("====================[DEBUG END]====================\n");
        pthread_mutex_unlock(&debug_printf_mutex);

        usleep(200*1000);
    }
}

int main(int argc, char **argv, char **envp) {
    atexit(defer);

    __debug_file = fopen(DEBUG_LOG_FILE_NAME, "w");

    if (__debug_file == NULL) {
        perror("    fopen");
        exit(1);
    }

    FILE* config_fp;

    if (CONFIG_FILE_NAME == NULL)
        config_fp = stdin;
    else {
        config_fp = fopen(CONFIG_FILE_NAME, "r");

        if (config_fp == NULL) {
            perror("    fopen");
            exit(1);
        }
    }

    sim_config = parse_config_from_file(config_fp);

    if (CONFIG_FILE_NAME != NULL)
        fclose(config_fp);

    if (sim_config == NULL) {
        perror("    parse_config_from_file");
        exit(1);
    }

    if (DEBUG_SIMULATOR)
        dump_config(__debug_file, sim_config);
    
    init_mutexes(sim_config->hubs_count);
    init_critical_structures(sim_config);

    for (int i = 0; i < sim_config->drones_count; i++)
        charging_spaces_remaining[sim_config->drones[i].starting_hub_id-1]--;

    int rv;

    hub_pthreads = (pthread_t*) malloc(sizeof(pthread_t) * sim_config->hubs_count);
    sender_pthreads = (pthread_t*) malloc(sizeof(pthread_t) * sim_config->hubs_count);
    receiver_pthreads = (pthread_t*) malloc(sizeof(pthread_t) * sim_config->hubs_count);
    drone_pthreads = (pthread_t*) malloc(sizeof(pthread_t) * sim_config->drones_count);

    SenderThreadConfig sender_confs[sim_config->hubs_count];
    ReceiverThreadConfig receiver_confs[sim_config->hubs_count];
    HubThreadConfig hub_confs[sim_config->hubs_count];
    DroneThreadConfig drone_confs[sim_config->drones_count];

    __simulation_alive_threads_ctr = sim_config->drones_count + (sim_config->hubs_count * 3);
    __count_packages_waiting = 0;

    for (int i = 0; i < sim_config->hubs_count; i++)
        __count_packages_waiting += sim_config->hubs[i].sender.total_packages;

    InitWriteOutput();

    for (int i = 0; i < sim_config->drones_count; i++) {
        drone_confs[i].self = (DroneInfo*) malloc(sizeof(DroneInfo));
        drone_confs[i].self->id = sim_config->drones[i].drone_id;
        drone_confs[i].self->current_hub_id = sim_config->drones[i].starting_hub_id;
        drone_confs[i].self->current_range = sim_config->drones[i].maximum_range;
        drone_confs[i].self->next_hub_id = 0;
        drone_confs[i].self->packageInfo = NULL;

        drone_confs[i].self_config = &sim_config->drones[i];
        drone_confs[i].simulation_config = sim_config;

        drone_info_registry[i]->info = drone_confs[i].self;

        if ((rv = pthread_create(&drone_pthreads[i], NULL, drone_thread, &drone_confs[i])) != 0) {
            fprintf(stderr, " !!! pthread_create/drone: %d\n", rv);
            exit(1);
        }
    }

    for (int i = 0; i < sim_config->hubs_count; i++) {
        sender_confs[i].self = (SenderInfo*) malloc(sizeof(SenderInfo));
        sender_confs[i].self->id = sim_config->hubs[i].sender.sender_id;
        sender_confs[i].self->current_hub_id = sim_config->hubs[i].hub_id;
        sender_confs[i].self->packageInfo = NULL;
        sender_confs[i].self->remaining_package_count = sim_config->hubs[i].sender.total_packages;

        sender_confs[i].self_config = &sim_config->hubs[i].sender;
        sender_confs[i].simulation_config = sim_config;

        if ((rv = pthread_create(&sender_pthreads[i], NULL, sender_thread, &sender_confs[i])) != 0) {
            fprintf(stderr, " !!! pthread_create/sender: %d\n", rv);
            exit(1);
        }

        receiver_confs[i].self = (ReceiverInfo*) malloc(sizeof(ReceiverInfo));
        receiver_confs[i].self->id = sim_config->hubs[i].receiver.receiver_id;
        receiver_confs[i].self->current_hub_id = sim_config->hubs[i].receiver.hub_id;
        receiver_confs[i].self->packageInfo = NULL;

        receiver_confs[i].self_config = &sim_config->hubs[i].receiver;
        receiver_confs[i].simulation_config = sim_config;

        if ((rv = pthread_create(&receiver_pthreads[i], NULL, receiver_thread, &receiver_confs[i])) != 0) {
            fprintf(stderr, " !!! pthread_create/receiver: %d\n", rv);
            exit(1);
        }

        hub_confs[i].self = (HubInfo*) malloc(sizeof(HubInfo));
        hub_confs[i].self->id = sim_config->hubs[i].hub_id;

        hub_confs[i].self_config = &sim_config->hubs[i];
        hub_confs[i].simulation_config = sim_config;

        if ((rv = pthread_create(&hub_pthreads[i], NULL, hub_thread, &hub_confs[i])) != 0) {
            fprintf(stderr, " !!! pthread_create/hub: %d\n", rv);
            exit(1);
        }
    }

#if (SUPERLOGGING)
    pthread_t debugger_th;
    pthread_create(&debugger_th, NULL, debugger_thread, NULL);  
#endif

    for (int i = 0; i < sim_config->hubs_count; i++) {
        if ((rv = pthread_join(sender_pthreads[i], NULL)) != 0) {
            fprintf(stderr, " !!! pthread_join/sender: %d\n", rv);
            exit(1);
        }

        if ((rv = pthread_join(receiver_pthreads[i], NULL)) != 0) {
            fprintf(stderr, " !!! pthread_join/receiver: %d\n", rv);
            exit(1);
        }

        if ((rv = pthread_join(hub_pthreads[i], NULL)) != 0) {
            fprintf(stderr, " !!! pthread_join/hub: %d\n", rv);
            exit(1);
        }
    }
    
    for (int i = 0; i < sim_config->drones_count; i++)
        if ((rv = pthread_join(drone_pthreads[i], NULL)) != 0) {
            fprintf(stderr, " !!! pthread_join/drone: %d\n", rv);
            exit(1);
        }
        
#if (SUPERLOGGING)
    pthread_join(debugger_th, NULL);
#endif

    exit(0);
}