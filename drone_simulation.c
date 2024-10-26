#include <stdio.h>
#include <png.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <string.h>



#define MAX_LINE_LENGTH 100
#define MAX_DRONES 100
#define MAX_NEIGHBORS 10
#define alpha 0.05
#define Vmax 50
#define POSITION_THRESHOLD 0.1
#define MIN_DISTANCE 2.0  // Minimum allowable distance between drones

typedef struct {
    int id;
    float x, y, z; // position
    float coverage;
    float vx, vy, vz; // velocity
    float camera_resolution;
    int neighbors[MAX_NEIGHBORS]; // list of neighbors
    int neighbor_count;
    int is_active; // 1 if drone is active, 0 if destroyed
    png_bytep *image; // Holds the captured image from this drone

} Drone;

typedef struct
{
 float xMin,xMax,yMin,yMax;
 float width,height;
 float grid_size1;

}Image;
typedef struct {
    float xMin, xMax, yMin, yMax;
    int width, height;
} MapArea;


 int is_position_occupied(float x, float y, float z, int moving_drone_id);  
 void voronoi_adapt_all_drones(int moved_drone_id,Image*img1);  
 void adapt_drones(int moved_drone_id,float xold,float yold,float zold,Image*img1,int num_drones);  
 void avoid_collisions(); 
 float calculate_distance(Drone* d1, Drone* d2);
 float random_float(float min, float max);  
 float go_to(float x, float y, float z, int id,Image*img1,int num_drones);
 float* speed(float vx, float vy, float vz, int id, float delta_t,Image*img1,int num_drones);
 float calculate_coverage(float altitude);
 Image spread_drones(int num_drones, float xmin, float ymin, float xmax, float ymax, float comm_range, float cam_res,Image Im1);
void capture_image_segment(Drone *drone, png_bytep *map, int map_width, int map_height, Image area);





#pragma pack(push, 1)
typedef struct {
    uint16_t bfType;
    uint32_t bfSize;
    uint16_t bfReserved1;
    uint16_t bfReserved2;
    uint32_t bfOffBits;
} BMPHeader;

typedef struct {
    uint32_t biSize;
    int32_t  biWidth;
    int32_t  biHeight;
    uint16_t biPlanes;
    uint16_t biBitCount;
    uint32_t biCompression;
    uint32_t biSizeImage;
    int32_t  biXPelsPerMeter;
    int32_t  biYPelsPerMeter;
    uint32_t biClrUsed;
    uint32_t biClrImportant;
} BMPInfoHeader;
#pragma pack(pop)

Drone drones[MAX_DRONES];
int drone_count = 0;
void *safe_malloc(size_t size) {
    void *ptr = malloc(size);
    if (!ptr) {
        fprintf(stderr, "Error: Memory allocation failed.\n");
        exit(EXIT_FAILURE);
    }
    return ptr;
}

// Function to calculate the coverage area of a drone
float calculate_coverage(float altitude) {
   return pow(2 * altitude * tan(alpha), 2);
}
// Function to check if two drones are within communication range
int within_communication_range(Drone* d1, Drone* d2) {
    float dx = d1->x - d2->x;
    float dy = d1->y - d2->y;
    float dz = d1->z - d2->z;
    float distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance <= d1->coverage;
}
// Function to update the neighbors for each drone
void update_neighbors() {
    for (int i = 0; i < drone_count; i++) {
        drones[i].neighbor_count = 0; // Reset the neighbor list

        for (int j = 0; j < drone_count; j++) {
            if ((i != j)&& (within_communication_range(&drones[i], &drones[j]))) {
                drones[i].neighbors[drones[i].neighbor_count] = drones[j].id;
                drones[i].neighbor_count++;
            }
        }
    }
}
// Function to initialize drones and spread them over a defined area
Image spread_drones(int num_drones, float xmin, float ymin, float xmax, float ymax, float comm_range, float cam_res, Image Im1) {
    float total_area = (xmax - xmin) * (ymax - ymin);
    float c = sqrt(total_area / num_drones);
    float h = c / (2 * tan(alpha));

    Im1.xMin = xmin;
    Im1.xMax = xmax;
    Im1.yMin = ymin;
    Im1.yMax = ymax;
    Im1.width = xmax - xmin;
    Im1.height = ymax - ymin;

    int drones_per_row = (int)ceil((xmax - xmin) / c);
    int drones_per_column = (int)ceil((ymax - ymin) / c);

    int drone_id = 0;
    for (int i = 0; i < drones_per_row && drone_id < num_drones; i++) {
        for (int j = 0; j < drones_per_column && drone_id < num_drones; j++) {
            float x = xmin + i * c + c / 2;
            float y = ymin + j * c + c / 2;
            float z = h;

            // Initialize the unique drone
            drones[drone_id].id = drone_id;  // Unique ID for each drone
            drones[drone_id].x = x;
            drones[drone_id].y = y;
            drones[drone_id].z = z;
            drones[drone_id].vx = 0;
            drones[drone_id].vy = 0;
            drones[drone_id].vz = 0;
            drones[drone_id].coverage = comm_range;
            drones[drone_id].camera_resolution = cam_res;
            drones[drone_id].is_active = 1;

            // Set coverage based on altitude
            drones[drone_id].coverage = pow(2 * z * tan(alpha), 2);
            printf("Drone %d coverage area set to %.2f\n", drone_id, drones[drone_id].coverage);

            drone_id++;  // Increment to the next drone
        }
    }

    drone_count = drone_id;
    return Im1;
}










// Function to adjust neighboring drones to cover for a destroyed drone
void adjust_neighbors_for_destroyed_drone(int destroyed_drone_id) {
    Drone destroyed_drone = drones[destroyed_drone_id];
    
    // Loop through all drones and check if any are neighbors of the destroyed drone
    for (int i = 0; i < drone_count; i++) {
        if ((drones[i].is_active) && (drones[i].neighbor_count > 0)) {
            for (int j = 0; j < drones[i].neighbor_count; j++) {
                if ((drones[i].neighbors[j])== (destroyed_drone.id)) {
                    // Calculate new position (move closer to destroyed drone's position)
                    // drones[i].x = (drones[i].x + destroyed_drone.x) / 2;
                    // drones[i].y = (drones[i].y + destroyed_drone.y) / 2;
                    
                    // Calculate new coverage area (cover own area + destroyed drone's area)
                    float new_coverage = drones[i].coverage + destroyed_drone.coverage;
                    
                    // Recalculate the altitude needed for new coverage area
                    drones[i].z = sqrt(new_coverage) / (2 * tan(alpha));

                    // Update coverage area for the neighbor
                    drones[i].coverage = calculate_coverage(drones[i].z);
                    
                    printf("Drone %d adapted its position to (%.2f, %.2f, %.2f) and coverage area to %.2f m^2 to cover for destroyed Drone %d.\n", 
                        drones[i].id, drones[i].x, drones[i].y, drones[i].z, drones[i].coverage, destroyed_drone.id);
                }
            }
        }
    }
}

// Function to check for destroyed drones and adapt neighbors accordingly
void check_and_adapt() {
    for (int i = 0; i < drone_count; i++) {
        if (!drones[i].is_active) {
            printf("Drone %d is destroyed, adapting neighbors...\n", drones[i].id);
            adjust_neighbors_for_destroyed_drone(i);
            update_neighbors();
        }
     }
}

// Function to print the positions of all drones and their coverage areas
// void print_drone_positions() {
//     for (int i = 0; i < drone_count; i++) {
//         printf("Drone %d: Position (x=%.2f, y=%.2f, z=%.2f), Coverage Area = %.2f m^2\n", 
//             drones[i].id, drones[i].x, drones[i].y, drones[i].z, drones[i].coverage);
//     }
// }
void print_drone_infos() {
    for (int i = 0; i < drone_count; i++) {
        printf("Drone %d:\n", drones[i].id);
        printf("  Position (x=%.3f, y=%.3f, z=%.3f)\n", drones[i].x, drones[i].y, drones[i].z);
        printf("  Coverage Area = %.3f m^2\n", drones[i].coverage);
        printf("  Velocity (vx=%.3f, vy=%.3f, vz=%.3f)\n", drones[i].vx, drones[i].vy, drones[i].vz);
        printf("  Communication Range = %.3f meters\n", drones[i].coverage);
        printf("  Camera Resolution = %.3f pixels\n", drones[i].camera_resolution);
        printf("  Active Status = %s\n", drones[i].is_active ? "Active" : "Destroyed");
        printf("\n");
    }
}

void destruc(int id){
    Drone *d = &drones[id-1];
    d->is_active=0;
}
void altitude_to_color(float altitude, uint8_t* r, uint8_t* g, uint8_t* b) {
    if (altitude < 20) {
        *r = 255; *g = 0; *b = 0; // Red for low altitude
    } else if (altitude < 40) {
        *r = 255; *g = 165; *b = 0; // Orange
    } else if (altitude < 60) {


*r = 255; *g = 255; *b = 0; // Yellow
    } else if (altitude < 80) {
        *r = 0; *g = 255; *b = 0; // Green
    } else {
        *r = 0; *g = 0; *b = 255; // Blue for high altitude
    }
}
int is_position_occupied(float x, float y, float z, int moving_drone_id) {
    for (int i = 0; i < drone_count; i++) {
        if ((i != moving_drone_id) && (drones[i].is_active)) {
            float dx = x - drones[i].x;
            float dy = y - drones[i].y;
            float dz = z - drones[i].z;
            if (sqrt(dx * dx + dy * dy + dz * dz) < POSITION_THRESHOLD) {
                return i;  // Return the ID of the drone that occupies the space
            }
        }
    }
    return -1;  // No drone occupies the space
}

// Function to calculate the centroid of all active drones' positions
void calculate_centroid(float* centroid_x, float* centroid_y, float* centroid_z) {
    float sum_x = 0, sum_y = 0, sum_z = 0;
    int active_drones = 0;

    for (int i = 0; i < drone_count; i++) {
        if (drones[i].is_active) {
            sum_x += drones[i].x;
            sum_y += drones[i].y;
            sum_z += drones[i].z;
            active_drones++;
        }
    }

    if (active_drones > 0) {
        *centroid_x = sum_x / active_drones;
        *centroid_y = sum_y / active_drones;
        *centroid_z = sum_z / active_drones;
    }
}

// Random float generator within a range
float random_float(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}

// Function to normalize a vector
void normalize_vector(float* vx, float* vy, float* vz) {
    float magnitude = sqrt((*vx) * (*vx) + (*vy) * (*vy) + (*vz) * (*vz));
    
    // Avoid division by zero
    if (magnitude != 0) {
        *vx /= magnitude;
        *vy /= magnitude;
        *vz /= magnitude;
    }
}

// Function to calculate distance between two drones
float calculate_distance(Drone* d1, Drone* d2) {
    return sqrt(pow(d1->x - d2->x, 2) + pow(d1->y - d2->y, 2) + pow(d1->z - d2->z, 2));
}

// Function to avoid overlap between drones
void avoid_overlap(Drone* d) {
    for (int j = 0; j < drone_count; j++) {
        if (d != &drones[j] && drones[j].is_active) {
            float distance = calculate_distance(d, &drones[j]);

            // If the drones are too close, apply a repulsion force
            if (distance < MIN_DISTANCE) {
                float repulsion_strength = MIN_DISTANCE - distance;
                
                // Calculate the repulsion vector
                float repulsion_x = d->x - drones[j].x;
                float repulsion_y = d->y - drones[j].y;
                float repulsion_z = d->z - drones[j].z;

                // Normalize the repulsion vector
                normalize_vector(&repulsion_x, &repulsion_y, &repulsion_z);

                // Apply the repulsion to push the drone away from the other
                d->x += repulsion_x * repulsion_strength;
                d->y += repulsion_y * repulsion_strength;
                d->z += repulsion_z * repulsion_strength;
            }
        }
    }
}
void adapt_drones(int moved_drone_id, float xold, float yold, float zold, Image* img1, int num_drones) {
    Drone* current_drone = &drones[moved_drone_id - 1];  // Start with the moved drone
    int adjustments_made;  // Flag to track if any adjustments were made in an iteration

    // Step 1: Adjust along X-axis (left or right)
    do {
        adjustments_made = 0;  // Reset at the start of each loop
        for (int i = 0; i < num_drones; i++) {
            if (i != moved_drone_id - 1) {
                Drone* d = &drones[i];  // Current drone being checked

                // If the current drone is too close in the X-axis
                if ((fabs(d->x - current_drone->x) < (img1->grid_size1 / 2))&&(fabs(d->y - current_drone->y) < (img1->grid_size1 / 2))&&(fabs(d->z - current_drone->z) < (img1->grid_size1 / 2))) {
                    // If current_drone's old X was greater, move the current drone right
                    if ((xold > d->x)&&(fabs(d->x - xold) > (img1->grid_size1 / 2))) {


d->x += img1->grid_size1;  // Move right
                        current_drone = &drones[i];  // Move focus to the current drone
                        moved_drone_id=i+1;
                        adjustments_made = 1;  // Mark that an adjustment was made
                    }
                    // If current_drone's old X was smaller, move the current drone left
                   if ((xold < d->x)&&(fabs(d->x - xold) > (img1->grid_size1 / 2))) {
                        d->x -= img1->grid_size1;  // Move left
                        current_drone = &drones[i];  // Move focus to the current drone
                        moved_drone_id=i+1;
                        adjustments_made = 1;  // Mark that an adjustment was made
                    }
                     if (fabs(d->x - xold) < (img1->grid_size1 / 2)) {
                        current_drone = &drones[i];  // Move focus to the current drone
                        moved_drone_id=i+1;
                        adjustments_made = 0;  // Mark that an adjustment was made
                    }
                    
                    
                }
            }
        }
    } while (adjustments_made);  // Repeat until no further adjustments in the X-axis

    // Step 2: Adjust along Y-axis (forward or backward)
    do {
        adjustments_made = 0;  // Reset at the start of each loop
        for (int i = 0; i < num_drones; i++) {
            if (i != moved_drone_id - 1) {
                Drone* d = &drones[i];  // Current drone being checked
                if ((fabs(d->x - current_drone->x) < (img1->grid_size1 / 2))&&(fabs(d->y - current_drone->y) < (img1->grid_size1 / 2))&&(fabs(d->z - current_drone->z) < (img1->grid_size1 / 2))) {

                  if ((yold > d->y)&&(fabs(d->y - yold) > (img1->grid_size1 / 2))) {
                        d->y += img1->grid_size1;  // Move right
                        current_drone = &drones[i];  // Move focus to the current drone
                        moved_drone_id=i+1;
                        adjustments_made = 1;  // Mark that an adjustment was made
                    }
                    // If current_drone's old X was smaller, move the current drone left
                   if ((yold < d->y)&&(fabs(d->y - yold) > (img1->grid_size1 / 2))) {
                        d->y -= img1->grid_size1;  // Move left
                        current_drone = &drones[i];  // Move focus to the current drone
                        moved_drone_id=i+1;
                        adjustments_made = 1;  // Mark that an adjustment was made
                    }
                     if (fabs(d->y - yold) < (img1->grid_size1 / 2)) {
                        current_drone = &drones[i];  // Move focus to the current drone
                        moved_drone_id=i+1;
                        adjustments_made = 0;  // Mark that an adjustment was made
                    }
                }
            }
        }
    } while (adjustments_made);  // Repeat until no further adjustments in the Y-axis

    // Step 3: Adjust along Z-axis (up or down)
    do {
        adjustments_made = 0;  // Reset at the start of each loop
        for (int i = 0; i < num_drones; i++) {
            if (i != moved_drone_id - 1) {
                Drone* d = &drones[i];  // Current drone being checked
             if ((fabs(d->x - current_drone->x) < (img1->grid_size1 / 2))&&(fabs(d->y - current_drone->y) < (img1->grid_size1 / 2))&&(fabs(d->z - current_drone->z) < (img1->grid_size1 / 2))) {

                if ((zold > d->z)&&(fabs(d->z - zold) > (img1->grid_size1 / 2))) {
                        d->z += img1->grid_size1;  // Move right
                        current_drone = &drones[i];  // Move focus to the current drone
                        moved_drone_id=i+1;
                        adjustments_made = 1;  // Mark that an adjustment was made
                    }
                    // If current_drone's old X was smaller, move the current drone left
                   if ((zold < d->z)&&(fabs(d->z - yold) > (img1->grid_size1 / 2))) {


d->z -= img1->grid_size1;  // Move left
                        current_drone = &drones[i];  // Move focus to the current drone
                        moved_drone_id=i+1;
                        adjustments_made = 1;  // Mark that an adjustment was made
                    }
                     if (fabs(d->z - zold) < (img1->grid_size1 / 2)) {
                        current_drone = &drones[i];  // Move focus to the current drone
                        moved_drone_id=i+1;
                        adjustments_made = 0;  // Mark that an adjustment was made
                    }
                }
            }
        }
    } while (adjustments_made);  // Repeat until no further adjustments in the Z-axis

    // Update neighbors after drone adjustment
    update_neighbors();
}

// Function to calculate new positions using Voronoi-like partitioning (adjust drones to optimize coverage)
void voronoi_adapt_all_drones(int moved_drone_id, Image* img1) {
    printf("Voronoi-based intelligent adaptation after drone %d moved...\n", moved_drone_id);
   

    // Calculate the centroid of the current drone positions
    float centroid_x, centroid_y, centroid_z;
    calculate_centroid(&centroid_x, &centroid_y, &centroid_z);

    // Now, for each drone, calculate an optimal position to spread it out
    for (int i = 0; i < drone_count; i++) {
        if ((i != moved_drone_id - 1) && drones[i].is_active) {
            Drone* d = &drones[i];

            // Calculate the vector from the centroid to the current drone position
            float vx = d->x - centroid_x;
            float vy = d->y - centroid_y;
            float vz = d->z - centroid_z;

            // Normalize the vector
            normalize_vector(&vx, &vy, &vz);

            // Move the drone along this vector to spread it out
            float offset_distance = random_float(1.0, 3.0);
            d->x += vx * offset_distance;
            d->y += vy * offset_distance;
            d->z += vz * offset_distance;

            // Ensure drones stay within bounds after movement
            if (d->x < img1->xMin) d->x = img1->xMin;
            if (d->x > img1->xMax) d->x = img1->xMax;
            if (d->y < img1->yMin) d->y = img1->yMin;
            if (d->y > img1->yMax) d->y = img1->yMax;
            if (d->z < 0) d->z = 0;

            // Apply overlap avoidance logic
            avoid_overlap(d);

            printf("Drone %d adjusted to new position (%.2f, %.2f, %.2f) using Voronoi partitioning.\n",
                   d->id, d->x, d->y, d->z);
        }
    }

    // Update neighbors after drone adjustment
    update_neighbors();
}

// Function to avoid collisions by ensuring drones are spaced out
void avoid_collisions() {
    for (int i = 0; i < drone_count; i++) {
        for (int j = i + 1; j < drone_count; j++) {
            if ((drones[i].is_active) && (drones[j].is_active)) {
                float distance = calculate_distance(&drones[i], &drones[j]);

                if (distance < POSITION_THRESHOLD) {
                    // Calculate the vector between the two drones
                    float dx = drones[i].x - drones[j].x;
                    float dy = drones[i].y - drones[j].y;
                    float dz = drones[i].z - drones[j].z;

                    // Normalize the vector
                    float magnitude = sqrt(dx * dx + dy * dy + dz * dz);
                    dx /= magnitude;
                    dy /= magnitude;
                    dz /= magnitude;

                    // Move the drones away from each other slightly
                    drones[i].x += dx * 0.5;
                    drones[i].y += dy * 0.5;
                    drones[i].z += dz * 0.5;

                    drones[j].x -= dx * 0.5;
                    drones[j].y -= dy * 0.5;
                    drones[j].z -= dz * 0.5;

                    printf("Collision avoided between Drone %d and Drone %d.\n", drones[i].id, drones[j].id);
                }
            }
        }
    }
}


// Command structure
typedef struct Command {
    char type[MAX_LINE_LENGTH];
    char args[MAX_LINE_LENGTH];
    struct Command* next;
} Command;

// Function to add a command to the list (queue)
Command* add_command(Command* head, const char* type, const char* args) {
    Command* new_command = (Command*)malloc(sizeof(Command));
    strcpy(new_command->type, type);
    strcpy(new_command->args, args);
    new_command->next = head;
    return new_command;
}

// Function to free the command list (queue)
void free_commands(Command* head) {
    Command* current = head;
    while (current != NULL) {
        Command* temp = current;
        current = current->next;
        free(temp);
    }
}
void free_drone_images(int num_drones) {
    for (int i = 0; i < num_drones; i++) {
        if (drones[i].image != NULL) {
            printf("Freeing drone %d image_data at address %p\n", i, (void*)drones[i].image);
            free(drones[i].image);
            drones[i].image = NULL;  // Set to NULL to prevent double free
        }
    }
}



// Function to execute all queued commands
void execute_commands(Command* head, float t_max, Image* img1,int num_drones) {
    Command* current = head;

    // Iterate through all commands in the queue and execute them
    while (current != NULL) {
        if ((strcmp(current->type, "goto")) == 0) {
            int id;
            float x, y, z;
            sscanf(current->args, "%d %f %f %f", &id, &x, &y, &z);
            float time_taken = go_to(x, y, z, id, img1, num_drones);
            printf("Time taken for drone %d to reach destination: %.2f\n", id, time_taken);


        } else if ((strcmp(current->type, "speed")) == 0) {
            int id;
            float vx, vy, vz, delta_t = 0;

            sscanf(current->args, "%d %f %f %f %f", &id, &vx, &vy, &vz, &delta_t);

            // If delta_t is not provided in the command, use global t_max
            if (delta_t == 0) {
                delta_t = t_max;
            }

            float* new_coords = speed(vx, vy, vz, id, delta_t, img1,num_drones);
            if (new_coords) {
                printf("Drone %d moved to new position: x = %f, y = %f, z = %f\n", id, new_coords[0], new_coords[1], new_coords[2]);
                free(new_coords);  // Free the memory after use
            }
        }

        current = current->next;  // Move to the next command
    }
}

// First pass to calculate global t_max
float find_global_tmax(const char* filename) {
    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        return -1;
    }

    char line[MAX_LINE_LENGTH];
    float global_t_max = 0.0;

    // Iterate over all lines to find the maximum time taken by goto commands
    while (fgets(line, sizeof(line), file)) {
        char type[MAX_LINE_LENGTH];
        char args[MAX_LINE_LENGTH];

        if ((sscanf(line, "%s %[^\n]", type, args)) == 2) {
            if ((strcmp(type, "goto")) == 0) {
                int id;
                float x, y, z;
                sscanf(args, "%d %f %f %f", &id, &x, &y, &z);
                
                // Calculate the time for this goto command
                // (We're not calling go_to because we only need the time, not to execute)
                float dx = x - drones[id].x;
                float dy = y - drones[id].y;
                float dz = z - drones[id].z;
                float distance = sqrt(dx * dx + dy * dy + dz * dz);
                float time_taken = distance / Vmax;

                // Update global t_max
                if (time_taken > global_t_max) {
                    global_t_max = time_taken;
                }
            }
        }
    }

    fclose(file);
    return global_t_max;
}

// Function to process the file and queue commands
void process_file(const char* filename, Image* img1,int num_drones) {
    // First, find the global t_max by scanning the file
    float t_max = find_global_tmax(filename);
    if (t_max < 0) {
        printf("Error in finding global t_max.\n");
        return;
    }

    FILE* file = fopen(filename, "r");
    if (!file) {
        perror("Error opening file");
        return;
    }

    char line[MAX_LINE_LENGTH];
    Command* command_queue = NULL;
    // Second pass: Read the file, queue commands, and execute on step
    while (fgets(line, sizeof(line), file)) {
        char type[MAX_LINE_LENGTH];
        char args[MAX_LINE_LENGTH];

        // Parse the command type and arguments
        if ((sscanf(line, "%s %[^\n]", type, args)) == 2) {

if (((strcmp(type, "goto")) == 0) || ((strcmp(type, "speed")) == 0)) {
                // Add the goto or speed command to the queue
                command_queue = add_command(command_queue, type, args);
            } else if ((strcmp(type, "step")) == 0) {
                // When we encounter a step, execute all queued commands
                printf("Executing step...\n");
                execute_commands(command_queue, t_max, img1,num_drones);

                // Clear the command queue after execution
                free_commands(command_queue);
                command_queue = NULL;
            }
        }
    }

    // Clean up the last batch of commands if a step wasn't called at the end
    if (command_queue != NULL) {
        printf("Executing remaining commands...\n");
        execute_commands(command_queue, t_max, img1,num_drones);
        free_commands(command_queue);
    }

    fclose(file);
}

float go_to(float x, float y, float z, int id, Image* img1, int num_drones) {
    if (id < 1 || id > drone_count || !drones[id - 1].is_active) {
        printf("Error: Invalid drone ID or drone is not active.\n");
        return -1;
    }
    Drone* d = &drones[id - 1];
    float old_x = d->x, old_y = d->y, old_z = d->z;

    // Check if the destination is within bounds; if not, clamp to boundaries
    if (x < img1->xMin) x = img1->xMin;
    if (x > img1->xMax) x = img1->xMax;
    if (y < img1->yMin) y = img1->yMin;
    if (y > img1->yMax) y = img1->yMax;
    if (z < 0) z = 0; // Assume ground level is 0 and disallow negative altitudes

    // Calculate distance and time
    float dx = x - d->x;
    float dy = y - d->y;
    float dz = z - d->z;
    float distance = sqrt(dx * dx + dy * dy + dz * dz);
    float time_taken = distance / Vmax;

    // Update position
    d->x = x;
    d->y = y;
    d->z = z;

    adapt_drones(id, old_x, old_y, old_z, img1, num_drones);

    return time_taken;
}

float* speed(float vx, float vy, float vz, int id, float delta_t, Image* img1, int num_drones) {
    if (id < 1 || id > drone_count || !drones[id - 1].is_active) {
        printf("Error: Invalid drone ID or drone is not active.\n");
        return NULL;
    }

    Drone* d = &drones[id - 1];
    float old_x = d->x, old_y = d->y, old_z = d->z;

    float speed_magnitude = sqrt(vx * vx + vy * vy + vz * vz);
    if (speed_magnitude > Vmax) {
        float scale_factor = Vmax / speed_magnitude;
        vx *= scale_factor;
        vy *= scale_factor;
        vz *= scale_factor;
    }

    // Calculate new position
    float xnew = vx * delta_t + d->x;
    float ynew = vy * delta_t + d->y;
    float znew = vz * delta_t + d->z;

    // Clamp to bounds if out of range
    if (xnew < img1->xMin) xnew = img1->xMin;
    if (xnew > img1->xMax) xnew = img1->xMax;
    if (ynew < img1->yMin) ynew = img1->yMin;
    if (ynew > img1->yMax) ynew = img1->yMax;
    if (znew < 0) znew = 0;

    d->x = xnew;
    d->y = ynew;
    d->z = znew;

    float* new_coords = (float*)malloc(3 * sizeof(float));
    if (!new_coords) {
        printf("Error: Memory allocation failed.\n");
        return NULL;
    }

    new_coords[0] = xnew;
    new_coords[1] = ynew;
    new_coords[2] = znew;

    adapt_drones(id, old_x, old_y, old_z, img1, num_drones);

    return new_coords;
}
void zoom_out(float z,int id,Image*img1,int num_drones){
    Drone *d = &drones[id];
    go_to(d->x,d->y,z,id,img1,num_drones);


}
png_bytep *load_png_image(const char *filename, int *width, int *height) {
    FILE *fp = fopen(filename, "rb");
    if (!fp) {
        fprintf(stderr, "Could not open file %s for reading.\n", filename);
        return NULL;
    }

    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) {
        fclose(fp);
        fprintf(stderr, "Failed to create png read struct.\n");
        return NULL;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_read_struct(&png, NULL, NULL);
        fclose(fp);
        fprintf(stderr, "Failed to create png info struct.\n");
        return NULL;
    }

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_read_struct(&png, &info, NULL);
        fclose(fp);
        fprintf(stderr, "Error during png creation.\n");
        return NULL;
    }

    png_init_io(png, fp);
    png_read_info(png, info);

    *width = png_get_image_width(png, info);
    *height = png_get_image_height(png, info);

    png_bytep *row_pointers = (png_bytep *)malloc(sizeof(png_bytep) * (*height));
    for (int y = 0; y < *height; y++) {
        row_pointers[y] = (png_byte *)malloc(png_get_rowbytes(png, info));
    }

    png_read_image(png, row_pointers);

    png_destroy_read_struct(&png, &info, NULL);
    fclose(fp);

    return row_pointers;
}


// Function to capture an image segment based on drone position

void capture_image_segment(Drone *drone, png_bytep *map, int map_width, int map_height, Image area) {
    (void)map;  // Mark 'map' as unused to avoid the warning

    // Calculate capture width and height based on drone's coverage and map dimensions
    int capture_width = (int)(area.width * drone->coverage / (map_width * map_height));
    int capture_height = (int)(area.height * drone->coverage / (map_width * map_height));

    // Ensure non-zero dimensions
    capture_width = (capture_width > 0) ? capture_width : 1;
    capture_height = (capture_height > 0) ? capture_height : 1;

    // Allocate memory for the image segment
    drone->image = malloc((int)drone->coverage * (int)drone->coverage * sizeof(png_byte));
    if (!drone->image) {
        fprintf(stderr, "Memory allocation failed for drone %d.\n", drone->id);
        exit(1);
    }

    for (int y = 0; y < capture_height; y++) {
        drone->image[y] = (png_bytep)malloc(capture_width * 4); // Assuming RGBA (4 bytes per pixel)
        if (!drone->image[y]) {
            fprintf(stderr, "Memory allocation failed at row %d for drone %d.\n", y, drone->id);
            // Free any previously allocated rows to prevent memory leaks
            for (int k = 0; k < y; k++) {
                free(drone->image[k]);
            }
            free(drone->image);
            exit(1);
        }
    }

    printf("Drone %d captured an image segment with dimensions %d x %d.\n", drone->id, capture_width, capture_height);
}


void stretch(float x,float y,int id,Image*img1,int num_drones){
    Drone *d = &drones[id];
    go_to(x,y,d->z,id,img1,num_drones);

}
void printneighbors(int id){
      Drone *d = &drones[id];
      printf("Neighbors of Drone %d:\n", id);
    for (int i = 0; i < d->neighbor_count; i++) {
        int neighbor_id = d->neighbors[i];
        printf("Neighbor ID: %d, Position: (%.2f, %.2f, %.2f)",
        neighbor_id, drones[neighbor_id].x, drones[neighbor_id].y, drones[neighbor_id].z);
       
    }
 }
    

void merge_images(Drone drones[], int num_drones, int final_width, int final_height) {
    png_bytep *final_image = (png_bytep *)malloc(sizeof(png_bytep) * final_height);
    for (int y = 0; y < final_height; y++) {
        final_image[y] = (png_byte *)calloc(final_width * 4, sizeof(png_byte)); // Allocate for RGBA per pixel
    }

    for (int i = 0; i < num_drones; i++) {
        Drone *drone = &drones[i];
        int capture_size = (int)sqrt(drone->coverage);
        int start_x = (int)(drone->x - capture_size / 2);
        int start_y = (int)(drone->y - capture_size / 2);

        for (int y = 0; y < capture_size; y++) {
            for (int x = 0; x < capture_size; x++) {
                int final_x = start_x + x;
                int final_y = start_y + y;

                if (final_x >= 0 && final_x < final_width && final_y >= 0 && final_y < final_height) {
                    memcpy(&final_image[final_y][final_x * 4], &drone->image[y][x * 4], 4); // Copy RGBA data
                }
            }
        }
    }

    // Free memory for final image
    for (int y = 0; y < final_height; y++) {
        free(final_image[y]);
    }
    free(final_image);
}




   int main() {
    int num_drones;
    Image Im1;
    float xMin, yMin, xMax, yMax, comm_range, cam_res;
    const char *map_filename = "Photo.png";  // Path to the map image file

    printf("Enter the number of drones: ");
    scanf("%d", &num_drones);
    if (num_drones > MAX_DRONES) {
        printf("Error: Number of drones exceeds maximum limit of %d.\n", MAX_DRONES);
        return 1;
    }

    printf("Enter the area dimensions (xMin yMin xMax yMax) [Ensure xMax > xMin and yMax > yMin]: ");
    scanf("%f %f %f %f", &xMin, &yMin, &xMax, &yMax);

    if (xMax <= xMin || yMax <= yMin) {
        printf("Error: xMax should be greater than xMin and yMax should be greater than yMin.\n");
        return 1;
    }

    printf("Enter the communication range: ");
    scanf("%f", &comm_range);
    printf("Enter the camera resolution: ");
    scanf("%f", &cam_res);

    // Initialize the area and spread the drones
    Im1 = spread_drones(num_drones, xMin, yMin, xMax, yMax, comm_range, cam_res, Im1);

    int map_width, map_height;
    png_bytep *map_image = load_png_image(map_filename, &map_width, &map_height);
    if (!map_image) {
        printf("Error: Could not load the map image.\n");
        return 1;
    }

    // Capture image segments for each drone
    for (int i = 0; i < num_drones; i++) {
        capture_image_segment(&drones[i], map_image, map_width, map_height, Im1);
        printf("Drone %d captured an image segment with dimensions %d x %d.\n", drones[i].id, 
               (int)drones[i].coverage, (int)drones[i].coverage);
    }

    // Merge the captured images into a final output image
   void merge_images(Drone drones[], int num_drones, int final_width, int final_height) {
    // Allocate final image array (RGBA format assumed)
    png_bytep *final_image = (png_bytep *)malloc(sizeof(png_bytep) * final_height);
    for (int y = 0; y < final_height; y++) {
        final_image[y] = (png_byte *)calloc(final_width * 4, sizeof(png_byte)); // Allocate for RGBA per pixel
    }

    // Copy each drone's image data to the final image at appropriate locations
    for (int i = 0; i < num_drones; i++) {
        Drone *drone = &drones[i];
        int capture_size = (int)sqrt(drone->coverage);
        int start_x = (int)(drone->x - capture_size / 2);
        int start_y = (int)(drone->y - capture_size / 2);

        for (int y = 0; y < capture_size; y++) {
            for (int x = 0; x < capture_size; x++) {
                int final_x = start_x + x;
                int final_y = start_y + y;

                // Ensure we are within final image boundaries
                if (final_x >= 0 && final_x < final_width && final_y >= 0 && final_y < final_height) {
                    memcpy(&final_image[final_y][final_x * 4], &drone->image[y][x * 4], 4); // Copy RGBA data
                }
            }
        }
    }

    // Optionally add code to save or process final_image here

    // Free final_image memory
    for (int y = 0; y < final_height; y++) {
        free(final_image[y]);
    }
    free(final_image);
}
   }
    // Process the command file
    process_file("commands.txt", &Im1,num_drones);

    // Free memory for the map image
    for (int y = 0; y < map_height; y++) {
        free(map_image[y]);
    }
    free(map_image);

    // Free memory for the drone images
    free_drone_images(num_drones);

    return 0;



