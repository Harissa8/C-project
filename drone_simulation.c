#include "drone_simulation.h"
#include <time.h>

static int next_id = 0;

int generate_unique_id() {
    // Generate a unique ID without reseeding srand repeatedly
    return next_id++;
}

Drone *create_drone(Vector3D position, Vector3D velocity, double sphere_radius, double battery_level, double battery_drain_rate) {
    Drone *drone = (Drone *)malloc(sizeof(Drone));
    drone->id = generate_unique_id();
    drone->position = position;
    drone->initial_position = position;
    drone->velocity = velocity;
    drone->sphere_radius = sphere_radius;
    drone->battery_level = battery_level;
    drone->is_dead = false;
    drone->battery_drain_rate = battery_drain_rate;
    return drone;
}

void update_drone_position(Drone *drone, double dt) {
    if (drone->is_dead) return;  // Do not update if drone is dead

    drone->position.x += drone->velocity.x * dt;
    drone->position.y += drone->velocity.y * dt;
    drone->position.z += drone->velocity.z * dt;
    drone->battery_level -= drone->battery_drain_rate * dt;

    if (drone->battery_level <= 0) {
        drone->is_dead = true;
        drone->position = drone->initial_position;  // Reset position to initial state
        printf("Drone %d has returned to the initial position.\n", drone->id);
    } else {
        printf("Drone %d position: (%f, %f, %f), Battery level: %f\n", 
                drone->id, drone->position.x, drone->position.y, drone->position.z, drone->battery_level);
    }
}

double distance_between_drones(Drone *drone1, Drone *drone2) {
    double dx = drone1->position.x - drone2->position.x;
    double dy = drone1->position.y - drone2->position.y;
    double dz = drone1->position.z - drone2->position.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

void update_neighbors(Drone *drone, Drone *drones[], int num_drones) {
    for (int i = 0; i < num_drones; i++) {
        if (drone != drones[i] && distance_between_drones(drone, drones[i]) <= drone->sphere_radius) {
            printf("Drone %d detected Drone %d as a neighbor.\n", drone->id, drones[i]->id);
        }
    }
}

void communicate_with_neighbors(Drone *drone, Drone *drones[], int num_drones) {
    for (int i = 0; i < num_drones; i++) {
        if (drone != drones[i] && distance_between_drones(drone, drones[i]) <= drone->sphere_radius) {
            printf("Drone %d communicated with Drone %d\n", drone->id, drones[i]->id);
        } else {
            printf("Drone %d is too far from Drone %d. Distance: %f\n", drone->id, drones[i]->id, distance_between_drones(drone, drones[i]));
        }
    }
}
