#ifndef DRONE_SIMULATION_H
#define DRONE_SIMULATION_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

// Define a 3D vector for positions and velocities
typedef struct {
    double x, y, z;
} Vector3D;

// Define a Drone structure
typedef struct {
    int id;
    Vector3D position;
    Vector3D initial_position; // Store the starting point
    Vector3D velocity;
    double sphere_radius;
    double battery_level;
    bool is_dead;
    double battery_drain_rate;
} Drone;

// Function declarations
Drone *create_drone(Vector3D position, Vector3D velocity, double sphere_radius, double battery_level, double battery_drain_rate);
void update_drone_position(Drone *drone, double dt);
void update_neighbors(Drone *drone, Drone *drones[], int num_drones);
void communicate_with_neighbors(Drone *drone, Drone *drones[], int num_drones);

#endif
