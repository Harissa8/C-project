#include "drone_simulation.h"
#include <time.h>

int main() {
    srand(time(NULL));  // Seed random number generator once

    Drone *drones[3];
    drones[0] = create_drone((Vector3D){0, 0, 100}, (Vector3D){1, 0, 0}, 100, 1000, 0.1);
    drones[1] = create_drone((Vector3D){100, 0, 100}, (Vector3D){-1, 0, 0}, 100, 1000, 0.1);
    drones[2] = create_drone((Vector3D){50, 50, 100}, (Vector3D){0, 0, 0}, 100, 1000, 0.1);

    for (int i = 0; i < 100; i++) {
        printf("Iteration %d\n", i);
        for (int j = 0; j < 3; j++) {
            update_drone_position(drones[j], 0.1);
            update_neighbors(drones[j], drones, 3);
            communicate_with_neighbors(drones[j], drones, 3);
        }
        fflush(stdout);
    }

    printf("Simulation finished.\n");

    for (int i = 0; i < 3; i++) {
        free(drones[i]);
    }

    return 0;
}
