#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include "signatures.h"
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"




float calculate_distance(Drone* d1, Drone* d2) {
    float dx = d1->x - d2->x;
    float dy = d1->y - d2->y;
    float dz = d1->z - d2->z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}


Drone drones[MAX_DRONES];
int drone_count = 0;
//calcule le Coverage d'un drone
float calculate_coverage(float altitude) {
   return pow(2 * altitude * tan(0.05), 2);
}
// voir si 2 drone son dans le rang de communication
int within_communication_range(Drone* d1, Drone* d2) {
    float dx = d1->x - d2->x;
    float dy = d1->y - d2->y;
    float dz = d1->z - d2->z;
    float distance = sqrt(dx * dx + dy * dy + dz * dz);
    return distance <= d1->communication_range;
}
//faire update des neighbors
void update_neighbors() {
    for (int i = 0; i < drone_count; i++) {
        drones[i].neighbor_count = 0;

        for (int j = 0; j < drone_count; j++) {
            if ((i != j)&& (within_communication_range(&drones[i], &drones[j]))) {
                drones[i].neighbors[drones[i].neighbor_count] = drones[j].id;
                drones[i].neighbor_count++;
            }
        }
    }
}

void spread_drones(int num_drones, float xmin, float ymin, float xmax, float ymax, float comm_range, float cam_res,Image Im1) {
   

    float total_area = (xmax - xmin) * (ymax - ymin);
    
    float c=sqrt(total_area/num_drones);
    float h=c/(2*tan(0.05));
    
    float grid_size = c;
    int drones_per_row = (int)ceil((xmax - xmin) / grid_size);
    int drones_per_column = (int)ceil((ymax - ymin) / grid_size);
    
    // Initialize the drones
    int drone_id = 0;
    for (int i = 0; i < drones_per_row && drone_id < num_drones; i++) {
        for (int j = 0; j < drones_per_column && drone_id < num_drones; j++) {
            // Calculate the position of each drone
            float x = xmin + i * grid_size + grid_size/2 ;
            float y = ymin + j * grid_size + grid_size/2;
            float z = h; // Adjust altitude dynamically if needed
            
            // Initialize the drone
            drones[drone_id].id = drone_id + 1;
            drones[drone_id].x = x;
            drones[drone_id].y = y;
            drones[drone_id].z = z;
            drones[drone_id].coverage = c*c;
            drones[drone_id].vx = 0;
            drones[drone_id].vy = 0;
            drones[drone_id].vz = 0;
            drones[drone_id].communication_range = comm_range;
            drones[drone_id].camera_resolution = cam_res;
            drones[drone_id].is_active=1;
        

            drone_id++;
        }
    }

    drone_count = drone_id;
    update_neighbors();
}







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
                    drones[i].z = sqrt(new_coverage) / (2 * tan(0.05));

                    // Update coverage area for the neighbor
                    drones[i].coverage = calculate_coverage(drones[i].z);
                    
                    printf("Drone %d adapted its position to (%.2f, %.2f, %.2f) and coverage area to %.2f m^2 to cover for destroyed Drone %d.\n", 
                        drones[i].id, drones[i].x, drones[i].y, drones[i].z, drones[i].coverage, destroyed_drone.id);
                }
            }
        }
    }
}

void check_and_adapt() {
    for (int i = 0; i < drone_count; i++) {
        if (!drones[i].is_active) {
            printf("Drone %d is destroyed, adapting neighbors...\n", drones[i].id);
            adjust_neighbors_for_destroyed_drone(i);
            update_neighbors();
        }
     }
}

//aficher les donnes des drones simplement
void print_drone_infos() {
    for (int i = 0; i < drone_count; i++) {
        printf("Drone %d:\n", drones[i].id);
        printf("  Position (x=%.3f, y=%.3f, z=%.3f)\n", drones[i].x, drones[i].y, drones[i].z);
        printf("  Coverage Area = %.3f m^2\n", drones[i].coverage);
        printf("  Velocity (vx=%.3f, vy=%.3f, vz=%.3f)\n", drones[i].vx, drones[i].vy, drones[i].vz);
        printf("  Communication Range = %.3f meters\n", drones[i].communication_range);
        printf("  Camera Resolution = %.3f pixels\n", drones[i].camera_resolution);
        printf("  Active Status = %s\n", drones[i].is_active ? "Active" : "Destroyed");
        printf("\n");
    }
}


//avoir si la position est occuper par un autre drone
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

//  calculate the centroid of all active drones' positions
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
float random_float(float min, float max) {
    return min + ((float)rand() / RAND_MAX) * (max - min);
}

// Function to calculate new positions using Voronoi-like partitioning (adjust drones to optimize coverage)
void voronoi_adapt_all_drones(int moved_drone_id,Image*img1) {
    printf("Voronoi-based intelligent adaptation after drone %d moved...\n", moved_drone_id);

   // Drone* moved_drone = &drones[moved_drone_id]; // Get the moved drone

    // Calculate the centroid of the current drone positions
    float centroid_x, centroid_y, centroid_z;
    calculate_centroid(&centroid_x, &centroid_y, &centroid_z);

    // Now, for each drone, we will calculate an optimal position that avoids overlap and maximizes coverage
    for (int i = 0; i < drone_count; i++) {
        if ((i != moved_drone_id-1) && (drones[i].is_active)) {
            Drone* d = &drones[i];

            // Calculate the vector from the centroid to the current drone position
            float vx = d->x - centroid_x;
            float vy = d->y - centroid_y;
            float vz = d->z - centroid_z;

            // Normalize the vector
            float magnitude = sqrt(vx * vx + vy * vy + vz * vz);
            vx /= magnitude;
            vy /= magnitude;
            vz /= magnitude;

            // Move the drone along this vector to spread it out (adjust the distance based on Voronoi partitioning)
            float offset_distance = random_float(1.0, 3.0); // Adjust the offset distance
            d->x += vx * offset_distance;
            d->y += vy * offset_distance;
            d->z += vz * offset_distance;

            // Make sure drones stay within bounds after movement
            if ((d->x) < (img1-> xMin)) d->x = img1-> xMin;
            if ((d->x) > (img1-> xMax)) d->x = img1-> xMax;
            if ((d->y) < (img1-> yMin)) d->y = img1-> yMin;
            if ((d->y) > (img1-> yMax)) d->y = img1-> yMax;
            if (d->z < 0) d->z = 0;

            printf("Drone %d adjusted to new position (%.2f, %.2f, %.2f) using Voronoi partitioning.\n",
                   d->id, d->x, d->y, d->z);
        }
    }
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

float go_to(float x, float y, float z, int id,Image*img1) {
     // Validate drone ID
    if ((id < 0) || (id >= drone_count) || (!drones[id].is_active)) {
        printf("Error: Invalid drone ID or drone is not active.\n");
        return -1; // Return -1 to indicate an error
    }

    // Check if the destination is within the valid boundaries
    if ((x <img1-> xMin)|| (x >img1-> xMax) || (y < img1->yMin)|| (y >img1-> yMax) || z < 0) {
        printf("Error: Destination is out of bounds.\n");
        return -1; // Return -1 to indicate an error
    }

    Drone *d = &drones[id-1];  // Get the drone by its ID
   // float old_x = d->x, old_y = d->y, old_z = d->z;  // Save the old position
      // Check if the destination is occupied by another drone
    int occupied_by_id = is_position_occupied(x, y, z, id);
    if (occupied_by_id != -1) {
        printf("Destination is occupied by drone %d. Adjusting all drones intelligently...\n", occupied_by_id);
        // Call the Voronoi partitioning function to adjust all drones' positions
        voronoi_adapt_all_drones(id,img1);
    }
    
    // Calculate the Euclidean distance between the current position and the destination
    float dx = x - d->x;
    float dy = y - d->y;
    float dz = z - d->z;
    float distance = sqrt(dx * dx + dy * dy + dz * dz);
    
    // Assuming constant velocity (Vmax), calculate the time taken to reach the destination
    float time_taken = distance / Vmax;

    // Update the drone's position to the new coordinates
    d->x = x;
    d->y = y;
    d->z = z;

    // Perform Voronoi partitioning again for system-wide adaptation
    voronoi_adapt_all_drones(id,img1);

    // Call the collision avoidance function to ensure no drones are too close after the move
    avoid_collisions();

    return time_taken; // Return the time taken to reach the destination
}

float* speed(float vx, float vy, float vz, int id, float delta_t,Image*img1) {
    // Validate drone ID
    if ((id < 0) || (id >= drone_count) || (!drones[id].is_active)) {
        printf("Error: Invalid drone ID or drone is not active.\n");
        return NULL; // Return NULL to indicate an error
    }

    Drone *d = &drones[id-1];  // Get the drone by its ID

    // Check if the velocity exceeds the maximum allowed speed (Vmax)
    float speed_magnitude = sqrt(vx * vx + vy * vy + vz * vz);
    if (speed_magnitude > Vmax) {
        // Scale down the velocity to respect the Vmax constraint
        float scale_factor = Vmax / speed_magnitude;
        vx *= scale_factor;
        vy *= scale_factor;
        vz *= scale_factor;
    }

    // Calculate new positions based on velocity and time delta (delta_t)
    float xnew = vx * delta_t + d->x;
    float ynew = vy * delta_t + d->y;
    float znew = vz * delta_t + d->z;

    // Check if the new position is within the valid boundaries
    if ((xnew < img1-> xMin) || (xnew > img1-> xMax) || 
        (ynew < img1-> yMin) || (ynew > img1-> yMax)||  
        znew <0) {
        printf("Error: New position out of bounds.\n");
        return NULL; // Return NULL to indicate an error
    }

    // Check if the destination is already occupied by another drone
    int occupied_by_id = is_position_occupied(xnew, ynew, znew, id);
    if (occupied_by_id != -1) {
        printf("Position is occupied by drone %d. Adjusting all drones intelligently...\n", occupied_by_id);
        // Call the intelligent adaptation function to adjust all drones' positions
        voronoi_adapt_all_drones(id,img1);
    }

    // Update the drone's position to the new coordinates
    d->x = xnew;
    d->y = ynew;
    d->z = znew;

    // Allocate memory to store the new coordinates
    float* newco = (float*)malloc(3 * sizeof(float));
    if (newco == NULL) {
        printf("Error: Memory allocation failed.\n");
        return NULL; // Return NULL if memory allocation fails
    }

    // Set the new coordinates
    newco[0] = xnew;
    newco[1] = ynew;
    newco[2] = znew;


    voronoi_adapt_all_drones(id,img1);  // Adjust all drones' positions based on the moved drone
    avoid_collisions();            
    return newco; // Return the new coordinates
}
void zoom_out(float z,int id,Image*img1){
    Drone *d = &drones[id];
    go_to(d->x,d->y,z,id,img1);


}
void stretch(float x,float y,int id,Image*img1){
    Drone *d = &drones[id];
    go_to(x,y,d->z,id,img1);

}
unsigned char* readPixelMatrix(const char* filename, int* width, int* height) {
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error: Could not open file %s for reading\n", filename);
        return NULL;
    }

    // First, read the dimensions of the image
    if (fscanf(file, "%d %d", width, height) != 2) {
        fprintf(stderr, "Error: Could not read image dimensions\n");
        fclose(file);
        return NULL;
    }

    // Allocate memory for the pixel data (width * height * 3 for RGB)
    unsigned char* pixelData = (unsigned char*)malloc((*width) * (*height) * 3);
    if (pixelData == NULL) {
        fprintf(stderr, "Error: Could not allocate memory for pixel data\n");
        fclose(file);
        return NULL;
    }

    // Read the pixel data from the file
    for (int y = 0; y < *height; y++) {
        for (int x = 0; x < *width; x++) {
            int r, g, b;
            if (fscanf(file, "%d %d %d", &r, &g, &b) != 3) {
                fprintf(stderr, "Error: Could not read pixel data at (%d, %d)\n", x, y);
                free(pixelData);
                fclose(file);
                return NULL;
            }
            // Store the pixel data in the array
            pixelData[(y * (*width) + x) * 3 + 0] = (unsigned char)r;
            pixelData[(y * (*width) + x) * 3 + 1] = (unsigned char)g;
            pixelData[(y * (*width) + x) * 3 + 2] = (unsigned char)b;
        }
    }

    fclose(file);
    return pixelData;
}
unsigned char* removeSubregion(unsigned char* pixelData, int origWidth, int origHeight,
                               int startLine, int startCol, int numLines, int numCols) {
    // Allocate memory for the new image with original dimensions
    unsigned char* newImage = (unsigned char*)malloc(origWidth * origHeight * 3);
    if (newImage == NULL) {
        fprintf(stderr, "Error: Could not allocate memory for new image\n");
        return NULL;
    }

    // Copy the original image into the new image
    memcpy(newImage, pixelData, origWidth * origHeight * 3);

    // Replace the subregion with black pixels (RGB: 0, 0, 0)
    for (int y = 0; y < numLines; y++) {
        for (int x = 0; x < numCols; x++) {
            // Check bounds to avoid accessing out-of-bounds memory
            if ((startLine + y) < origHeight && (startCol + x) < origWidth) {
                // Set the pixels in the subregion to black
                int index = ((startLine + y) * origWidth + (startCol + x)) * 3;
                newImage[index + 0] = 0;  // Red
                newImage[index + 1] = 0;  // Green
                newImage[index + 2] = 0;  // Blue
            }
        }
    }

    // Return the pointer to the modified image
    return newImage;
}

unsigned char* createImageWithSubregion(unsigned char* pixelData, int origWidth, int origHeight,
                                         int startLine, int startCol, int numLines, int numCols) {
    unsigned char* newImage = (unsigned char*)malloc(origWidth * origHeight * 3);
    if (newImage == NULL) {
        fprintf(stderr, "Error: Could not allocate memory for new image\n");
        return NULL;
    }

    for (int i = 0; i < origWidth * origHeight * 3; i++) {
        newImage[i] = 0; 
    }

    for (int y = 0; y < numLines; y++) {
        for (int x = 0; x < numCols; x++) {
            if ((startLine + y) < origHeight && (startCol + x) < origWidth) {
                newImage[((startLine + y) * origWidth + (startCol + x)) * 3 + 0] = pixelData[((startLine + y) * origWidth + (startCol + x)) * 3 + 0];
                newImage[((startLine + y) * origWidth + (startCol + x)) * 3 + 1] = pixelData[((startLine + y) * origWidth + (startCol + x)) * 3 + 1];
                newImage[((startLine + y) * origWidth + (startCol + x)) * 3 + 2] = pixelData[((startLine + y) * origWidth + (startCol + x)) * 3 + 2];
            }
        }
    }

    return newImage;
}

void saveImageAsPNG(const char* filename, unsigned char* pixelData, int width, int height) {
    if (stbi_write_png(filename, width, height, 3, pixelData, width * 3)) {
        printf("Image saved to: %s\n", filename);
    } else {
        fprintf(stderr, "Error: Could not save image to %s\n", filename);
    }
}
int widthh=1357;
int heightt = 648;
//1357 648

void copySubregion(unsigned char* srcData, unsigned char* targetData, int imgWidth, int imgHeight, 
                   int startLine, int startCol, int subWidth, int subHeight) {
    for (int y = 0; y < subHeight; y++) {
        for (int x = 0; x < subWidth; x++) {
            // Make sure we're within the bounds of the image
            if ((startLine + y) < imgHeight && (startCol + x) < imgWidth) {
                // Copy pixel data (RGB) from source to target
                targetData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 0] = srcData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 0];
                targetData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 1] = srcData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 1];
                targetData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 2] = srcData[((startLine + y) * imgWidth + (startCol + x)) * 3 + 2];
            }
        }
    }
}
int assignStandardValue(int value) {
    // Define the three standard values
    int standard1 = 700;
    int standard2 = 2000;
    int standard3 = 3300;

    // Calculate the difference between the given value and each standard value
    int diff1 = abs(value - standard1);
    int diff2 = abs(value - standard2);
    int diff3 = abs(value - standard3);

    // Assign the value closest to one of the standard values
    if (diff1 <= diff2 && diff1 <= diff3) {
        return standard1;
    } else if (diff2 <= diff1 && diff2 <= diff3) {
        return standard2;
    } else {
        return standard3;
    }
}

void capture(const char *path1,const char *path2,const char *path3)
{
    
    unsigned char* newImage;
   unsigned char* pixelData3 = readPixelMatrix(path3,&widthh,&heightt);
   unsigned char* pixelData2 = readPixelMatrix(path2,&widthh,&heightt);
   unsigned char* pixelData1 = readPixelMatrix(path1,&widthh,&heightt);
    for(int i=0;i<drone_count;i++){
        float c = sqrt(calculate_coverage(drones[i].z));
            int ynew = (int)(drones[i].x-c/2);
            int xnew = (int)(drones[i].y-c/2);
            int b = assignStandardValue(drones[i].z);
          printf("c= %f, ynew= %d,  xnew= %d, b= %d\n",c,ynew,xnew,b);
        if(drones[i].is_active){
            

            if (b==700){
                printf("700 \n");

            newImage = createImageWithSubregion(pixelData1, widthh, heightt, xnew, ynew, (int)c, (int)c);
            }
             if (b==2000){
                printf("2000 \n");
             newImage = createImageWithSubregion(pixelData2, widthh, heightt, xnew, ynew, (int)c, (int)c);
            }
             if (b==3300){
                printf("3300 \n");
            newImage = createImageWithSubregion(pixelData3, widthh, heightt, xnew, ynew, (int)c, (int)c);
            }
            if (newImage == NULL) {
            fprintf(stderr, "Error: Failed to create subregion for drone %d\n", i);
            continue;  
            }
        }
        else{
            if (b==700){
                printf("700 \n");

            newImage = removeSubregion(pixelData1, widthh, heightt, xnew, ynew, (int)c, (int)c);
            }
             if (b==2000){
                printf("2000 \n");
             newImage = removeSubregion(pixelData2, widthh, heightt, xnew, ynew, (int)c, (int)c);
            }
             if (b==3300){
                printf("3300 \n");
            newImage = removeSubregion(pixelData3, widthh, heightt, xnew, ynew, (int)c, (int)c);
            }
            if (newImage == NULL) {
            fprintf(stderr, "Error: Failed to create subregion for drone %d\n", i);
            continue;  // Skip to the next iteration if subregion creation failed
            }
        }


        saveImageAsPNG("C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\output\\subregion_image.png", newImage, widthh, heightt);

        
        char *outputsuregionname = "C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\output\\subregion_image.png";
        saveImageAsPNG(outputsuregionname, newImage, widthh, heightt);
        const char* originalImageFilename = "C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\output\\subregion_image.png";
        const char* targetImageFilename = "C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\output\\black_image.png";
        const char* outputImageFilename = "C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\output\\black_image.png";
       int origWidth, origHeight, origChannels;
       unsigned char* originalData = stbi_load(originalImageFilename, &origWidth, &origHeight, &origChannels, 3);
        int targetWidth, targetHeight, targetChannels;
        unsigned char* targetData = stbi_load(targetImageFilename, &targetWidth, &targetHeight, &targetChannels, 3);
        
     copySubregion(originalData, targetData, origWidth, origHeight, xnew, ynew, (int)c, (int)c);

     if (stbi_write_png(outputImageFilename, targetWidth, targetHeight, 3, targetData, targetWidth * 3)) {
        printf("Modified image saved to: %s\n", outputImageFilename);
     } else {
        fprintf(stderr, "Error: Could not save output image\n");
     }
    free(newImage);
printf("drone: %d",i);
    }
    

    
}



int main() {
    int num_drones;
    Image Im1;
    //1357 648
    //pour l'image on choisit l'image de la captile de Liban:Beirut!!
    //l'image est size 1357 x 648
    float xMin=0, yMin=0, xMax=1357, yMax=648;
    Image *img1;
    img1 = &Im1;
    img1->xMax=xMax;
    img1->xMin=xMin;
    img1->yMax=yMax;
    img1->yMin=yMin;
    img1->height=648;//pour notre image
    img1->width=1357;//pour notre image
    
    //NB: pour faire run il faut changer les InputFileName, car ils correspond pour les paths sur nos PC!
    const char* inputFilename9 = "C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\pixel_matrix1.txt";
    const char* inputFilename8 = "C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\pixel_matrix2.txt";
    const char* inputFilename7 = "C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\pixel_matrix3.txt";
    const char *filename="C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\Command.txt";
    FILE *file = fopen(filename, "r");//lire les command
    
    
    
    //On peut faire lire les commande par l'utlisateur directement,sans avoir besoin de l'utlisation d'un fichier
    /*    while (1) {
        printf("Enter command (or type 'exit' to quit): ");
        fgets(command, sizeof(command), stdin);  // Read command from user

        // Check for exit condition
        if (strcmp(command, "exit\n") == 0) {
            break;
        }

        char cmd_type[10];
        sscanf(command, "%s", cmd_type);  // Extract command type

        if (strcmp(cmd_type, "goto") == 0) {
            float x, y, z;
            int drone_id;
            sscanf(command, "goto %f %f %f %d", &x, &y, &z, &drone_id);
            float t = go_to(x, y, z, drone_id, &Im1);
            print_drone_infos();
            printf("The drone took %f seconds to arrive at its destination\n", t);
        } 
        else if (strcmp(cmd_type, "speed") == 0) {
            float vx, vy, vz, delta_t;
            int drone_id;
            sscanf(command, "speed %f %f %f %d %f", &vx, &vy, &vz, &drone_id, &delta_t);
            float* ff = speed(vx, vy, vz, drone_id, delta_t, &Im1);
            printf("La nouvelle position est: x=%d / y=%d / z=%d\n", (int)ff[0], (int)ff[1], (int)ff[2]);
        } 
        else if (strcmp(cmd_type, "destruct") == 0) {
            int drone_id;
            sscanf(command, "destruct %d", &drone_id);
            // Assuming drones is a global or accessible variable
            drones[drone_id - 1].is_active = 0;
            spread_drones(num_drones - 1, Im1.xMin, Im1.yMin, Im1.xMax, Im1.yMax, communication_range, camera_resolution, Im1);
            print_drone_infos();
        } else {
            printf("Unknown command: %s\n", cmd_type);
        }
    }

    return 0;
}*/
    float communication_range, camera_resolution;
    fscanf(file, "%d", &num_drones);
    printf("Number of drones: %d\n", num_drones);
    fscanf(file, "%f", &communication_range);
    printf("Communication range: %.2f meters\n", communication_range);
    fscanf(file, "%f", &camera_resolution);
    printf("Camera resolution: %.2f megapixels\n", camera_resolution);
    spread_drones(num_drones, xMin, yMin, xMax, yMax,communication_range,camera_resolution,Im1);
     print_drone_infos();
     capture(inputFilename9,inputFilename8, inputFilename7);
    char command[50];
    while (fgets(command, sizeof(command), file)) {  
        char cmd_type[10];
        sscanf(command, "%s", cmd_type); 
        if (strcmp(cmd_type, "goto") == 0) {
            float x, y, z;
            int drone_id;
            sscanf(command, "goto %f %f %f %d", &x, &y, &z, &drone_id);
           float gogoto= go_to(x, y, z,drone_id, img1);
           capture(inputFilename9,inputFilename8, inputFilename7);
           print_drone_infos();
           printf("The drone took %f seconds to arrive at its destination",gogoto);
        } 
        else if (strcmp(cmd_type, "speed") == 0) {//le command speed
            float vx, vy, vz, delta_t;
            int drone_id;
            sscanf(command, "speed %f %f %f %d %f", &vx, &vy, &vz, &drone_id, &delta_t);
            float *new;
            new=speed(vx, vy, vz, drone_id, delta_t,img1);
            int newx=(int)new[0];
            int newy=(int)new[1];
            int newz=(int)new[2];
            capture(inputFilename9,inputFilename8, inputFilename7);
            print_drone_infos();
            printf("la nouvelle position est: x=%d / y=%d / z=%d",newx,newy,newz);
        } 
        else if (strcmp(cmd_type, "destruct") == 0) {//si l'utilisateur a chosir le command destruct
            int drone_id;
            sscanf(command,"destruct %d",&drone_id);
            drones[drone_id-1].is_active=0;//faire deactiver ce drone
            capture(inputFilename9,inputFilename8, inputFilename7);//faire capturer 
            print_drone_infos();//afficher les donnes
           spread_drones(num_drones-1, xMin, yMin, xMax, yMax,communication_range,camera_resolution,Im1);//refaire le partionier des drones mais son le dernier qui est detruit
            capture(inputFilename9,inputFilename8, inputFilename7);
            print_drone_infos();
        }
    }

    fclose(file);

}
