#include <stdio.h>
#include <png.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define MAX_DRONES 100
#define alpha 0.05
#define POSITION_THRESHOLD 0.1

typedef struct {
    int id;
    float x, y; // position
    float coverage;
    png_bytep *image; // Holds the captured image from this drone
} Drone;

typedef struct {
    float xMin, xMax, yMin, yMax;
    int width, height;
} Image;

Drone drones[MAX_DRONES];
int drone_count = 0;

png_bytep *load_png_image(const char *filename, int *width, int *height);
void capture_image_segment(Drone *drone, png_bytep *map, int map_width, int map_height, Image area);
void merge_images(const char *output_filename, Drone drones[], int num_drones, int final_width, int final_height);
void save_png_image(const char *filename, png_bytep *row_pointers, int width, int height);

png_bytep *load_png_image(const char *filename, int *width, int *height) {
    FILE *fp = fopen(filename, "rb");
    if (!fp) {
        fprintf(stderr, "Could not open file %s for reading.\n", filename);
        return NULL;
    }

    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) {
        fclose(fp);
        return NULL;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_read_struct(&png, NULL, NULL);
        fclose(fp);
        return NULL;
    }

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_read_struct(&png, &info, NULL);
        fclose(fp);
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

void capture_image_segment(Drone *drone, png_bytep *map, int map_width, int map_height, Image area) {
    int capture_size = (int)sqrt(drone->coverage); // Calculating capture area based on coverage
    drone->image = malloc(sizeof(png_bytep) * capture_size);
    for (int y = 0; y < capture_size; y++) {
        drone->image[y] = (png_bytep)malloc(capture_size * 4); // RGBA format
    }

    // Scale drone position to map coordinates
    int start_x = (int)((drone->x - area.xMin) / (area.xMax - area.xMin) * map_width) - (capture_size / 2);
    int start_y = (int)((drone->y - area.yMin) / (area.yMax - area.yMin) * map_height) - (capture_size / 2);

    for (int y = 0; y < capture_size; y++) {
        for (int x = 0; x < capture_size; x++) {
            int map_x = start_x + x;
            int map_y = start_y + y;

            if (map_x >= 0 && map_x < map_width && map_y >= 0 && map_y < map_height) {
                memcpy(&drone->image[y][x * 4], &map[map_y][map_x * 4], 4); // Copy RGBA pixel
            } else {
                memset(&drone->image[y][x * 4], 0, 4); // Set out-of-bounds pixels to black
            }
        }
    }
}

void merge_images(const char *output_filename, Drone drones[], int num_drones, int final_width, int final_height) {
    png_bytep *final_image = (png_bytep *)malloc(sizeof(png_bytep) * final_height);
    for (int y = 0; y < final_height; y++) {
        final_image[y] = (png_byte *)calloc(final_width, 4); // RGBA format
    }

    for (int i = 0; i < num_drones; i++) {
        Drone *drone = &drones[i];
        int capture_size = (int)sqrt(drone->coverage);
        int start_x = (int)((drone->x - capture_size / 2));
        int start_y = (int)((drone->y - capture_size / 2));

        for (int y = 0; y < capture_size; y++) {
            for (int x = 0; x < capture_size; x++) {
                int final_x = start_x + x;
                int final_y = start_y + y;

                if (final_x >= 0 && final_x < final_width && final_y >= 0 && final_y < final_height) {
                    memcpy(&final_image[final_y][final_x * 4], &drone->image[y][x * 4], 4);
                }
            }
        }
    }

    save_png_image(output_filename, final_image, final_width, final_height);

    for (int y = 0; y < final_height; y++) {
        free(final_image[y]);
    }
    free(final_image);
}

void save_png_image(const char *filename, png_bytep *row_pointers, int width, int height) {
    FILE *fp = fopen(filename, "wb");
    if (!fp) {
        perror("Failed to open file for writing");
        return;
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) {
        fclose(fp);
        return;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_write_struct(&png, NULL);
        fclose(fp);
        return;
    }

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        return;
    }

    png_init_io(png, fp);
    png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGBA, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);
    png_write_image(png, row_pointers);
    png_write_end(png, NULL);

    png_destroy_write_struct(&png, &info);
    fclose(fp);
}

void free_drone_images(int num_drones) {
    for (int i = 0; i < num_drones; i++) {
        int capture_size = (int)sqrt(drones[i].coverage);
        for (int y = 0; y < capture_size; y++) {
            free(drones[i].image[y]);
        }
        free(drones[i].image);
    }
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

    printf("Enter the area dimensions (xMin yMin xMax yMax): ");
    scanf("%f %f %f %f", &xMin, &yMin, &xMax, &yMax);
    printf("Enter the communication range: ");
    scanf("%f", &comm_range);
    printf("Enter the camera resolution: ");
    scanf("%f", &cam_res);

    Im1.xMin = xMin;
    Im1.yMin = yMin;
    Im1.xMax = xMax;
    Im1.yMax = yMax;
    Im1.width = xMax - xMin;
    Im1.height = yMax - yMin;

    int map_width, map_height;
    png_bytep *map_image = load_png_image(map_filename, &map_width, &map_height);
    if (!map_image) {
        printf("Error: Could not load the map image.\n");
        return 1;
    }

    for (int i = 0; i < num_drones; i++) {
        drones[i].id = i;
        drones[i].x = xMin + (i % (int)sqrt(num_drones)) * comm_range;  // Example positioning
        drones[i].y = yMin + (i / (int)sqrt(num_drones)) * comm_range;
        drones[i].coverage = cam_res;
        capture_image_segment(&drones[i], map_image, map_width, map_height, Im1);
    }

    merge_images("output.png", drones, num_drones, map_width, map_height);

    for (int y = 0; y < map_height; y++) {
        free(map_image[y]);
    }
    free(map_image);
    free_drone_images(num_drones);

    return 0;
}
