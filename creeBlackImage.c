#include <stdio.h>
#include <stdlib.h>

// Include the STB image write library
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

int main() {
    // Define the dimensions of the image
    int width = 1357;
    int height = 648;
    int channels = 3; // RGB

    // Allocate memory for the image data
    unsigned char* imageData = (unsigned char*)malloc(width * height * channels);
    if (imageData == NULL) {
        fprintf(stderr, "Error: Could not allocate memory for image data\n");
        return 1;
    }

    // Set all pixel values to 0 (black)
    for (int i = 0; i < width * height * channels; i++) {
        imageData[i] = 0; // Set each channel (R, G, B) to 0
    }

    // Save the image as a PNG file
    const char* filename = "black_image.png";
    stbi_write_png(filename, width, height, channels, imageData, width * channels);
    printf("Black image created: %s\n", filename);

    // Free the allocated memory
    free(imageData);

    return 0;
}
