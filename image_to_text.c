#include <stdio.h>
#include <stdlib.h>

// Include the STB image library
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// Function to load an image
unsigned char* loadImage(const char* filename, int* width, int* height, int* channels) {
    unsigned char* data = stbi_load(filename, width, height, channels, 0);
    if (data == NULL) {
        fprintf(stderr, "Error: Could not load image %s\n", filename);
        return NULL;
    }
    return data;
}

// Function to save the pixel matrix to a text file in RGB format
void savePixelMatrix(const char* filename, unsigned char* data, int width, int height, int channels) {
    FILE* file = fopen(filename, "w");
    if (file == NULL) {
        fprintf(stderr, "Error: Could not open file %s for writing\n", filename);
        return;
    }

    // Write pixel values as RGB
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int index = (y * width + x) * channels; // Calculate index for the pixel
            // Write RGB values, considering only the first three channels (R, G, B)
            if (channels >= 3) { // Ensure at least 3 channels are present
                fprintf(file, "%d %d %d ", data[index], data[index + 1], data[index + 2]);
            }
        }
        fprintf(file, "\n"); // Newline after each row
    }

    fclose(file);
    printf("Pixel matrix saved to: %s\n", filename);
}

// Main function
int main() {
    // Update this path to the image you want to load
    const char* inputFilename = "C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\Niveau 3.png";
    const char* outputFilename = "C:\\Users\\Administrator\\Desktop\\Sayed Ahmad Ali ,Harissa Hussein\\pixel_matrix3.txt"; 
    int width, height, channels;

    // Load the image
    unsigned char* pixelData = loadImage(inputFilename, &width, &height, &channels);
    if (pixelData == NULL) {
        return 1; // Exit if the image could not be loaded
    }

    // Save the pixel matrix to a text file in RGB format
    savePixelMatrix(outputFilename, pixelData, width, height, channels);

    // Free the loaded image data
    stbi_image_free(pixelData);

    return 0;
}
