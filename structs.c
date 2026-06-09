#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "structs.h"

void make_road(struct Road* road, int x1, int y1, int x2, int y2){
    // Create a rectangular road from corner (x1, y1) to corner (x2, y2).
    road->x1 = x1; road->y1 = y1;
    road->x2 = x2; road->y2 = y2;
    // Calculate the number of pixels needed for the road.
    int length = abs(x2 - x1) + 1, width = abs(y2 - y1) + 1;
    road->length = length;
    road->width = width;
    road->pixels = (struct Pixel**)malloc(length * width * sizeof(struct Pixel*));
    // Draw a rectangle to represent the road.
    for(int i = 0; i < length; i++){
        for(int j = 0; j < width; j++){
            road->pixels[i * width + j] = (struct Pixel*)malloc(sizeof(struct Pixel));
            road->pixels[i * width + j]->x = x1 + i;
            road->pixels[i * width + j]->y = y1 + j;
            // Set the color of the road's pixels to gray.
            road->pixels[i * width + j]->r = 128;
            road->pixels[i * width + j]->g = 128;
            road->pixels[i * width + j]->b = 128;
            // Initially, no pixel is lighted.
            road->pixels[i * width + j]->light = 0;
        }
    }
}

void make_vehicle(struct Vehicle* vehicle, int x, int y, int r, int g, int b, int dx, int dy, int length, int width){
    // Create a vehicle at (x, y) with direction (dx, dy) and a specified number of pixels.
    vehicle->x = x; vehicle->y = y;
    vehicle->r = r; vehicle->g = g; vehicle->b = b;
    vehicle->dx = dx; vehicle->dy = dy;
    vehicle->length = length; vehicle->width = width;
    vehicle->num_pixels = length * width;
    vehicle->pixels = (struct Pixel**)malloc(vehicle->num_pixels * sizeof(struct Pixel*));

    // Fill the pixels of the vehicle, starting from point (x, y) and expanding by (length) and (width).
    for(int i = 0; i < vehicle->length; i++){
        for(int j = 0; j < vehicle->width; j++){
            vehicle->pixels[i * vehicle->width + j] = (struct Pixel*)malloc(sizeof(struct Pixel));
            vehicle->pixels[i * vehicle->width + j]->x = x + i;
            vehicle->pixels[i * vehicle->width + j]->y = y + j;
            // Set the color of the vehicle's pixels.
            vehicle->pixels[i * vehicle->width + j]->r = vehicle->r;
            vehicle->pixels[i * vehicle->width + j]->g = vehicle->g;
            vehicle->pixels[i * vehicle->width + j]->b = vehicle->b;
            // Initially, all pixels of the vehicle are lighted.
            vehicle->pixels[i * vehicle->width + j]->light = 1;
        }
    }
}

void make_map(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles){
    printf("Map:\n");
    for(int i = 0; i < num_roads; i++){
        printf("Road %d: (%d, %d) to (%d, %d)\n", i, roads[i].x1, roads[i].y1, roads[i].x2, roads[i].y2);
    }
    for(int i = 0; i < num_vehicles; i++){
        printf("Vehicle %d: Position (%d, %d), Direction (%d, %d), Number of Pixels: %d\n",
            i, vehicles[i].x, vehicles[i].y, vehicles[i].dx, vehicles[i].dy, vehicles[i].length * vehicles[i].width);
    }
}

void move_vehicle(struct Vehicle* vehicle){
    // Move the vehicle one step in its current direction.
    for(int i = 0; i < vehicle->num_pixels; i++){
        vehicle->pixels[i]->x += vehicle->dx;
        vehicle->pixels[i]->y += vehicle->dy;
    }
    vehicle->x += vehicle->dx;
    vehicle->y += vehicle->dy;
}

void check_vehicle_collision(struct Vehicle* vehicle1, struct Vehicle* vehicle2){
    // Check if two vehicles collide by comparing their pixels' positions.
    for(int i = 0; i < vehicle1->num_pixels; i++){
        for(int j = 0; j < vehicle2->num_pixels; j++){
            if(vehicle1->pixels[i]->x == vehicle2->pixels[j]->x &&
               vehicle1->pixels[i]->y == vehicle2->pixels[j]->y){
                printf("Collision detected between Vehicle 1 and Vehicle 2 at (%d, %d)\n",
                    vehicle1->pixels[i]->x, vehicle1->pixels[i]->y);
                return;
            }
        }
    }
    printf("No collision detected between Vehicle 1 and Vehicle 2.\n");
}

void check_dead_end(struct Vehicle* vehicle, struct Road* roads, int num_roads){
    // Check if the vehicle has reached a dead end by checking if its next position is on a road.
    int next_x = vehicle->x + vehicle->dx;
    int next_y = vehicle->y + vehicle->dy;
    for(int i = 0; i < num_roads; i++){
        for(int j = 0; j < roads[i].length * roads[i].width; j++){
            if(roads[i].pixels[j]->x == next_x && roads[i].pixels[j]->y == next_y){
                printf("Vehicle is not at a dead end. Next position (%d, %d) is on a road.\n", next_x, next_y);
                return;
            }
        }
    }
    printf("Vehicle has reached a dead end at (%d, %d).\n", next_x, next_y);
}

void draw_scene(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles){
    // Draw the scene with ANSI escape codes.
    // Dots represent roads. Asterisks represent vehicles.
    char scene[20][20];
    int r[20][20], g[20][20], b[20][20];

    for(int y = 0; y < 20; y++){
        for(int x = 0; x < 20; x++){
            scene[y][x] = ' ';
            r[y][x] = 255;
            g[y][x] = 255;
            b[y][x] = 255;
        }
    }

    // Place the roads on the scene.
    for (int i = 0; i < num_roads; i++){
        for(int j = 0; j < roads[i].length * roads[i].width; j++){
            int x = roads[i].pixels[j]->x;
            int y = roads[i].pixels[j]->y;
            if(x >= 0 && x < 20 && y >= 0 && y < 20){
                scene[y][x] = '.';
                r[y][x] = roads[i].pixels[j]->r;
                g[y][x] = roads[i].pixels[j]->g;
                b[y][x] = roads[i].pixels[j]->b;
            }
        }
    }

    // Place the vehicles on the scene.
    for(int i = 0; i < num_vehicles; i++){
        for(int j = 0; j < vehicles[i].length * vehicles[i].width; j++){
            int x = vehicles[i].pixels[j]->x;
            int y = vehicles[i].pixels[j]->y;
            if(x >= 0 && x < 20 && y >= 0 && y < 20){
                scene[y][x] = '*';
                r[y][x] = vehicles[i].pixels[j]->r;
                g[y][x] = vehicles[i].pixels[j]->g;
                b[y][x] = vehicles[i].pixels[j]->b;
            }
        }
    }

    // Print the scene with colour applied to each visible character.
    for(int y = 0; y < 20; y++){
        for(int x = 0; x < 20; x++){
            if(scene[y][x] != ' '){
                printf("\033[38;2;%d;%d;%dm%c\033[0m", r[y][x], g[y][x], b[y][x], scene[y][x]);
            } else {
                printf(" ");
            }
        }
        printf("\n");
    }
}

void run_animation(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles){
    for(int step = 0; step < 12; step++){
        printf("\033[2J\033[H");
        for (int i = 0; i < num_vehicles; i++) {
            move_vehicle(&vehicles[i]);
        }
        draw_scene(roads, num_roads, vehicles, num_vehicles);
        printf("Step %d\n", step + 1);
        usleep(300000);
    }
}

void free_memory(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles){
    for(int i = 0; i < num_roads; i++){
        for(int j = 0; j < roads[i].length * roads[i].width; j++){
            free(roads[i].pixels[j]);
        }
        free(roads[i].pixels);
    }
    for(int i = 0; i < num_vehicles; i++){
        for(int j = 0; j < vehicles[i].num_pixels; j++){
            free(vehicles[i].pixels[j]);
        }
        free(vehicles[i].pixels);
    }
}

void handle_terminal_relaunch(int argc, char *argv[]) {
    if (argc == 1) {
        char command[512];
        snprintf(command, sizeof(command), "x-terminal-emulator -e %s --child &", argv[0]);
        if (system(command) != -1) {
            exit(0); // Exit the original process once the child is launched
        }
    }
}