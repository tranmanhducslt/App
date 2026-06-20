#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

struct Pixel{
    // Similar to "Snake", each Pixel is a part of the scene.
    // It has a position (x, y) and a color (r, g, b).
    int x, y;
    int r, g, b;
    // A Pixel is a part of a Road.
    // Its direction is that of the Vehicle that occupies it.
    int dx, dy;
};

struct Road{
    // A rectangular Road is made up of many Pixels.
    // Roads are straight (for now) and can intersect with each other, perpendiclarly.
    int x1, y1, x2, y2; // The Road's two endpoints.
    struct Pixel** pixels; // The array of the road's Pixels.
    int length, width;
};

struct Vehicle{
    // A Vehicle is a collection of Pixels that move together.
    // It has a position (x, y) and a direction (dx, dy).
    int x, y;
    int r, g, b;
    int dx, dy; // The direction of the vehicle's movement.
    struct Pixel** pixels; // The array of the vehicle's Pixels.
    int length, width, num_pixels; // The number of pixels that make up the vehicle.
    // Saved original direction so we can stop and later resume movement.
    int orig_dx, orig_dy;
    // Stopped flag and waiting time (in steps) used by arbitration logic.
    int stopped;
    int wait_time;
};

// Global registry so helper routines can inspect all vehicles in the scene.
struct Vehicle* g_vehicles = NULL;
int g_num_vehicles = 0;

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
            // Initially, no pixel is occupied.
            road->pixels[i * width + j]->dx = 0;
            road->pixels[i * width + j]->dy = 0;
        }
    }
}

void make_vehicle(struct Vehicle* vehicle, int x, int y, int r, int g, int b, int dx, int dy, int length, int width){
    // Create a vehicle at (x, y) with direction (dx, dy) and a specified number of pixels.
    vehicle->x = x; vehicle->y = y;
    vehicle->r = r; vehicle->g = g; vehicle->b = b;
    vehicle->dx = dx; vehicle->dy = dy;
    vehicle->orig_dx = dx; vehicle->orig_dy = dy;
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
            vehicle->pixels[i * vehicle->width + j]->dx = dx;
            vehicle->pixels[i * vehicle->width + j]->dy = dy;
        }
    }
    vehicle->stopped = 0;
    vehicle->wait_time = 0;
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

/*
int check_dead_end(struct Vehicle* vehicle, struct Road* roads, int num_roads){
    // Check if the vehicle has reached a dead end by checking if its next position is on a road.
    int next_x = vehicle->x + vehicle->dx;
    int next_y = vehicle->y + vehicle->dy;
    for(int i = 0; i < num_roads; i++){
        for(int j = 0; j < roads[i].length * roads[i].width; j++){
            if(roads[i].pixels[j]->x == next_x && roads[i].pixels[j]->y == next_y){
                printf("Vehicle is not at a dead end. Next position (%d, %d) is on a road.\n", next_x, next_y);
                return 0;
            }
        }
    }
    printf("Vehicle has reached a dead end at (%d, %d).\n", next_x, next_y);
    return 1;
}
*/

int predict_pixel_collision(int x1, int y1, int dx1, int dy1,
                           int x2, int y2, int dx2, int dy2,
                           int max_steps){
    // Predict whether two distinct lit pixels will collide in the next `max_steps` steps.
    // Each pixel moves one step per frame in its recorded direction.
    for(int step = 1; step <= max_steps; step++){
        int next1_x = x1 + dx1 * step;
        int next1_y = y1 + dy1 * step;
        int next2_x = x2 + dx2 * step;
        int next2_y = y2 + dy2 * step;

        // Same position at the same future step.
        if(next1_x == next2_x && next1_y == next2_y){
            return 1;
        }

        // Swap positions in the same step (head-on pass-through collision).
        int prev1_x = x1 + dx1 * (step - 1);
        int prev1_y = y1 + dy1 * (step - 1);
        int prev2_x = x2 + dx2 * (step - 1);
        int prev2_y = y2 + dy2 * (step - 1);
        if(next1_x == prev2_x && next1_y == prev2_y &&
           next2_x == prev1_x && next2_y == prev1_y){
            return 1;
        }
    }
    return 0;
}

int predict_pixel_collision_from_pixels(const struct Pixel* a, const struct Pixel* b, int max_steps){
    return predict_pixel_collision(a->x, a->y, a->dx, a->dy,
                                   b->x, b->y, b->dx, b->dy,
                                   max_steps);
}

int predict_vehicles_collision(const struct Vehicle* v1, const struct Vehicle* v2, int max_steps){
    for(int i = 0; i < v1->num_pixels; i++){
        for(int j = 0; j < v2->num_pixels; j++){
            if(predict_pixel_collision_from_pixels(v1->pixels[i], v2->pixels[j], max_steps)){
                return 1;
            }
        }
    }
    return 0;
}

void stop_vehicle(struct Vehicle* vehicle){
    vehicle->stopped = 1;
    vehicle->dx = 0;
    vehicle->dy = 0;
    for(int i = 0; i < vehicle->num_pixels; i++){
        vehicle->pixels[i]->dx = 0;
        vehicle->pixels[i]->dy = 0;
    }
}

int can_resume_vehicle(int vidx, int max_steps){
    // Returns 1 if vehicle `vidx` can resume (no predicted collision within max_steps)
    if(g_vehicles == NULL || vidx < 0 || vidx >= g_num_vehicles) return 0;
    struct Vehicle* v = &g_vehicles[vidx];
    if(!v->stopped) return 1;
    if(v->orig_dx == 0 && v->orig_dy == 0) return 0;

    for(int pi = 0; pi < v->num_pixels; pi++){
        struct Pixel* p = v->pixels[pi];
        for(int oi = 0; oi < g_num_vehicles; oi++){
            if(oi == vidx) continue;
            struct Vehicle* ov = &g_vehicles[oi];
            for(int op = 0; op < ov->num_pixels; op++){
                struct Pixel* q = ov->pixels[op];
                if(predict_pixel_collision(p->x, p->y, v->orig_dx, v->orig_dy,
                                           q->x, q->y, q->dx, q->dy, max_steps)){
                    return 0;
                }
            }
        }
    }
    return 1;
}

void resume_vehicle(int vidx){
    if(g_vehicles == NULL || vidx < 0 || vidx >= g_num_vehicles) return;
    struct Vehicle* v = &g_vehicles[vidx];
    v->stopped = 0;
    v->dx = v->orig_dx;
    v->dy = v->orig_dy;
    v->wait_time = 0;
    for(int i = 0; i < v->num_pixels; i++){
        v->pixels[i]->dx = v->dx;
        v->pixels[i]->dy = v->dy;
    }
    printf("Resumed vehicle %d: direction=(%d,%d)\n", vidx, v->dx, v->dy);
}

void resize_terminal(int rows, int cols){
    struct winsize ws;
    ws.ws_row = rows;
    ws.ws_col = cols;
    ws.ws_xpixel = 0;
    ws.ws_ypixel = 0;
    if(ioctl(STDOUT_FILENO, TIOCSWINSZ, &ws) == 0){
        // Some terminals honor the ANSI window resize sequence as well.
        printf("\033[8;%d;%dt", rows, cols);
        fflush(stdout);
    }
}

void draw_scene(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles, int canvaSide){
    // Draw the scene with ANSI escape codes.
    // Dots represent roads. Asterisks represent vehicles.
    char scene[canvaSide][canvaSide];
    int r[canvaSide][canvaSide], g[canvaSide][canvaSide], b[canvaSide][canvaSide];

    for(int y = 0; y < canvaSide; y++){
        for(int x = 0; x < canvaSide; x++){
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
            if(x >= 0 && x < canvaSide && y >= 0 && y < canvaSide){
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
            if(x >= 0 && x < canvaSide && y >= 0 && y < canvaSide){
                scene[y][x] = '*';
                r[y][x] = vehicles[i].pixels[j]->r;
                g[y][x] = vehicles[i].pixels[j]->g;
                b[y][x] = vehicles[i].pixels[j]->b;
            }
        }
    }

    // Print the scene with colour applied to each visible character.
    for(int y = 0; y < canvaSide; y++){
        for(int x = 0; x < canvaSide; x++){
            if(scene[y][x] != ' '){
                printf("\033[38;2;%d;%d;%dm%c\033[0m", r[y][x], g[y][x], b[y][x], scene[y][x]);
            } else {
                printf(" ");
            }
        }
        printf("\n");
    }
}

void run_animation(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles, int canvaSide, int stepCount){
    for(int step = 0; step < stepCount; step++){
        // Predict imminent collisions across all vehicle pixels before moving.
        for(int i = 0; i < num_vehicles; i++){
            for(int j = i + 1; j < num_vehicles; j++){
                if(predict_vehicles_collision(&vehicles[i], &vehicles[j], 3)){
                    if(vehicles[i].dx != 0 || vehicles[i].dy != 0 || vehicles[j].dx != 0 || vehicles[j].dy != 0){
                        printf("Stopping vehicles %d and %d to avoid predicted collision.\n", i, j);
                    }
                    stop_vehicle(&vehicles[i]);
                    stop_vehicle(&vehicles[j]);
                }
            }
        }

        // Increment waiting time for stopped vehicles and attempt to resume one vehicle
        // using a simple arbitration: higher wait_time wins, but only if safe to resume.
        for(int i = 0; i < num_vehicles; i++){
            if(vehicles[i].stopped){
                vehicles[i].wait_time++;
            }
        }

        for(int i = 0; i < num_vehicles; i++){
            for(int j = i + 1; j < num_vehicles; j++){
                if(vehicles[i].stopped && vehicles[j].stopped){
                    int pick = -1;
                    if(vehicles[i].wait_time > vehicles[j].wait_time) pick = i;
                    else if(vehicles[j].wait_time > vehicles[i].wait_time) pick = j;
                    else pick = i; // tie-breaker: lower index

                    // Try to resume the picked vehicle if safe; otherwise try the other.
                    if(pick != -1){
                        int other = (pick == i) ? j : i;
                        if(vehicles[pick].wait_time > vehicles[other].wait_time){
                            if(can_resume_vehicle(pick, 3)){
                                printf("Resuming vehicle %d: higher wait_time (%d > %d)\n", pick, vehicles[pick].wait_time, vehicles[other].wait_time);
                                resume_vehicle(pick);
                            } else if(can_resume_vehicle(other, 3)){
                                printf("Resuming vehicle %d: picked vehicle %d couldn't resume, other is safe\n", other, pick);
                                resume_vehicle(other);
                            }
                        } else if(vehicles[other].wait_time > vehicles[pick].wait_time){
                            if(can_resume_vehicle(other, 3)){
                                printf("Resuming vehicle %d: higher wait_time (%d > %d)\n", other, vehicles[other].wait_time, vehicles[pick].wait_time);
                                resume_vehicle(other);
                            } else if(can_resume_vehicle(pick, 3)){
                                printf("Resuming vehicle %d: picked vehicle %d couldn't resume, other is safe\n", pick, other);
                                resume_vehicle(pick);
                            }
                        } else {
                            // tie -> lower index wins
                            int winner = (i < j) ? i : j;
                            if(can_resume_vehicle(winner, 3)){
                                printf("Resuming vehicle %d: tie on wait_time, lower index wins\n", winner);
                                resume_vehicle(winner);
                            } else {
                                int loser = (winner == i) ? j : i;
                                if(can_resume_vehicle(loser, 3)){
                                    printf("Resuming vehicle %d: winner couldn't resume, other is safe\n", loser);
                                    resume_vehicle(loser);
                                }
                            }
                        }
                    }
                } else if(vehicles[i].stopped && !vehicles[j].stopped){
                    // If only i is stopped, see if it can safely resume now.
                    if(can_resume_vehicle(i, 3)){
                        printf("Resuming vehicle %d: safe to resume (no predicted collisions)\n", i);
                        resume_vehicle(i);
                    }
                } else if(!vehicles[i].stopped && vehicles[j].stopped){
                    if(can_resume_vehicle(j, 3)){
                        printf("Resuming vehicle %d: safe to resume (no predicted collisions)\n", j);
                        resume_vehicle(j);
                    }
                }
            }
        }

        printf("\033[2J\033[H");
        for (int i = 0; i < num_vehicles; i++) {
            move_vehicle(&vehicles[i]);
        }
        draw_scene(roads, num_roads, vehicles, num_vehicles, canvaSide);
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
    if (argc != 1) return;

    // Prefer user-specified terminal via $TERMINAL
    const char *env_term = getenv("TERMINAL");
    char cmd[1024];
    if (env_term && *env_term) {
        snprintf(cmd, sizeof(cmd), "command -v %s >/dev/null 2>&1", env_term);
        if (system(cmd) == 0) {
            snprintf(cmd, sizeof(cmd), "%s -e %s --child &", env_term, argv[0]);
            if (system(cmd) != -1) exit(0);
        }
    }

    // Fallback to common terminal emulators
    const char *candidates[] = {"x-terminal-emulator", "xterm", "gnome-terminal", "konsole", "alacritty", "kitty", "urxvt", "st", NULL};
    for (int i = 0; candidates[i]; i++) {
        const char *term = candidates[i];
        snprintf(cmd, sizeof(cmd), "command -v %s >/dev/null 2>&1", term);
        if (system(cmd) == 0) {
            if (strcmp(term, "gnome-terminal") == 0) {
                snprintf(cmd, sizeof(cmd), "%s -- %s --child &", term, argv[0]);
            } else {
                snprintf(cmd, sizeof(cmd), "%s -e %s --child &", term, argv[0]);
            }
            if (system(cmd) != -1) exit(0);
        }
    }

    // No terminal found: continue in current terminal.
}

int colli_avoid(struct Pixel* front_pixel, int steps){
    // Check a 7x7 area centered on front_pixel (offsets -3..3 in x and y).
    // For any pixel present in that area (from any vehicle in the global registry),
    // predict whether it will collide with front_pixel within `distance` steps.
    if(front_pixel == NULL) return 0;
    if(g_vehicles == NULL || g_num_vehicles == 0) return 0;

    for(int oy = -3; oy <= 3; oy++){
        for(int ox = -3; ox <= 3; ox++){
            int nx = front_pixel->x + ox;
            int ny = front_pixel->y + oy;

            // Skip checking the centre pixel (same pixel) - optional.
            if(ox == 0 && oy == 0) continue;

            // Scan all known vehicle pixels to see if any occupy (nx, ny).
            for(int vi = 0; vi < g_num_vehicles; vi++){
                struct Vehicle* v = &g_vehicles[vi];
                for(int pi = 0; pi < v->num_pixels; pi++){
                    struct Pixel* p = v->pixels[pi];
                    if(p->x == nx && p->y == ny){
                        // Found a lit pixel in the area; predict collision.
                        if(predict_pixel_collision_from_pixels(front_pixel, p, steps)){
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int main(int argc, char *argv[]){
    // Ensure the application is running in its own terminal window.
    handle_terminal_relaunch(argc, argv);

    // Preset figures.
    int roadNum = 3, 
        vehiNum = 3, 
        canvaSide = 30, 
        stepCount = 15,
        maxSteps = 3;

    struct Road roads[roadNum];
    struct Vehicle vehicles[vehiNum];

    make_road(&roads[0], 0, 5, 29, 8);
    make_road(&roads[1], 10, 0, 13, 29);
    make_road(&roads[2], 10, 12, 29, 15);

    make_vehicle(&vehicles[0], 0, 8, 255, 0, 0, 2, 0, 3, 1);
    make_vehicle(&vehicles[1], 10, 0, 0, 255, 0, 0, 1, 2, 4);
    make_vehicle(&vehicles[2], 20, 5, 0, 0, 255, -2, 0, 5, 1);

    // Resize the terminal to match the canvas before drawing.
    resize_terminal(canvaSide + 6, canvaSide + 2);

    // Populate global registry for helper functions (e.g., colli_avoid).
    g_vehicles = vehicles;
    g_num_vehicles = vehiNum;

    make_map(roads, roadNum, vehicles, vehiNum);
    run_animation(roads, roadNum, vehicles, vehiNum, canvaSide, stepCount);

    // Keep the terminal open until the user interacts.
    printf("\nAnimation complete. Press Enter to close the window...");
    getchar();

    free_memory(roads, roadNum, vehicles, vehiNum);
    return 0;
}
