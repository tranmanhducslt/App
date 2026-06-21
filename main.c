#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

struct Pixel{
    int x, y;
    int r, g, b;
    int dx, dy;
};

struct Road{
    int x1, y1, x2, y2; 
    struct Pixel** pixels; 
    int length, width;
};

struct Vehicle{
    int x, y;
    int r, g, b;
    int dx, dy; 
    struct Pixel** pixels; 
    int length, width, num_pixels; 
    int orig_dx, orig_dy;
};

// Global registries
struct Vehicle* g_vehicles = NULL;
int g_num_vehicles = 0;

// Spacetime Grid Pointer: 3D array layout [X][Y][Time_Step]
// 0 means free, any positive number (vehicle_index + 1) means reserved by that vehicle.
int*** g_spacetime_grid = NULL;
int g_canvas_side = 0;
int g_max_lookahead = 0;

void init_spacetime_grid(int canvaSide, int maxSteps) {
    g_canvas_side = canvaSide;
    g_max_lookahead = maxSteps;
    
    g_spacetime_grid = (int***)malloc(canvaSide * sizeof(int**));
    for (int x = 0; x < canvaSide; x++) {
        g_spacetime_grid[x] = (int**)malloc(canvaSide * sizeof(int*));
        for (int y = 0; y < canvaSide; y++) {
            g_spacetime_grid[x][y] = (int*)calloc(maxSteps + 1, sizeof(int));
        }
    }
}

void clear_spacetime_grid() {
    for (int x = 0; x < g_canvas_side; x++) {
        for (int y = 0; y < g_canvas_side; y++) {
            memset(g_spacetime_grid[x][y], 0, (g_max_lookahead + 1) * sizeof(int));
        }
    }
}

void free_spacetime_grid() {
    if (!g_spacetime_grid) return;
    for (int x = 0; x < g_canvas_side; x++) {
        for (int y = 0; y < g_canvas_side; y++) {
            free(g_spacetime_grid[x][y]);
        }
        free(g_spacetime_grid[x]);
    }
    free(g_spacetime_grid);
}

void make_road(struct Road* road, int x1, int y1, int x2, int y2){
    road->x1 = x1; road->y1 = y1;
    road->x2 = x2; road->y2 = y2;
    int length = abs(x2 - x1) + 1, width = abs(y2 - y1) + 1;
    road->length = length;
    road->width = width;
    road->pixels = (struct Pixel**)malloc(length * width * sizeof(struct Pixel*));
    for(int i = 0; i < length; i++){
        for(int j = 0; j < width; j++){
            road->pixels[i * width + j] = (struct Pixel*)malloc(sizeof(struct Pixel));
            road->pixels[i * width + j]->x = x1 + i;
            road->pixels[i * width + j]->y = y1 + j;
            road->pixels[i * width + j]->r = 128;
            road->pixels[i * width + j]->g = 128;
            road->pixels[i * width + j]->b = 128;
            road->pixels[i * width + j]->dx = 0;
            road->pixels[i * width + j]->dy = 0;
        }
    }
}

void build_vehicle_pixels(struct Vehicle* vehicle) {
    vehicle->num_pixels = vehicle->length * vehicle->width;
    vehicle->pixels = (struct Pixel**)malloc(vehicle->num_pixels * sizeof(struct Pixel*));
    for(int i = 0; i < vehicle->length; i++){
        for(int j = 0; j < vehicle->width; j++){
            int idx = i * vehicle->width + j;
            vehicle->pixels[idx] = (struct Pixel*)malloc(sizeof(struct Pixel));
            vehicle->pixels[idx]->x = vehicle->x + i;
            vehicle->pixels[idx]->y = vehicle->y + j;
            vehicle->pixels[idx]->r = vehicle->r;
            vehicle->pixels[idx]->g = vehicle->g;
            vehicle->pixels[idx]->b = vehicle->b;
            vehicle->pixels[idx]->dx = vehicle->dx;
            vehicle->pixels[idx]->dy = vehicle->dy;
        }
    }
}

void make_vehicle(struct Vehicle* vehicle, int x, int y, int r, int g, int b, int dx, int dy, int length, int width){
    vehicle->x = x; vehicle->y = y;
    vehicle->r = r; vehicle->g = g; vehicle->b = b;
    vehicle->dx = dx; vehicle->dy = dy;
    vehicle->orig_dx = dx; vehicle->orig_dy = dy;
    vehicle->length = length; vehicle->width = width;
    build_vehicle_pixels(vehicle);
}

void turn_and_flip_vehicle(struct Vehicle* v, int new_dx, int new_dy, int target_x, int target_y) {
    int current_horizontal = (v->orig_dx != 0);
    int new_horizontal = (new_dx != 0);
    
    if (current_horizontal != new_horizontal) {
        int temp = v->length;
        v->length = v->width;
        v->width = temp;
    }

    for (int i = 0; i < v->num_pixels; i++) {
        free(v->pixels[i]);
    }
    free(v->pixels);

    v->x = target_x;
    v->y = target_y;
    v->dx = new_dx;
    v->dy = new_dy;
    v->orig_dx = new_dx;
    v->orig_dy = new_dy;

    build_vehicle_pixels(v);
}

void move_vehicle(struct Vehicle* vehicle){
    for(int i = 0; i < vehicle->num_pixels; i++){
        vehicle->pixels[i]->x += vehicle->dx;
        vehicle->pixels[i]->y += vehicle->dy;
    }
    vehicle->x += vehicle->dx;
    vehicle->y += vehicle->dy;
}

// Map planned trajectory into the Spacetime Grid
// Returns the earliest conflict timestep found (0 if no conflict)
int check_and_reserve_spacetime(int vehicle_idx) {
    struct Vehicle* v = &g_vehicles[vehicle_idx];
    int earliest_conflict = 0;

    // We check future states sequentially
    for (int step = 1; step <= g_max_lookahead; step++) {
        int conflict_at_step = 0;

        // Project where all pixels of this vehicle will be at this future time step
        for (int i = 0; i < v->num_pixels; i++) {
            int proj_x = v->pixels[i]->x + (v->orig_dx * step);
            int proj_y = v->pixels[i]->y + (v->orig_dy * step);

            // Keep bounds checked within canvas
            if (proj_x >= 0 && proj_x < g_canvas_side && proj_y >= 0 && proj_y < g_canvas_side) {
                int occupant = g_spacetime_grid[proj_x][proj_y][step];
                if (occupant != 0 && occupant != (vehicle_idx + 1)) {
                    conflict_at_step = 1;
                    if (earliest_conflict == 0) {
                        earliest_conflict = step;
                    }
                }
            }
        }

        // If no conflict at this step, lock the pixels down under our vehicle's ID
        if (!conflict_at_step) {
            for (int i = 0; i < v->num_pixels; i++) {
                int proj_x = v->pixels[i]->x + (v->orig_dx * step);
                int proj_y = v->pixels[i]->y + (v->orig_dy * step);
                if (proj_x >= 0 && proj_x < g_canvas_side && proj_y >= 0 && proj_y < g_canvas_side) {
                    g_spacetime_grid[proj_x][proj_y][step] = (vehicle_idx + 1);
                }
            }
        }
    }
    return earliest_conflict;
}

void apply_spacetime_arbitration() {
    clear_spacetime_grid();

    // Vehicles with lower indices have higher priority for reservations
    for (int i = 0; i < g_num_vehicles; i++) {
        int conflict_step = check_and_reserve_spacetime(i);
        
        if (conflict_step > 0) {
            // Collision predicted! Yield dynamically
            if (conflict_step == 1) {
                // Imminent collision: Full Brake
                g_vehicles[i].dx = 0;
                g_vehicles[i].dy = 0;
                printf("Spacetime Alert: Vehicle %d executing emergency stop.\n", i);
            } else {
                // Future collision: Smooth down velocity proportionally
                g_vehicles[i].dx = g_vehicles[i].orig_dx / 2;
                g_vehicles[i].dy = g_vehicles[i].orig_dy / 2;
                printf("Spacetime Alert: Vehicle %d slowing down (Conflict at Step %d).\n", i, conflict_step);
            }
        } else {
            // Path clear: Resume original cruise velocity
            g_vehicles[i].dx = g_vehicles[i].orig_dx;
            g_vehicles[i].dy = g_vehicles[i].orig_dy;
        }

        // Apply updated step velocities to internal pixels
        for (int p = 0; p < g_vehicles[i].num_pixels; p++) {
            g_vehicles[i].pixels[p]->dx = g_vehicles[i].dx;
            g_vehicles[i].pixels[p]->dy = g_vehicles[i].dy;
        }
    }
}

void resize_terminal(int rows, int cols){
    struct winsize ws;
    ws.ws_row = rows;
    ws.ws_col = cols;
    ws.ws_xpixel = 0;
    ws.ws_ypixel = 0;
    if(ioctl(STDOUT_FILENO, TIOCSWINSZ, &ws) == 0){
        printf("\033[8;%d;%dt", rows, cols);
        fflush(stdout);
    }
}

void draw_scene(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles, int canvaSide){
    char scene[canvaSide][canvaSide];
    int r[canvaSide][canvaSide], g[canvaSide][canvaSide], b[canvaSide][canvaSide];

    for(int y = 0; y < canvaSide; y++){
        for(int x = 0; x < canvaSide; x++){
            scene[y][x] = ' ';
            r[y][x] = 255; g[y][x] = 255; b[y][x] = 255;
        }
    }

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
        
        /*
        if (step == 6) {
            printf("\n--- Routing Event: Vehicle 1 is turning right & flipping structural dimensions! ---\n");
            turn_and_flip_vehicle(&vehicles[1], -2, 0, 14, 12);
            usleep(1000000); 
        }
        */

        apply_spacetime_arbitration();

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
    free_spacetime_grid();
}

void handle_terminal_relaunch(int argc, char *argv[]) {
    if (argc != 1) return;
    const char *env_term = getenv("TERMINAL");
    char cmd[1024];
    if (env_term && *env_term) {
        snprintf(cmd, sizeof(cmd), "command -v %s >/dev/null 2>&1", env_term);
        if (system(cmd) == 0) {
            snprintf(cmd, sizeof(cmd), "%s -e %s --child &", env_term, argv[0]);
            if (system(cmd) != -1) exit(0);
        }
    }
    const char *candidates[] = {"x-terminal-emulator", "xterm", "gnome-terminal", "konsole", "alacritty", "kitty", "urvxt", "st", NULL};
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
}

int main(int argc, char *argv[]){
    handle_terminal_relaunch(argc, argv);

    // Grid properties (Ready to upscale to 100+)
    int roadNum = 3, 
        vehiNum = 3, 
        canvaSide = 40, 
        stepCount = 20,
        maxSteps = 4; // Lookahead time window horizon

    init_spacetime_grid(canvaSide, maxSteps);

    struct Road roads[roadNum];
    struct Vehicle vehicles[vehiNum];

    make_road(&roads[0], 0, 5, 39, 8);
    make_road(&roads[1], 10, 0, 15, 39);
    make_road(&roads[2], 10, 12, 39, 15);

    make_vehicle(&vehicles[0], 0, 7, 255, 0, 0, 2, 0, 3, 1);
    make_vehicle(&vehicles[1], 11, 0, 0, 255, 0, 0, 1, 2, 4);
    make_vehicle(&vehicles[2], 30, 5, 0, 0, 255, -3, 0, 5, 1);

    // Resize window to accommodate size for the map.
    resize_terminal(canvaSide + 6, canvaSide + 2);

    g_vehicles = vehicles;
    g_num_vehicles = vehiNum;

    draw_scene(roads, roadNum, vehicles, vehiNum, canvaSide);
    run_animation(roads, roadNum, vehicles, vehiNum, canvaSide, stepCount);

    printf("\nAnimation complete. Press Enter to close...");
    getchar();

    free_memory(roads, roadNum, vehicles, vehiNum);
    return 0;
}