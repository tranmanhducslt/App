#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <time.h>

// Compile-time upper bound for the planned-path arrays stored on each vehicle.
// Must be >= maxSteps passed to init_spacetime_grid().
#define MAX_LOOKAHEAD 32

/* ═══════════════════════════════════════════════════════════════════════════
   Data Structures
═══════════════════════════════════════════════════════════════════════════ */

struct Pixel {
    int x, y;
    int r, g, b;
    int dx, dy;
};

struct Road {
    int x1, y1, x2, y2;
    struct Pixel** pixels;
    int length, width;
};

struct Vehicle {
    int x, y;           // Anchor (top-left corner) of bounding box
    int r, g, b;        // Colour
    int dx, dy;         // Step velocity committed by the planner for this tick
    struct Pixel** pixels;
    int length, width, num_pixels;
    int orig_dx, orig_dy; // Initial direction hint (kept for reference)

    // ── WHCA* fields ──────────────────────────────────────────────
    int priority;                   // Planning order: lower = earlier, higher priority
    int goal_x, goal_y;             // Destination anchor the vehicle is driving toward
    int planned_dx[MAX_LOOKAHEAD];  // Unit moves planned by A* for each lookahead step
    int planned_dy[MAX_LOOKAHEAD];
};

/* ═══════════════════════════════════════════════════════════════════════════
   Global State
═══════════════════════════════════════════════════════════════════════════ */

struct Vehicle* g_vehicles     = NULL;
int             g_num_vehicles = 0;

// Road constraints
struct Road*    g_roads         = NULL;
int             g_num_roads     = 0;

// Spacetime reservation grid [x][y][t].
// 0 = free; positive integer N means "reserved by vehicle index N-1".
int*** g_spacetime_grid = NULL;
int   g_canvas_side     = 0;
int   g_max_lookahead   = 0;

/* ═══════════════════════════════════════════════════════════════════════════
   Spacetime Grid
═══════════════════════════════════════════════════════════════════════════ */

void init_spacetime_grid(int side, int steps) {
    g_canvas_side   = side;
    g_max_lookahead = steps;
    g_spacetime_grid = malloc(side * sizeof(int**));
    for (int x = 0; x < side; x++) {
        g_spacetime_grid[x] = malloc(side * sizeof(int*));
        for (int y = 0; y < side; y++)
            g_spacetime_grid[x][y] = calloc(steps + 1, sizeof(int));
    }
}

void clear_spacetime_grid() {
    for (int x = 0; x < g_canvas_side; x++)
        for (int y = 0; y < g_canvas_side; y++)
            memset(g_spacetime_grid[x][y], 0, (g_max_lookahead + 1) * sizeof(int));
}

void free_spacetime_grid() {
    if (!g_spacetime_grid) return;
    for (int x = 0; x < g_canvas_side; x++) {
        for (int y = 0; y < g_canvas_side; y++) free(g_spacetime_grid[x][y]);
        free(g_spacetime_grid[x]);
    }
    free(g_spacetime_grid);
}

/* ═══════════════════════════════════════════════════════════════════════════
   Road & Vehicle Construction
═══════════════════════════════════════════════════════════════════════════ */

void make_road(struct Road* road, int x1, int y1, int x2, int y2) {
    road->x1 = x1; road->y1 = y1;
    road->x2 = x2; road->y2 = y2;
    int length = abs(x2 - x1) + 1, width = abs(y2 - y1) + 1;
    road->length = length; road->width = width;
    road->pixels = malloc(length * width * sizeof(struct Pixel*));
    for (int i = 0; i < length; i++) {
        for (int j = 0; j < width; j++) {
            struct Pixel* p = malloc(sizeof(struct Pixel));
            p->x = x1 + i; p->y = y1 + j;
            p->r = 128; p->g = 128; p->b = 128;
            p->dx = p->dy = 0;
            road->pixels[i * width + j] = p;
        }
    }
}

void build_vehicle_pixels(struct Vehicle* v) {
    v->num_pixels = v->length * v->width;
    v->pixels = malloc(v->num_pixels * sizeof(struct Pixel*));
    for (int i = 0; i < v->length; i++) {
        for (int j = 0; j < v->width; j++) {
            int idx = i * v->width + j;
            struct Pixel* p = malloc(sizeof(struct Pixel));
            p->x = v->x + i; p->y = v->y + j;
            p->r = v->r; p->g = v->g; p->b = v->b;
            p->dx = v->dx; p->dy = v->dy;
            v->pixels[idx] = p;
        }
    }
}

// Construct a vehicle.  dx/dy are kept as a direction hint; the WHCA* planner
// will override them with unit-step A* moves each tick.
void make_vehicle(struct Vehicle* v,
                  int x, int y, int r, int g, int b,
                  int dx, int dy, int length, int width,
                  int priority, int goal_x, int goal_y) {
    v->x = x; v->y = y;
    v->r = r; v->g = g; v->b = b;
    v->dx = dx; v->dy = dy;
    v->orig_dx = dx; v->orig_dy = dy;
    v->length = length; v->width = width;
    v->priority  = priority;
    v->goal_x = goal_x; v->goal_y = goal_y;
    memset(v->planned_dx, 0, sizeof(v->planned_dx));
    memset(v->planned_dy, 0, sizeof(v->planned_dy));
    build_vehicle_pixels(v);
}

// Placeholder vehicle turner. Will change later.

void turn_vehicle(struct Vehicle* v, int new_dx, int new_dy, int new_x, int new_y) {
    // 1. Update anchor and velocity hints
    v->x = new_x;
    v->y = new_y;
    v->dx = new_dx;
    v->dy = new_dy;
    v->orig_dx = new_dx;
    v->orig_dy = new_dy;

    // 2. Rotate dimensions (swap length and width if changing orientation)
    int old_oriented_horizontally = (v->orig_dx != 0);
    int new_oriented_horizontally = (new_dx != 0);
    if (old_oriented_horizontally != new_oriented_horizontally) {
        int temp = v->length;
        v->length = v->width;
        v->width = temp;
    }

    // 3. Free old pixels and rebuild them at the new position/orientation
    for (int j = 0; j < v->num_pixels; j++) {
        free(v->pixels[j]);
    }
    free(v->pixels);
    build_vehicle_pixels(v);
}

void move_vehicle(struct Vehicle* v) {
    v->x += v->dx; v->y += v->dy;
    for (int i = 0; i < v->num_pixels; i++) {
        v->pixels[i]->x += v->dx;
        v->pixels[i]->y += v->dy;
    }
}

int is_vehicle_on_road(struct Vehicle v, struct Road road){
    return (v.x >= road.x1 && v.x + v.length - 1 <= road.x2 &&
            v.y >= road.y1 && v.y + v.width - 1 <= road.y2);
}

int is_vehicle_on_any_road(struct Vehicle v, struct Road* roads, int num_roads){
    for (int i = 0; i < num_roads; i++){
        if (is_vehicle_on_road(v, roads[i])){
            return 1;
        }
    }
    return 0;
}

int is_vehicle_overlapping_others(struct Vehicle v, struct Vehicle* vehicles, int num_vehicles, int self_index){
    for (int i = 0; i < num_vehicles; i++){
        if (i == self_index) continue;
        struct Vehicle other = vehicles[i];
        if (v.x < other.x + other.length && v.x + v.length > other.x &&
            v.y < other.y + other.width && v.y + v.width > other.y){
            return 1; // Overlap detected
        }
    }
    return 0; // No overlap
}

/* ═══════════════════════════════════════════════════════════════════════════
   WHCA* — Windowed Hierarchical Cooperative A*
   ─────────────────────────────────────────────
   Each vehicle runs spacetime A* within a sliding time window [0, g_max_lookahead].
   Vehicles plan in ascending priority order; each planner sees the reservations
   already placed by higher-priority vehicles and treats them as hard obstacles.
   Only the first planned move (dx[0], dy[0]) is executed per tick; the window
   then re-opens from the new position next frame.
═══════════════════════════════════════════════════════════════════════════ */

// ── Min-heap (open set for A*) ────────────────────────────────────────────

typedef struct { int x, y, t, g, f; } HNode;
typedef struct { HNode* data; int size, cap; } MinHeap;

static void mh_swap(MinHeap* h, int a, int b) {
    HNode tmp = h->data[a]; h->data[a] = h->data[b]; h->data[b] = tmp;
}

static void mh_push(MinHeap* h, HNode n) {
    if (h->size >= h->cap) return; // should never overflow given cap = nstates
    h->data[h->size++] = n;
    for (int i = h->size - 1; i > 0;) {
        int p = (i - 1) / 2;
        if (h->data[p].f > h->data[i].f) { mh_swap(h, p, i); i = p; }
        else break;
    }
}

static HNode mh_pop(MinHeap* h) {
    HNode top = h->data[0];
    h->data[0] = h->data[--h->size];
    for (int i = 0;;) {
        int l = 2*i+1, r = 2*i+2, s = i;
        if (l < h->size && h->data[l].f < h->data[s].f) s = l;
        if (r < h->size && h->data[r].f < h->data[s].f) s = r;
        if (s == i) break;
        mh_swap(h, i, s); i = s;
    }
    return top;
}

// ── Came-from table (path reconstruction) ────────────────────────────────

// Records how we arrived at each (x, y, t) cell during A*.
typedef struct { int px, py, pt, mdx, mdy; } CFEntry;

// Flat index into the (x, y, t) state space.
#define SIDX(x, y, t) (((x) * g_canvas_side + (y)) * (g_max_lookahead + 1) + (t))

// ── Collision predicate ───────────────────────────────────────────────────

// Returns 1 iff vehicle vid's bounding box placed at anchor (ax, ay) is fully
// within bounds, free of foreign reservations at timestep t, and entirely on roads.
static int pos_free(int vid, int ax, int ay, int t) {
    struct Vehicle* v = &g_vehicles[vid];
    
    // Check bounds and reservations for each cell of the vehicle
    for (int i = 0; i < v->length; i++) {
        for (int j = 0; j < v->width; j++) {
            int px = ax + i, py = ay + j;
            if (px < 0 || px >= g_canvas_side ||
                py < 0 || py >= g_canvas_side) return 0;
            if (t >= 0 && t <= g_max_lookahead) {
                int occ = g_spacetime_grid[px][py][t];
                if (occ != 0 && occ != vid + 1) return 0;
            }
        }
    }
    
    // Check that the vehicle is entirely on roads
    struct Vehicle temp_v = *v;
    temp_v.x = ax;
    temp_v.y = ay;
    if (!is_vehicle_on_any_road(temp_v, g_roads, g_num_roads)) {
        return 0;
    }
    if (is_vehicle_overlapping_others(temp_v, g_vehicles, g_num_vehicles, vid)) {
        return 0;
    }
    
    return 1;
}

// ── Spacetime A* for one vehicle ─────────────────────────────────────────

// Plans vehicle vid's path to its goal within the current lookahead window.
//   • Fills planned_dx[] / planned_dy[] with unit moves.
//   • Reserves those cells in the shared spacetime grid.
//   • Sets v->dx / v->dy to the first planned step (executed this tick).
// Returns 1 if the goal is reachable within the window, 0 for a partial path.
static int sign(int val) {
    return (val > 0) - (val < 0);
}

// Plans vehicle vid's path to its goal within the current lookahead window.
//   • Fills planned_dx[] / planned_dy[] with moves.
//   • Reserves those cells (including intermediate swept cells) in the shared spacetime grid.
//   • Sets v->dx / v->dy to the first planned step (executed this tick).
// Returns 1 if the goal is reachable within the window, 0 for a partial path.
static int whca_astar(int vid) {
    struct Vehicle* v = &g_vehicles[vid];
    const int sx = v->x, sy = v->y;
    const int gx = v->goal_x, gy = v->goal_y;

    int ns = g_canvas_side * g_canvas_side * (g_max_lookahead + 1);

    CFEntry* cf   = calloc(ns, sizeof(CFEntry));
    int*     gval = malloc(ns * sizeof(int));
    for (int i = 0; i < ns; i++) gval[i] = INT_MAX;

    MinHeap open = { malloc(ns * sizeof(HNode)), 0, ns };

    // Seed the start state
    int h0 = abs(sx - gx) + abs(sy - gy);
    gval[SIDX(sx, sy, 0)] = 0;
    cf[SIDX(sx, sy, 0)].pt = -1;          // sentinel: no parent
    mh_push(&open, (HNode){sx, sy, 0, 0, h0});

    int spd = abs(v->orig_dx) + abs(v->orig_dy);
    if (spd == 0) spd = 1; // safeguard

    int found = 0;
    int bx = sx, by = sy, bt = 0; // best endpoint found (lowest h)
    int bh = h0;

    while (open.size > 0) {
        HNode cur = mh_pop(&open);
        int cx = cur.x, cy = cur.y, ct = cur.t;

        if (cur.g > gval[SIDX(cx, cy, ct)]) continue; // stale duplicate

        if (cx == gx && cy == gy) {
            // Goal reached within window
            found = 1; bx = cx; by = cy; bt = ct;
            break;
        }

        // Track the node closest to goal as a fallback for partial paths
        int ch = abs(cx - gx) + abs(cy - gy);
        if (ch < bh) { bh = ch; bx = cx; by = cy; bt = ct; }

        if (ct >= g_max_lookahead) continue; // window exhausted, can't expand

        // 1. Wait action (move of 0 steps)
        {
            int nx = cx, ny = cy, nt = ct + 1;
            if (pos_free(vid, nx, ny, nt)) {
                int ng = cur.g + 1, ni = SIDX(nx, ny, nt);
                if (ng < gval[ni]) {
                    gval[ni] = ng;
                    cf[ni] = (CFEntry){ cx, cy, ct, 0, 0 };
                    int nh = abs(nx - gx) + abs(ny - gy);
                    mh_push(&open, (HNode){ nx, ny, nt, ng, ng + nh });
                }
            }
        }

        // 2. Cardinal moves: up to speed 'spd' in each direction
        const int dx_dir[] = { 1, -1,  0,  0 };
        const int dy_dir[] = { 0,  0,  1, -1 };
        for (int d = 0; d < 4; d++) {
            for (int k = 1; k <= spd; k++) {
                int mx = dx_dir[d] * k;
                int my = dy_dir[d] * k;
                int nx = cx + mx, ny = cy + my, nt = ct + 1;

                // Check all cells swept through (Option B)
                int possible = 1;
                for (int step = 1; step <= k; step++) {
                    int ix = cx + dx_dir[d] * step;
                    int iy = cy + dy_dir[d] * step;
                    if (!pos_free(vid, ix, iy, nt)) {
                        possible = 0;
                        break;
                    }
                }
                if (!possible) continue;

                int ng = cur.g + 1, ni = SIDX(nx, ny, nt);
                if (ng < gval[ni]) {
                    gval[ni] = ng;
                    cf[ni] = (CFEntry){ cx, cy, ct, mx, my };
                    int nh = abs(nx - gx) + abs(ny - gy);
                    mh_push(&open, (HNode){ nx, ny, nt, ng, ng + nh });
                }
            }
        }
    }

    // ── Reconstruct planned moves by walking the came-from chain ──────────
    memset(v->planned_dx, 0, sizeof(v->planned_dx));
    memset(v->planned_dy, 0, sizeof(v->planned_dy));

    for (int tx = bx, ty = by, tt = bt; tt > 0;) {
        int idx = SIDX(tx, ty, tt);
        v->planned_dx[tt - 1] = cf[idx].mdx;
        v->planned_dy[tt - 1] = cf[idx].mdy;
        int npx = cf[idx].px, npy = cf[idx].py, npt = cf[idx].pt;
        tx = npx; ty = npy; tt = npt;
    }

    // ── Reserve the planned path in the shared spacetime grid ─────────────
    int ax = sx, ay = sy;
    // Reserve at s = 0 (starting position)
    for (int i = 0; i < v->length; i++) {
        for (int j = 0; j < v->width; j++) {
            int px = ax + i, py = ay + j;
            if (px >= 0 && px < g_canvas_side &&
                py >= 0 && py < g_canvas_side)
                g_spacetime_grid[px][py][0] = vid + 1;
        }
    }
    // Reserve at s > 0 (subsequent timesteps)
    for (int s = 1; s <= g_max_lookahead; s++) {
        int prev_x = ax, prev_y = ay;
        ax += v->planned_dx[s - 1];
        ay += v->planned_dy[s - 1];

        int dx = v->planned_dx[s - 1];
        int dy = v->planned_dy[s - 1];
        int steps = abs(dx) + abs(dy);
        if (steps == 0) {
            for (int i = 0; i < v->length; i++) {
                for (int j = 0; j < v->width; j++) {
                    int px = ax + i, py = ay + j;
                    if (px >= 0 && px < g_canvas_side &&
                        py >= 0 && py < g_canvas_side)
                        g_spacetime_grid[px][py][s] = vid + 1;
                }
            }
        } else {
            int sdx = sign(dx);
            int sdy = sign(dy);
            for (int step = 0; step <= steps; step++) {
                int ix = prev_x + sdx * step;
                int iy = prev_y + sdy * step;
                for (int i = 0; i < v->length; i++) {
                    for (int j = 0; j < v->width; j++) {
                        int px = ix + i, py = iy + j;
                        if (px >= 0 && px < g_canvas_side &&
                            py >= 0 && py < g_canvas_side)
                            g_spacetime_grid[px][py][s] = vid + 1;
                    }
                }
            }
        }
    }

    // ── Commit first step as this tick's velocity ─────────────────────────
    v->dx = v->planned_dx[0];
    v->dy = v->planned_dy[0];
    for (int p = 0; p < v->num_pixels; p++) {
        v->pixels[p]->dx = v->dx;
        v->pixels[p]->dy = v->dy;
    }

    free(cf); free(gval); free(open.data);
    return found;
}

// ── Priority comparator for qsort ────────────────────────────────────────

static int cmp_priority(const void* a, const void* b) {
    return g_vehicles[*(const int*)a].priority
         - g_vehicles[*(const int*)b].priority;
}

// ── Top-level WHCA* planner ───────────────────────────────────────────────

// Plans all vehicles cooperatively for this tick.
// Vehicles are processed in ascending priority order; each one sees the
// spacetime reservations already placed by higher-priority vehicles.
void apply_whca_star(struct Road* roads, int num_roads) {
    g_roads = roads;
    g_num_roads = num_roads;
    
    clear_spacetime_grid();

    // Build priority-sorted planning order
    int* order = malloc(g_num_vehicles * sizeof(int));
    for (int i = 0; i < g_num_vehicles; i++) order[i] = i;
    qsort(order, g_num_vehicles, sizeof(int), cmp_priority);

    for (int k = 0; k < g_num_vehicles; k++) {
        int vi = order[k];
        struct Vehicle* v = &g_vehicles[vi];

        if (v->x == v->goal_x && v->y == v->goal_y) {
            // Vehicle has reached its goal: park it and hold the space for the
            // full window so it remains a hard obstacle for lower-priority peers.
            v->dx = v->dy = 0;
            for (int p = 0; p < v->num_pixels; p++)
                v->pixels[p]->dx = v->pixels[p]->dy = 0;
            for (int s = 0; s <= g_max_lookahead; s++)
                for (int i = 0; i < v->length; i++)
                    for (int j = 0; j < v->width; j++) {
                        int px = v->x + i, py = v->y + j;
                        if (px >= 0 && px < g_canvas_side &&
                            py >= 0 && py < g_canvas_side)
                            g_spacetime_grid[px][py][s] = vi + 1;
                    }
            continue;
        }

        whca_astar(vi);
    }

    free(order);
}

/* ═══════════════════════════════════════════════════════════════════════════
   Terminal & Rendering
═══════════════════════════════════════════════════════════════════════════ */

void resize_terminal(int rows, int cols) {
    struct winsize ws = { rows, cols, 0, 0 };
    if (ioctl(STDOUT_FILENO, TIOCSWINSZ, &ws) == 0) {
        printf("\033[8;%d;%dt", rows, cols);
        fflush(stdout);
    }
}

void draw_scene(struct Road* roads, int num_roads,
                struct Vehicle* vehicles, int num_vehicles, int side) {
    char scene[side][side];
    int  rc[side][side], gc[side][side], bc[side][side];

    for (int y = 0; y < side; y++)
        for (int x = 0; x < side; x++) {
            scene[y][x] = ' ';
            rc[y][x] = gc[y][x] = bc[y][x] = 255;
        }

    for (int i = 0; i < num_roads; i++)
        for (int j = 0; j < roads[i].length * roads[i].width; j++) {
            int x = roads[i].pixels[j]->x, y = roads[i].pixels[j]->y;
            if (x >= 0 && x < side && y >= 0 && y < side) {
                scene[y][x] = '.';
                rc[y][x] = gc[y][x] = bc[y][x] = 128;
            }
        }

    for (int i = 0; i < num_vehicles; i++)
        for (int j = 0; j < vehicles[i].num_pixels; j++) {
            int x = vehicles[i].pixels[j]->x, y = vehicles[i].pixels[j]->y;
            if (x >= 0 && x < side && y >= 0 && y < side) {
                scene[y][x] = '*';
                rc[y][x] = vehicles[i].pixels[j]->r;
                gc[y][x] = vehicles[i].pixels[j]->g;
                bc[y][x] = vehicles[i].pixels[j]->b;
            }
        }

    for (int y = 0; y < side; y++) {
        for (int x = 0; x < side; x++) {
            if (scene[y][x] != ' ')
                printf("\033[38;2;%d;%d;%dm%c\033[0m",
                       rc[y][x], gc[y][x], bc[y][x], scene[y][x]);
            else putchar(' ');
        }
        putchar('\n');
    }
}

void run_animation(struct Road* roads, int num_roads,
                   struct Vehicle* vehicles, int num_vehicles,
                   int side, int step_count) {
    for (int step = 0; step < step_count; step++) {
        // Plan (sets dx/dy on every vehicle), then move, then draw
        apply_whca_star(roads, num_roads);

        printf("\033[2J\033[H");
        for (int i = 0; i < num_vehicles; i++) move_vehicle(&vehicles[i]);
        draw_scene(roads, num_roads, vehicles, num_vehicles, side);

        // ── Per-vehicle HUD ───────────────────────────────────────
        printf("\n Step %2d / %d\n", step + 1, step_count);
        printf(" %-3s %-5s %-17s %-13s %-10s %s\n",
               "V", "Pri", "Position", "Goal", "Move", "Status");
        printf(" ───────────────────────────────────────────────────────────\n");
        for (int i = 0; i < num_vehicles; i++) {
            int at_goal = (vehicles[i].x == vehicles[i].goal_x &&
                           vehicles[i].y == vehicles[i].goal_y);
            printf(" \033[38;2;%d;%d;%dmV%2d\033[0m   %-4d (%2d,%2d)       (%2d,%2d)       (%+d,%+d)     %s\n",
                   vehicles[i].r, vehicles[i].g, vehicles[i].b, i,
                   vehicles[i].priority,
                   vehicles[i].x, vehicles[i].y,
                   vehicles[i].goal_x, vehicles[i].goal_y,
                   vehicles[i].dx, vehicles[i].dy,
                   at_goal ? "\033[32m\xe2\x9c\x93 GOAL\033[0m" : "en route");
        }

        usleep(300000);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   Memory Cleanup
═══════════════════════════════════════════════════════════════════════════ */

void free_memory(struct Road* roads, int num_roads,
                 struct Vehicle* vehicles, int num_vehicles) {
    for (int i = 0; i < num_roads; i++) {
        for (int j = 0; j < roads[i].length * roads[i].width; j++)
            free(roads[i].pixels[j]);
        free(roads[i].pixels);
    }
    for (int i = 0; i < num_vehicles; i++) {
        for (int j = 0; j < vehicles[i].num_pixels; j++)
            free(vehicles[i].pixels[j]);
        free(vehicles[i].pixels);
    }
    free_spacetime_grid();
}

/* ═══════════════════════════════════════════════════════════════════════════
   Random Vehicle Generation
═══════════════════════════════════════════════════════════════════════════ */

void generate_random_vehicles(struct Road* roads, int num_roads,
                              struct Vehicle* vehicles, int num_vehicles) {
    srand(time(NULL));

    int colors[][3] = {
        {255, 0, 0},    // Red
        {0, 255, 0},    // Lime
        {0, 0, 255},    // Blue
        {255, 0, 255},  // Magenta
        {0, 255, 255},  // Cyan
        {255, 255, 0},  // Yellow
        {255, 128, 0},  // Orange
        {0, 128, 0},    // Green
        {128, 0, 128},  // Purple
        {255, 0, 127},  // Rose
        {128, 128, 128},// Grey
        {192, 192, 192},// Silver
        {255, 102, 102},// Coral
        {255, 178, 102},// Light Orange
        {178, 255, 102},// Lime Green
        {102, 255, 178},// Mint
        {102, 255, 255},// Ice Blue
        {102, 178, 255},// Sky Blue
        {178, 102, 255},// Lavender
        {255, 102, 255},// Pink
        {255, 102, 178} // Deep Pink
    };
    int num_colors = sizeof(colors) / sizeof(colors[0]);

    for (int v_idx = 0; v_idx < num_vehicles; v_idx++) {
        int placed = 0;
        for (int retry = 0; retry < 2000; retry++) {
            int r_idx = rand() % num_roads;
            struct Road road = roads[r_idx];

            int is_horizontal = (road.x2 - road.x1) > (road.y2 - road.y1);
            int road_width = is_horizontal ? (road.y2 - road.y1 + 1) : (road.x2 - road.x1 + 1);
            int road_length = is_horizontal ? (road.x2 - road.x1 + 1) : (road.y2 - road.y1 + 1);

            int max_v_width = road_width / 2;
            if (max_v_width < 1) max_v_width = 1;
            int v_width = rand() % max_v_width + 1;

            int v_length = rand() % 3 + 2; // 2 to 4
            if (v_length >= road_length) {
                v_length = road_length - 1;
                if (v_length < 1) v_length = 1;
            }

            int dx = 0, dy = 0;
            int x_start = 0, y_start = 0;
            int x_goal = 0, y_goal = 0;
            int speed = rand() % 3 + 1;

            if (is_horizontal) {
                int east = rand() % 2;
                if (east) {
                    dx = speed;
                    dy = 0;
                    y_start = road.y2 - v_width + 1;
                    y_goal = y_start;

                    int x_range = road.x2 - road.x1 + 1 - v_length + 1;
                    if (x_range > 1) {
                        int x_a = road.x1 + (rand() % x_range);
                        int x_b = road.x1 + (rand() % x_range);
                        if (x_a == x_b) {
                            if (x_a > road.x1) x_a--;
                            else x_b++;
                        }
                        if (x_a < x_b) {
                            x_start = x_a;
                            x_goal = x_b;
                        } else {
                            x_start = x_b;
                            x_goal = x_a;
                        }
                    } else {
                        x_start = road.x1;
                        x_goal = road.x1;
                    }
                } else {
                    dx = -speed;
                    dy = 0;
                    y_start = road.y1;
                    y_goal = y_start;

                    int x_range = road.x2 - road.x1 + 1 - v_length + 1;
                    if (x_range > 1) {
                        int x_a = road.x1 + (rand() % x_range);
                        int x_b = road.x1 + (rand() % x_range);
                        if (x_a == x_b) {
                            if (x_a > road.x1) x_a--;
                            else x_b++;
                        }
                        if (x_a > x_b) {
                            x_start = x_a;
                            x_goal = x_b;
                        } else {
                            x_start = x_b;
                            x_goal = x_a;
                        }
                    } else {
                        x_start = road.x1;
                        x_goal = road.x1;
                    }
                }
            } else {
                int south = rand() % 2;
                if (south) {
                    dx = 0;
                    dy = speed;
                    x_start = road.x1;
                    x_goal = x_start;

                    int y_range = road.y2 - road.y1 + 1 - v_length + 1;
                    if (y_range > 1) {
                        int y_a = road.y1 + (rand() % y_range);
                        int y_b = road.y1 + (rand() % y_range);
                        if (y_a == y_b) {
                            if (y_a > road.y1) y_a--;
                            else y_b++;
                        }
                        if (y_a < y_b) {
                            y_start = y_a;
                            y_goal = y_b;
                        } else {
                            y_start = y_b;
                            y_goal = y_a;
                        }
                    } else {
                        y_start = road.y1;
                        y_goal = road.y1;
                    }
                } else {
                    dx = 0;
                    dy = -speed;
                    x_start = road.x2 - v_width + 1;
                    x_goal = x_start;

                    int y_range = road.y2 - road.y1 + 1 - v_length + 1;
                    if (y_range > 1) {
                        int y_a = road.y1 + (rand() % y_range);
                        int y_b = road.y1 + (rand() % y_range);
                        if (y_a == y_b) {
                            if (y_a > road.y1) y_a--;
                            else y_b++;
                        }
                        if (y_a > y_b) {
                            y_start = y_a;
                            y_goal = y_b;
                        } else {
                            y_start = y_b;
                            y_goal = y_a;
                        }
                    } else {
                        y_start = road.y1;
                        y_goal = road.y1;
                    }
                }
            }

            struct Vehicle temp_v;
            temp_v.x = x_start;
            temp_v.y = y_start;
            if (is_horizontal) {
                temp_v.length = v_length;
                temp_v.width = v_width;
            } else {
                temp_v.length = v_width;
                temp_v.width = v_length;
            }

            if (!is_vehicle_overlapping_others(temp_v, vehicles, v_idx, -1)) {
                if (is_vehicle_on_any_road(temp_v, roads, num_roads)) {
                    int* col = colors[v_idx % num_colors];
                    int priority = v_idx + 1;

                    make_vehicle(&vehicles[v_idx], x_start, y_start, col[0], col[1], col[2],
                                 dx, dy, temp_v.length, temp_v.width, priority, x_goal, y_goal);
                    placed = 1;
                    break;
                }
            }
        }

        if (!placed) {
            for (int r_idx = 0; r_idx < num_roads; r_idx++) {
                struct Road road = roads[r_idx];
                int is_horizontal = (road.x2 - road.x1) > (road.y2 - road.y1);
                int y_pos = is_horizontal ? (road.y2) : (road.y1);
                int x_pos = is_horizontal ? (road.x1) : (road.x2);

                struct Vehicle temp_v;
                temp_v.x = x_pos;
                temp_v.y = y_pos;
                temp_v.length = 1;
                temp_v.width = 1;

                if (!is_vehicle_overlapping_others(temp_v, vehicles, v_idx, -1) &&
                    is_vehicle_on_any_road(temp_v, roads, num_roads)) {
                    int* col = colors[v_idx % num_colors];
                    int priority = v_idx + 1;
                    make_vehicle(&vehicles[v_idx], x_pos, y_pos, col[0], col[1], col[2],
                                 0, 0, 1, 1, priority, x_pos, y_pos);
                    placed = 1;
                    break;
                }
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   Terminal Auto-launch
═══════════════════════════════════════════════════════════════════════════ */

void handle_terminal_relaunch(int argc, char *argv[]) {
    int is_child = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--child") == 0) {
            is_child = 1;
            break;
        }
    }
    if (is_child) return;

    char args_str[512] = "";
    for (int i = 1; i < argc; i++) {
        strncat(args_str, " ", sizeof(args_str) - strlen(args_str) - 1);
        strncat(args_str, argv[i], sizeof(args_str) - strlen(args_str) - 1);
    }
    strncat(args_str, " --child", sizeof(args_str) - strlen(args_str) - 1);

    const char *env_term = getenv("TERMINAL");
    char cmd[1024];
    if (env_term && *env_term) {
        snprintf(cmd, sizeof(cmd), "command -v %s >/dev/null 2>&1", env_term);
        if (system(cmd) == 0) {
            snprintf(cmd, sizeof(cmd), "%s -e %s%s &", env_term, argv[0], args_str);
            if (system(cmd) != -1) exit(0);
        }
    }
    const char *candidates[] = {
        "x-terminal-emulator", "xterm", "gnome-terminal",
        "konsole", "alacritty", "kitty", "urxvt", "st", NULL
    };
    for (int i = 0; candidates[i]; i++) {
        snprintf(cmd, sizeof(cmd), "command -v %s >/dev/null 2>&1", candidates[i]);
        if (system(cmd) == 0) {
            if (strcmp(candidates[i], "gnome-terminal") == 0)
                snprintf(cmd, sizeof(cmd), "%s -- %s%s &", candidates[i], argv[0], args_str);
            else
                snprintf(cmd, sizeof(cmd), "%s -e %s%s &", candidates[i], argv[0], args_str);
            if (system(cmd) != -1) exit(0);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   Entry Point
═══════════════════════════════════════════════════════════════════════════ */

int main(int argc, char *argv[]) {
    handle_terminal_relaunch(argc, argv);

    int roadNum   = 6,
        vehiNum   = 13,
        canvaSide = 30,
        stepCount = 30,
        maxSteps  = 8;  // WHCA* lookahead window (re-planned every tick)

    int use_random = 0,
        custom_vehi_num = 30; // placeholder threshold

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--random") == 0 || strcmp(argv[i], "random") == 0) {
            use_random = 1;
            // if a number succeedes --random, use that as the vehicle count
            if (i + 1 < argc) {
                if (argv[i+1][0] != '-' && strcmp(argv[i+1], "--child") != 0) {
                    custom_vehi_num = atoi(argv[i+1]);
                    if (custom_vehi_num <= 0) {
                        custom_vehi_num = vehiNum;
                    } else if (custom_vehi_num > 50) {
                        custom_vehi_num = 50;
                    }
                }
            }
        }
    }

    if (use_random) {
        vehiNum = custom_vehi_num;
    }

    init_spacetime_grid(canvaSide, maxSteps);

    struct Road    roads[roadNum];
    struct Vehicle vehicles[vehiNum];

    // Road layout
    make_road(&roads[0],  0,  5, 29,  8); 
    make_road(&roads[1], 10,  0, 15, 29); 
    make_road(&roads[2], 10, 12, 29, 15); 
    make_road(&roads[3], 20,  5, 23, 29);
    make_road(&roads[4],  0, 27, 29, 28);
    make_road(&roads[5],  0, 20, 23, 21);

    if (use_random) {
        // how many until lock?
        generate_random_vehicles(roads, roadNum, vehicles, vehiNum);
    } else {
        make_vehicle(&vehicles[ 0],  4,  8, 255,   0,   0,  2,  0,  3,  1,  1, 20,  8); // red
        make_vehicle(&vehicles[ 1], 10,  0,   0, 255,   0,  0,  1,  2,  4,  2, 10, 25); // lime
        make_vehicle(&vehicles[ 2], 25,  5,   0,   0, 255, -3,  0,  5,  1,  3,  4,  5); // blue
        make_vehicle(&vehicles[ 3], 23, 26, 255,   0, 255,  0, -2,  1,  4,  2, 23,  8); // magenta
        make_vehicle(&vehicles[ 4], 14, 22,   0, 255, 255,  0,  3,  1,  2,  3, 14,  0); // cyan
        make_vehicle(&vehicles[ 5], 25, 12, 255, 255,   0, -3,  0,  3,  2,  1, 17, 12); // yellow
        make_vehicle(&vehicles[ 6],  0,  8, 255, 128,   0,  3,  0,  3,  1,  2, 24,  8); // orange
        make_vehicle(&vehicles[ 7], 20,  5,   0, 128,   0, -1,  0,  2,  2,  3,  1,  5); // green
        make_vehicle(&vehicles[ 8], 15, 20, 128,   0, 128,  0, -2,  1,  2,  1, 13,  2); // purple
        make_vehicle(&vehicles[ 9], 15, 25, 255,   0, 127,  0, -4,  1,  4,  4, 15,  1); // rose
        make_vehicle(&vehicles[10], 27, 27, 128, 128, 128, -1,  0,  2,  1,  2,  0, 27); // grey
        make_vehicle(&vehicles[11],  0, 21, 192, 192, 192,  1,  0,  3,  1,  4, 17, 21); // silver
        make_vehicle(&vehicles[12], 14, 24,   0,   0,   0,  0,  4,  1,  2,  3,  0,  5); // black
    }
    // Resize window: extra rows for HUD, extra cols for status text
    resize_terminal(canvaSide + vehiNum + 8, canvaSide + 40);

    g_vehicles     = vehicles;
    g_num_vehicles = vehiNum;

    draw_scene(roads, roadNum, vehicles, vehiNum, canvaSide);
    run_animation(roads, roadNum, vehicles, vehiNum, canvaSide, stepCount);

    printf("\nAnimation complete. Press Enter to close...");
    getchar();

    free_memory(roads, roadNum, vehicles, vehiNum);
    return 0;
}