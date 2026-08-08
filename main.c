#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/time.h>
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
    int x, y;
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
    int planned_turn[MAX_LOOKAHEAD]; // Turn decision (1 if turned sideways on this step, 0 otherwise)
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

void make_road(struct Road* road, int x, int y, int length, int width) {
    road->x = x; road->y = y;
    road->length = length; road->width = width;
    road->pixels = malloc(length * width * sizeof(struct Pixel*));
    for (int i = 0; i < length; i++) {
        for (int j = 0; j < width; j++) {
            struct Pixel* p = malloc(sizeof(struct Pixel));
            p->x = x + i; p->y = y + j;
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
    memset(v->planned_turn, 0, sizeof(v->planned_turn));
    build_vehicle_pixels(v);
}

// Turns a vehicle sideways by swapping its length and width and rebuilding its pixel representation.
void turn_vehicle(struct Vehicle* v){
    // Swap length and width
    int temp = v->length;
    v->length = v->width;
    v->width = temp;

    // Free old pixels and rebuild them at the new position/orientation
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

int is_vehicle_ori_by_velo(struct Vehicle v){
    return (v.dx > v.dy && v.length > v.width) || (v.dy > v.dx && v.width > v.length);
}

int is_vehicle_on_road(struct Vehicle v, struct Road road){
    return (v.x >= road.x && v.x + v.length - 1 < road.x + road.length &&
            v.y >= road.y && v.y + v.width - 1 < road.y + road.width);
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
typedef struct { int px, py, pt, mdx, mdy, turn; } CFEntry;

// Flat index into the (x, y, t) state space.
#define SIDX(x, y, t) (((x) * g_canvas_side + (y)) * (g_max_lookahead + 1) + (t))

// ── Collision predicate ───────────────────────────────────────────────────

// Returns 1 iff vehicle vid's bounding box placed at anchor (ax, ay) is fully
// within bounds, free of foreign reservations at timestep t, and entirely on roads.
static int pos_free(int vid, int ax, int ay, int t, int need_turn) {
    struct Vehicle* v = &g_vehicles[vid];
    
    int length = v->length, width = v->width, max = (v->length > v->width) ? v->length : v->width;
    // At a turn, the bounding box is a square of side max(length, width).
    if (need_turn){
        length = max;
        width = max;
    }

    // Check bounds and reservations for each cell of the vehicle
    for (int i = 0; i < length; i++) {
        for (int j = 0; j < width; j++) {
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
    temp_v.length = length;
    temp_v.width = width;
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

// Artificial Potential Field heuristic:
// - attractive term pulls the search toward the goal
// - repulsive term penalizes states near already-reserved cells
// - only checks cells within a small radius for efficiency
static int apf_heuristic(int x, int y, int gx, int gy, int vid) {
    int attraction = abs(x - gx) + abs(y - gy);
    int repulsion = 0;

    const int SEARCH_RADIUS = 6;  // Only check nearby obstacles

    // Only check current and next timestep for efficiency
    for (int t = 0; t <= 1 && t <= g_max_lookahead; t++) {
        // Check only cells within SEARCH_RADIUS of current position
        for (int i = x - SEARCH_RADIUS; i <= x + SEARCH_RADIUS; i++) {
            for (int j = y - SEARCH_RADIUS; j <= y + SEARCH_RADIUS; j++) {
                // Skip out-of-bounds cells
                if (i < 0 || i >= g_canvas_side || j < 0 || j >= g_canvas_side)
                    continue;

                int occ = g_spacetime_grid[i][j][t];
                if (occ == 0 || occ == vid + 1) continue;  // Skip free or own cells

                int dist = abs(x - i) + abs(y - j);
                if (dist < 4) {
                    repulsion += (4 - dist) * 8;
                }
            }
        }
    }

    return attraction + repulsion;
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

    // Heap for the open set of A*; capacity = number of states in the search space
    MinHeap open = { malloc(ns * sizeof(HNode)), 0, ns };

    // Seed the start state with an APF-guided heuristic.
    int h0 = apf_heuristic(sx, sy, gx, gy, vid);
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
            int turn_needed = 0;
            int valid = 1;
            if (!pos_free(vid, nx, ny, nt, 0)) {
                if (pos_free(vid, nx, ny, nt, 1)) {
                    turn_needed = 1;
                } else {
                    valid = 0;
                }
            }
            if (valid) {
                int ng = cur.g + 1, ni = SIDX(nx, ny, nt);
                if (ng < gval[ni]) {
                    gval[ni] = ng;
                    cf[ni] = (CFEntry){ cx, cy, ct, 0, 0, turn_needed };
                    int nh = apf_heuristic(nx, ny, gx, gy, vid);
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
                int turn_needed = 0;
                for (int step = 1; step <= k; step++) {
                    int ix = cx + dx_dir[d] * step;
                    int iy = cy + dy_dir[d] * step;
                    if (!pos_free(vid, ix, iy, nt, 0)) {
                        if (pos_free(vid, ix, iy, nt, 1)) {
                            turn_needed = 1;
                        } else {
                            possible = 0;
                            break;
                        }
                    }
                }
                if (!possible) continue;

                int ng = cur.g + 1, ni = SIDX(nx, ny, nt);
                if (ng < gval[ni]) {
                    gval[ni] = ng;
                    cf[ni] = (CFEntry){ cx, cy, ct, mx, my, turn_needed };
                    int nh = apf_heuristic(nx, ny, gx, gy, vid);
                    mh_push(&open, (HNode){ nx, ny, nt, ng, ng + nh });
                }
            }
        }
    }

    // ── Reconstruct planned moves by walking the came-from chain ──────────
    memset(v->planned_dx, 0, sizeof(v->planned_dx));
    memset(v->planned_dy, 0, sizeof(v->planned_dy));
    memset(v->planned_turn, 0, sizeof(v->planned_turn));

    for (int tx = bx, ty = by, tt = bt; tt > 0;) {
        int idx = SIDX(tx, ty, tt);
        v->planned_dx[tt - 1] = cf[idx].mdx;
        v->planned_dy[tt - 1] = cf[idx].mdy;
        v->planned_turn[tt - 1] = cf[idx].turn;
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

        int cur_len = v->length;
        int cur_wid = v->width;
        if (v->planned_turn[s - 1]) {
            cur_len = v->width;
            cur_wid = v->length;
        }

        int dx = v->planned_dx[s - 1];
        int dy = v->planned_dy[s - 1];
        int steps = abs(dx) + abs(dy);
        if (steps == 0) {
            for (int i = 0; i < cur_len; i++) {
                for (int j = 0; j < cur_wid; j++) {
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
                for (int i = 0; i < cur_len; i++) {
                    for (int j = 0; j < cur_wid; j++) {
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

static double monotonic_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec * 1000.0 + (double)tv.tv_usec / 1000.0;
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
    double total_runtime_ms = 0.0;

    for (int step = 0; step < step_count; step++) {
        double step_start_ms = monotonic_ms();

        double plan_start_ms = monotonic_ms();
        apply_whca_star(roads, num_roads);
        double plan_ms = monotonic_ms() - plan_start_ms;

        printf("\033[2J\033[H");

        double move_start_ms = monotonic_ms();
        for (int i = 0; i < num_vehicles; i++) {
            if (vehicles[i].planned_turn[0]) {
                turn_vehicle(&vehicles[i]);
            }
            move_vehicle(&vehicles[i]);
        }
        double move_ms = monotonic_ms() - move_start_ms;

        double draw_start_ms = monotonic_ms();
        draw_scene(roads, num_roads, vehicles, num_vehicles, side);
        double draw_ms = monotonic_ms() - draw_start_ms;

        double step_ms = monotonic_ms() - step_start_ms;
        total_runtime_ms += step_ms;

        // ── Per-vehicle HUD ───────────────────────────────────────
        printf("\n Step %2d / %d\n", step + 1, step_count);
        printf(" Timing: plan %.2f ms | move %.2f ms | draw %.2f ms | total %.2f ms\n",
               plan_ms, move_ms, draw_ms, step_ms);
        printf(" %-3s %-9s %-9s %-19s %-13s %-10s %s\n",
               "V", "Pri", "Size", "Position", "Goal", "Move", "Status");
        printf(" ───────────────────────────────────────────────────────────────────────────\n");
        for (int i = 0; i < num_vehicles; i++) {
            int at_goal = (vehicles[i].x == vehicles[i].goal_x &&
                           vehicles[i].y == vehicles[i].goal_y);
            printf(" \033[38;2;%d;%d;%dmV%2d\033[0m   %-4d (%2d,%2d)       (%2d,%2d)         (%2d,%2d)       (%+d,%+d)     %s\n",
                   vehicles[i].r, vehicles[i].g, vehicles[i].b, i,
                   vehicles[i].priority,
                   vehicles[i].length, vehicles[i].width,
                   vehicles[i].x, vehicles[i].y,
                   vehicles[i].goal_x, vehicles[i].goal_y,
                   vehicles[i].dx, vehicles[i].dy,
                   at_goal ? "\033[32m✔ GOAL\033[0m" : "en route");
        }

        usleep(300000);
    }

    printf("\nTotal animation runtime: %.2f ms (avg per step: %.2f ms)\n",
           total_runtime_ms, total_runtime_ms / step_count);
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
int* rand_col() {
    int* rgb = malloc(3 * sizeof(int));
    rgb[0] = rand() % 256; // Red
    rgb[1] = rand() % 256; // Green
    rgb[2] = rand() % 256; // Blue
    return rgb;
}

struct Pixel rand_road_point(struct Road* roads, int num_roads){
    // choose random road id
    int rid = rand() % num_roads;
    struct Road goal_road = roads[rid];
    // find its start and end points
    int x_r_start = goal_road.x;
    int x_r_end = goal_road.x + goal_road.length;
    int y_r_start = goal_road.y;
    int y_r_end = goal_road.y + goal_road.width;
    // choose random goal points
    int x_goal = x_r_start + rand() % (x_r_end - x_r_start);
    int y_goal = y_r_start + rand() % (y_r_end - y_r_start);
    return (struct Pixel){.x = x_goal, .y = y_goal};
}

void generate_random_vehicles(struct Road* roads, int num_roads,
                              struct Vehicle* vehicles, int num_vehicles) {
    srand(time(NULL));

    for (int v_idx = 0; v_idx < num_vehicles; v_idx++) {
        int placed = 0;
        for (int retry = 0; retry < 2000; retry++) {
            int r_idx = rand() % num_roads;
            struct Road road = roads[r_idx];

            int is_horizontal = road.length > road.width;
            int road_width = is_horizontal ? road.width : road.length;
            int road_length = is_horizontal ? road.length : road.width;

            int max_v_width = road_width / 2;
            if (max_v_width < 1) max_v_width = 1;
            int v_width = rand() % max_v_width + 1;

            int v_length = rand() % 3 + 2; // 2 to 4
            if (v_length >= road_length) {
                v_length = road_length - 1;
                if (v_length < 1) v_length = 1;
            }

            int dx = 0, dy = 0;
            int speed = rand() % 3 + 1;

            struct Pixel p_start = rand_road_point(roads, num_roads);
            struct Pixel p_goal = rand_road_point(roads, num_roads);

            struct Vehicle temp_v;
            temp_v.x = p_start.x;
            temp_v.y = p_start.y;
            if (is_horizontal) {
                temp_v.length = v_length;
                temp_v.width = v_width;
            } else {
                temp_v.length = v_width;
                temp_v.width = v_length;
            }

            if (!is_vehicle_overlapping_others(temp_v, vehicles, v_idx, -1)) {
                if (is_vehicle_on_any_road(temp_v, roads, num_roads)) {
                    int* col = rand_col();
                    int priority = v_idx + 1;

                    make_vehicle(&vehicles[v_idx], p_start.x, p_start.y, col[0], col[1], col[2],
                                 dx, dy, temp_v.length, temp_v.width, priority, p_goal.x, p_goal.y);
                    placed = 1;
                    break;
                }
            }
        }

        if (!placed) {
            for (int r_idx = 0; r_idx < num_roads; r_idx++) {
                struct Road road = roads[r_idx];
                int is_horizontal = (road.x + road.length) > (road.y + road.width);
                int y_pos = is_horizontal ? (road.y + road.width) : (road.y);
                int x_pos = is_horizontal ? (road.x) : (road.x + road.length);

                struct Vehicle temp_v;
                temp_v.x = x_pos;
                temp_v.y = y_pos;
                temp_v.length = 1;
                temp_v.width = 1;

                if (!is_vehicle_overlapping_others(temp_v, vehicles, v_idx, -1) &&
                    is_vehicle_on_any_road(temp_v, roads, num_roads)) {
                    int* col = rand_col();
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
        vehiNum   = 15,
        canvaSide = 30,
        stepCount = 30,
        maxSteps  = 5;  // WHCA* lookahead window (re-planned every tick)

    int use_random = 0;

    /* zeroing values are to debug*/
    for (int i = 1; i < argc; i++) {
        // random vehicles
        if (strcmp(argv[i], "--random") == 0 || strcmp(argv[i], "random") == 0) {
            use_random = 1;
            if (i + 1 < argc) {
                if (argv[i+1][0] != '-' && strcmp(argv[i+1], "--child") != 0) {
                    vehiNum = atoi(argv[i+1]);
                    if (vehiNum < 0) {
                        vehiNum = 15;
                    } 
                }
            }
        }
        // # of animation steps
        if (strcmp(argv[i], "--steps") == 0 || strcmp(argv[i], "steps") == 0) {
            if (i + 1 < argc) {
                if (argv[i+1][0] != '-' && strcmp(argv[i+1], "--child") != 0) {
                    stepCount = atoi(argv[i+1]);
                    if (stepCount < 0) {
                        stepCount = 30;
                    } 
                }
            }
        }
        // # of planned steps in WHCA*
        if (strcmp(argv[i], "--lookahead") == 0 || strcmp(argv[i], "lookahead") == 0) {
            if (i + 1 < argc) {
                if (argv[i+1][0] != '-' && strcmp(argv[i+1], "--child") != 0) {
                    maxSteps = atoi(argv[i+1]);
                    if (maxSteps < 0) {
                        maxSteps = 5;
                    } 
                }
            }
        }
    }

    init_spacetime_grid(canvaSide, maxSteps);

    struct Road    roads[roadNum];
    struct Vehicle vehicles[vehiNum];

    // Road layout
    make_road(&roads[0],  0,  5, 30,  4); 
    make_road(&roads[1], 10,  0,  6, 30); 
    make_road(&roads[2], 10, 12, 20,  4); 
    make_road(&roads[3], 20,  5,  4, 25);
    make_road(&roads[4],  0, 27, 30,  2);
    make_road(&roads[5],  0, 20, 24,  2);

    if (use_random) {
        generate_random_vehicles(roads, roadNum, vehicles, vehiNum);
    } else {
        make_vehicle(&vehicles[ 0],  4,  8, 255,   0,   0,  2,  0,  3,  1,  1, 10, 15); // red
        make_vehicle(&vehicles[ 1], 10,  0,   0, 255,   0,  0,  1,  2,  4,  2, 10, 25); // lime
        make_vehicle(&vehicles[ 2], 25,  5,   0,   0, 255, -3,  0,  5,  1,  3,  4,  5); // blue
        make_vehicle(&vehicles[ 3], 23, 26, 255,   0, 255,  0, -2,  1,  4,  2, 23,  8); // magenta
        make_vehicle(&vehicles[ 4], 14, 22,   0, 255, 255,  0,  3,  1,  2,  3, 14,  0); // cyan
        make_vehicle(&vehicles[ 5], 25, 12, 255, 255,   0, -3,  0,  3,  2,  1, 17, 12); // yellow
        make_vehicle(&vehicles[ 6], 12,  0, 150,  75,   0,  0,  2,  1,  2,  2,  0, 20); // brown
        make_vehicle(&vehicles[ 7],  0,  8, 255, 128,   0,  3,  0,  3,  1,  2, 24,  8); // orange
        make_vehicle(&vehicles[ 8], 20,  5,   0, 128,   0, -1,  0,  2,  2,  3,  1,  5); // green
        make_vehicle(&vehicles[ 9], 29, 14, 152, 251, 203, -1,  0,  2,  1,  3, 10, 27); // mint
        make_vehicle(&vehicles[10], 15, 20, 128,   0, 128,  0, -2,  1,  2,  1, 13,  2); // purple
        make_vehicle(&vehicles[11], 15, 25, 255,   0, 127,  0, -4,  1,  4,  4, 15,  1); // rose
        make_vehicle(&vehicles[12], 27, 27, 128, 128, 128, -1,  0,  2,  1,  2,  0, 27); // grey
        make_vehicle(&vehicles[13],  0, 21, 192, 192, 192,  1,  0,  3,  1,  4, 22, 18); // silver
        make_vehicle(&vehicles[14], 14, 24,   0,   0,   0,  0,  4,  1,  2,  3,  0,  5); // black
    }
    // Resize window: extra rows for HUD, extra cols for status text
    resize_terminal(canvaSide + vehiNum + 10, canvaSide + 50);

    g_vehicles     = vehicles;
    g_num_vehicles = vehiNum;

    draw_scene(roads, roadNum, vehicles, vehiNum, canvaSide);
    run_animation(roads, roadNum, vehicles, vehiNum, canvaSide, stepCount);

    printf("\nAnimation complete. Press Enter to close...");
    getchar();

    free_memory(roads, roadNum, vehicles, vehiNum);
    return 0;
}