struct Pixel{
    // Similar to "Snake", each Pixel is a part of the scene.
    // It has a position (x, y) and a color (r, g, b).
    int x, y;
    int r, g, b;
    // A Pixel is a part of a Road.
    // It can also be "lighted" to show that there is a car on it.
    int light; // 0 for not lighted, 1 for lighted
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
};

void make_road(struct Road* road, int x1, int y1, int x2, int y2);
void make_vehicle(struct Vehicle* vehicle, int x, int y, int r, int g, int b, int dx, int dy, int length, int width);
void make_map(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles);
void move_vehicle(struct Vehicle* vehicle);
void check_vehicle_collision(struct Vehicle* vehicle1, struct Vehicle* vehicle2);
void check_dead_end(struct Vehicle* vehicle, struct Road* roads, int num_roads);
void draw_scene(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles);
void run_animation(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles);
void free_memory(struct Road* roads, int num_roads, struct Vehicle* vehicles, int num_vehicles);
void handle_terminal_relaunch(int argc, char *argv[]);