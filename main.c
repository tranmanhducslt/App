#include "structs.h"

int main(int argc, char *argv[]){
    // Ensure the application is running in its own terminal window.
    handle_terminal_relaunch(argc, argv);

    struct Road roads[2];
    struct Vehicle vehicles[2];

    make_road(&roads[0], 0, 5, 19, 8);
    make_road(&roads[1], 10, 0, 13, 19);

    make_vehicle(&vehicles[0], 0, 8, 255, 0, 0, 1, 0, 3, 1);
    make_vehicle(&vehicles[1], 10, 0, 0, 255, 0, 0, 1, 2, 4);

    make_map(roads, 2, vehicles, 2);
    check_vehicle_collision(&vehicles[0], &vehicles[1]);
    check_dead_end(&vehicles[0], roads, 2);
    run_animation(roads, 2, vehicles, 2);

    // Keep the terminal open until the user interacts.
    printf("\nAnimation complete. Press Enter to close the window...");
    getchar();

    free_memory(roads, 2, vehicles, 2);
    return 0;
}
