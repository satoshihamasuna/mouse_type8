#ifndef SYSTEM_INC_EXPLORE_H_
#define SYSTEM_INC_EXPLORE_H_
class wall_class;
// Latest completed BLE exploration; retained until the next exploration/reset.
wall_class *explore_completed_maze();
int shell_explore(int argc, char **argv);
#endif
