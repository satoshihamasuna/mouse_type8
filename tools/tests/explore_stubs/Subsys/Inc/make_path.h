#ifndef TEST_SHORTEST_MAKE_PATH_H_
#define TEST_SHORTEST_MAKE_PATH_H_
#include "System/Inc/demo_util.h"
#include <cstring>
enum t_run_config_type { RUN_CONFIG_PLAIN, RUN_CONFIG_SUCTION, RUN_CONFIG_VARIABLE_TURN };
struct t_run_config { const char *key; t_run_config_type type; };
static const t_run_config plain = {"plain700", RUN_CONFIG_PLAIN};
static const t_run_config plain500 = {"plain500", RUN_CONFIG_PLAIN};
static const t_run_config suction = {"suction1200_v0", RUN_CONFIG_SUCTION};
static const t_run_config acc = {"acc1600_v1", RUN_CONFIG_VARIABLE_TURN};
static const t_run_config *const run_config_table[] = {&plain500, &plain, &suction, &acc};
static const uint16_t run_config_count = 4;
inline const t_run_config *find_run_config(const char *key)
{
	for(auto config : run_config_table) if(strcmp(key, config->key) == 0) return config;
	return nullptr;
}
extern bool test_no_path;
extern int test_plan_calls;
void test_shortest_plan();
void test_shortest_run(t_run_config_type, bool (*)(), wall_class *, Motion *);
class Dijkstra
{
	wall_class *wall;
public:
	bool (*abort_requested)() = nullptr;
	explicit Dijkstra(wall_class *w) : wall(w) {}
	void run_config_set(const t_run_config *) {}
	t_bool check_DijkstraPath(t_position, t_direction, t_position, uint8_t)
	{ ++test_plan_calls; test_shortest_plan(); return test_no_path ? False : True; }
	void run_Dijkstra(t_position, t_direction, t_position, uint8_t, const t_run_config *, Motion *m)
	{ test_shortest_run(RUN_CONFIG_PLAIN, abort_requested, wall, m); }
	void run_Dijkstra_suction(t_position, t_direction, t_position, uint8_t, const t_run_config *, Motion *m)
	{ test_shortest_run(RUN_CONFIG_SUCTION, abort_requested, wall, m); }
	void run_Dijkstra_suction_acc(t_position, t_direction, t_position, uint8_t, const t_run_config *, Motion *m)
	{ test_shortest_run(RUN_CONFIG_VARIABLE_TURN, abort_requested, wall, m); }
};
#endif
