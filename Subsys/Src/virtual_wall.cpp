#include "virtual_wall_class.h"

namespace {
struct cell_t { int16_t x; int16_t y; };
struct bridge_t {
	int16_t parent;
	int16_t child;
	t_direction direction;
	int16_t subtree_begin;
	int16_t subtree_size;
	int16_t component_begin;
	int16_t component_end;
};

static int16_t discovery[MAZE_SIZE];
static int16_t low_link[MAZE_SIZE];
static int16_t parent_node[MAZE_SIZE];
static int16_t subtree_size[MAZE_SIZE];
static uint8_t next_direction[MAZE_SIZE];
static t_direction parent_direction[MAZE_SIZE];
static int16_t dfs_stack[MAZE_SIZE];
static int16_t preorder[MAZE_SIZE];
static bridge_t bridges[MAZE_SIZE];
static const t_direction dirs[4] = {North, East, South, West};

static int16_t cell_id(int x,int y) { return (int16_t)(y * MAZE_SIZE_X + x); }
static int cell_x(int16_t id) { return id % MAZE_SIZE_X; }
static int cell_y(int16_t id) { return id / MAZE_SIZE_X; }

static t_bool neighbor(int x,int y,t_direction dir,int *nx,int *ny)
{
	*nx = x; *ny = y;
	switch(dir) {
		case North: ++*ny; break; case East: ++*nx; break;
		case South: --*ny; break; case West: --*nx; break;
		default: return False;
	}
	return (*nx >= 0 && *nx < MAZE_SIZE_X && *ny >= 0 && *ny < MAZE_SIZE_Y) ? True : False;
}

static t_wall_state real_state(const wall_class *wall,int x,int y,t_direction dir)
{
	switch(dir) {
		case North: return (t_wall_state)wall->wall[x][y].north;
		case East: return (t_wall_state)wall->wall[x][y].east;
		case South: return (t_wall_state)wall->wall[x][y].south;
		case West: return (t_wall_state)wall->wall[x][y].west;
		default: return WALL;
	}
}

static t_bool graph_open(const wall_class *wall,int x,int y,t_direction dir,t_bool optimistic)
{
	int nx, ny;
	if(neighbor(x, y, dir, &nx, &ny) != True || wall->get_virtual_wall(x, y, dir) == True) return False;
	t_wall_state state = real_state(wall, x, y, dir);
	return optimistic == True ? ((state != WALL && state != VWALL) ? True : False)
			: (state == NOWALL ? True : False);
}

static int find_bridges(const wall_class *wall,t_bool optimistic)
{
	for(int i = 0; i < MAZE_SIZE; i++) {
		discovery[i] = -1;
		low_link[i] = -1;
		parent_node[i] = -1;
		subtree_size[i] = 0;
		next_direction[i] = 0;
	}

	int16_t timer = 0;
	int bridge_count = 0;
	for(int16_t root = 0; root < MAZE_SIZE; root++) {
		if(discovery[root] >= 0) continue;
		const int bridge_begin = bridge_count;
		const int16_t component_begin = timer;
		discovery[root] = low_link[root] = timer;
		preorder[timer++] = root;
		subtree_size[root] = 1;
		int stack_size = 0;
		dfs_stack[stack_size++] = root;

		while(stack_size > 0) {
			const int16_t node = dfs_stack[stack_size - 1];
			if(next_direction[node] < 4) {
				const t_direction dir = dirs[next_direction[node]++];
				const int x = cell_x(node), y = cell_y(node);
				if(graph_open(wall, x, y, dir, optimistic) != True) continue;
				int nx, ny; neighbor(x, y, dir, &nx, &ny);
				const int16_t adjacent = cell_id(nx, ny);
				if(discovery[adjacent] < 0) {
					parent_node[adjacent] = node;
					parent_direction[adjacent] = dir;
					discovery[adjacent] = low_link[adjacent] = timer;
					preorder[timer++] = adjacent;
					subtree_size[adjacent] = 1;
					dfs_stack[stack_size++] = adjacent;
				} else if(adjacent != parent_node[node] && discovery[adjacent] < low_link[node]) {
					low_link[node] = discovery[adjacent];
				}
				continue;
			}

			stack_size--;
			const int16_t parent = parent_node[node];
			if(parent < 0) continue;
			if(low_link[node] < low_link[parent]) low_link[parent] = low_link[node];
			subtree_size[parent] += subtree_size[node];
			if(low_link[node] > discovery[parent] && bridge_count < MAZE_SIZE) {
				bridges[bridge_count++] = {parent, node, parent_direction[node], discovery[node],
					subtree_size[node], component_begin, 0};
			}
		}

		for(int i = bridge_begin; i < bridge_count; i++) bridges[i].component_end = timer;
	}
	return bridge_count;
}

static t_bool index_in_subtree(int index,const bridge_t& bridge)
{
	return (index >= bridge.subtree_begin &&
		index < bridge.subtree_begin + bridge.subtree_size) ? True : False;
}

static t_direction opposite(t_direction dir)
{
	return (t_direction)(((int)dir + 4) % 8);
}

static cell_t cell_from_id(int16_t id)
{
	return {(int16_t)cell_x(id), (int16_t)cell_y(id)};
}

static t_bool is_in_component(int index,const bridge_t& bridge)
{
	return (index >= bridge.component_begin && index < bridge.component_end) ? True : False;
}

static t_bool is_on_side(int index,const bridge_t& bridge,t_bool child_side)
{
	if(is_in_component(index, bridge) != True) return False;
	const t_bool in_subtree = index_in_subtree(index, bridge);
	return child_side == True ? in_subtree : (in_subtree == True ? False : True);
	}
}

t_bool virtual_wall_class::is_protected(int x,int y,const t_virtual_wall_context& c) const
{
	return ((x == c.start.x && y == c.start.y) || (x == c.mouse.x && y == c.mouse.y) ||
		(x >= c.goal.x && x < c.goal.x + c.goal_size && y >= c.goal.y && y < c.goal.y + c.goal_size)) ? True : False;
}

t_bool virtual_wall_class::edge_touches_goal(int x,int y,t_direction dir,const t_virtual_wall_context& c) const
{
	auto goal = [&c](int cx,int cy) {
		return cx >= c.goal.x && cx < c.goal.x + c.goal_size && cy >= c.goal.y && cy < c.goal.y + c.goal_size;
	};
	int nx, ny;
	if(goal(x, y)) return True;
	return neighbor(x, y, dir, &nx, &ny) == True && goal(nx, ny) ? True : False;
}

t_bool virtual_wall_class::set_wall(int x,int y,t_direction dir,const t_virtual_wall_context& c)
{
	if(edge_touches_goal(x, y, dir, c) == True) return False;
	return wall_property->set_virtual_wall((uint16_t)x, (uint16_t)y, dir);
}

t_bool virtual_wall_class::add_pillar_walls(const t_virtual_wall_context& c)
{
	t_bool changed = False;
	for(int px = 1; px < MAZE_SIZE_X; px++) for(int py = 1; py < MAZE_SIZE_Y; py++) {
		const int xs[4] = {px, px, px - 1, px - 1};
		const int ys[4] = {py, py, py - 1, py - 1};
		const t_direction ds[4] = {West, South, East, North};
		int open = 0, unknown = -1;
		for(int i = 0; i < 4; i++) {
			t_wall_state state = real_state(wall_property, xs[i], ys[i], ds[i]);
			if(state == NOWALL) open++; else if(state == UNKNOWN) unknown = i;
		}
		if(open == 3 && unknown >= 0 && set_wall(xs[unknown], ys[unknown], ds[unknown], c) == True) changed = True;
	}
	return changed;
}

t_bool virtual_wall_class::add_dead_end_walls(const t_virtual_wall_context& c)
{
	t_bool changed = False;
	for(int x = 0; x < MAZE_SIZE_X; x++) for(int y = 0; y < MAZE_SIZE_Y; y++) {
		if(is_protected(x, y, c) == True) continue;
		int blocked = 0, remaining = -1;
		for(int i = 0; i < 4; i++) {
			t_wall_state state = real_state(wall_property, x, y, dirs[i]);
			if(state == WALL || state == VWALL || wall_property->get_virtual_wall(x, y, dirs[i]) == True) blocked++;
			else remaining = i;
		}
		if(blocked == 3 && remaining >= 0 && set_wall(x, y, dirs[remaining], c) == True) changed = True;
	}
	return changed;
}

t_bool virtual_wall_class::add_bridge_walls(const t_virtual_wall_context& c,t_bool optimistic)
{
	const int bridge_count = find_bridges(wall_property, optimistic);
	for(int i = 0; i < bridge_count; i++) {
		const bridge_t& bridge = bridges[i];
		const cell_t parent = cell_from_id(bridge.parent);
		const cell_t child = cell_from_id(bridge.child);

		// Pocket pruning may only replace a physically observed passage.
		if(optimistic == True &&
		   real_state(wall_property, parent.x, parent.y, bridge.direction) != NOWALL) continue;

		for(int side = 0; side < 2; side++) {
			const t_bool child_side = side == 0 ? True : False;
			t_bool protected_cell = False;
			t_bool has_unknown = False;
			t_bool fully_observed = True;
			for(int order = bridge.component_begin; order < bridge.component_end; order++) {
				if(is_on_side(order, bridge, child_side) != True) continue;
				const cell_t cell = cell_from_id(preorder[order]);
				if(is_protected(cell.x, cell.y, c) == True) protected_cell = True;
				if(wall_property->is_unknown(cell.x, cell.y) == True) {
					has_unknown = True;
					fully_observed = False;
				}
			}
			const t_bool prunable = optimistic == True ? has_unknown : fully_observed;
			if(protected_cell == True || prunable != True) continue;

			if(child_side == True)
				return set_wall(parent.x, parent.y, bridge.direction, c);
			return set_wall(child.x, child.y, opposite(bridge.direction), c);
		}
	}
	return False;
}

void virtual_wall_class::update(const t_virtual_wall_context& context)
{
	wall_property->clear_virtual_wall();
	add_pillar_walls(context);
	while(True) {
		t_bool changed = add_dead_end_walls(context);
		if(add_bridge_walls(context, False) == True) changed = True;
		if(add_bridge_walls(context, True) == True) changed = True;
		if(changed != True) break;
	}
}
