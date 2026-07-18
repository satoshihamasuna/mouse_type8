#include "virtual_wall_class.h"

namespace {
static const t_direction dirs[4] = {North, East, South, West};

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
}

t_bool virtual_wall_class::is_protected(int x,int y,const t_virtual_wall_context& c) const
{
	return ((x == c.maze_start.x && y == c.maze_start.y) ||
		(x == c.mouse.x && y == c.mouse.y) ||
		(x >= c.maze_goal.x && x < c.maze_goal.x + c.maze_goal_size &&
		 y >= c.maze_goal.y && y < c.maze_goal.y + c.maze_goal_size)) ? True : False;
}

void virtual_wall_class::clear_cell_walls(int x,int y)
{
	if(x < 0 || x >= MAZE_SIZE_X || y < 0 || y >= MAZE_SIZE_Y) return;
	for(int i = 0; i < 4; i++) {
		wall_property->clear_virtual_wall((uint16_t)x, (uint16_t)y, dirs[i]);
	}
}

void virtual_wall_class::clear_protected_walls(const t_virtual_wall_context& c)
{
	clear_cell_walls(c.maze_start.x, c.maze_start.y);
	for(int x = c.maze_goal.x; x < c.maze_goal.x + c.maze_goal_size; x++) {
		for(int y = c.maze_goal.y; y < c.maze_goal.y + c.maze_goal_size; y++) {
			clear_cell_walls(x, y);
		}
	}
}

t_bool virtual_wall_class::edge_touches_protected(int x,int y,t_direction dir,const t_virtual_wall_context& c) const
{
	int nx, ny;
	if(is_protected(x, y, c) == True) return True;
	return neighbor(x, y, dir, &nx, &ny) == True &&
		is_protected(nx, ny, c) == True ? True : False;
}

t_bool virtual_wall_class::set_wall(int x,int y,t_direction dir,const t_virtual_wall_context& c)
{
	if(edge_touches_protected(x, y, dir, c) == True) return False;
	return wall_property->set_virtual_wall((uint16_t)x, (uint16_t)y, dir);
}

t_bool virtual_wall_class::add_pillar_walls(const t_virtual_wall_context& c)
{
	t_bool changed = False;
	for(int px = 1; px < MAZE_SIZE_X; px++) for(int py = 1; py < MAZE_SIZE_Y; py++) {
		// Four wall segments connected to the pillar at (px, py).
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

void virtual_wall_class::update(const t_virtual_wall_context& context)
{
	// Keep inferred walls between moves. The maze's intrinsic start/goal edges
	// must always remain accessible, so clear those persistent exceptions. The mouse exception
	// prevents new inference in this update, but must not erase an existing wall
	// that closes an adjacent dead end; clearing it makes full-search maps
	// alternate as the mouse moves between neighboring cells.
	clear_protected_walls(context);
	add_pillar_walls(context);
	add_dead_end_walls(context);
}
