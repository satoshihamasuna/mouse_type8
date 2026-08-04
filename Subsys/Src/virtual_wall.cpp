#include "Subsys/Inc/virtual_wall_class.h"

namespace {
static const t_direction dirs[4] = {North, East, South, West};

enum : uint8_t {
	BRANCH_PROTECTED = 0x01,
	BRANCH_INCOMPLETE = 0x02,
	BRANCH_BRIDGE_TO_PARENT = 0x04,
	BRANCH_DETACHED = 0x08,
};

// This workspace is static so the iterative graph walk does not consume the
// motion task's stack.  virtual_wall_class is used synchronously from search.
struct t_branch_workspace {
	uint16_t discover[MAZE_SIZE];
	uint16_t low[MAZE_SIZE];
	int16_t parent[MAZE_SIZE];
	uint16_t order[MAZE_SIZE];
	uint16_t stack[MAZE_SIZE];
	uint8_t next_dir[MAZE_SIZE];
	uint8_t flags[MAZE_SIZE];
};

static t_branch_workspace branch_workspace;

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

static uint16_t cell_index(int x,int y)
{
	return (uint16_t)(y * MAZE_SIZE_X + x);
}

static void cell_position(uint16_t index,int *x,int *y)
{
	*x = index % MAZE_SIZE_X;
	*y = index / MAZE_SIZE_X;
}

static t_direction direction_between(int x,int y,int nx,int ny)
{
	if(nx == x && ny == y + 1) return North;
	if(nx == x + 1 && ny == y) return East;
	if(nx == x && ny == y - 1) return South;
	if(nx == x - 1 && ny == y) return West;
	return Dir_None;
}

static t_bool graph_edge_is_open(t_wall_state state,t_virtual_branch_mode mode)
{
	return (state == NOWALL ||
		(mode == VIRTUAL_BRANCH_UNKNOWN_OPEN && state == UNKNOWN)) ? True : False;
}

static t_bool cell_blocks_branch_closure(const wall_class *wall,int x,int y,
		t_virtual_branch_mode mode)
{
	for(int i = 0; i < 4; i++) {
		t_wall_state state = real_state(wall, x, y, dirs[i]);
		if(state == VWALL ||
			(mode == VIRTUAL_BRANCH_OBSERVED_ONLY && state == UNKNOWN)) return True;
	}
	return False;
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

t_bool virtual_wall_class::add_explored_branch_walls(const t_virtual_wall_context& c)
{
	// Treat cells as vertices and physical NOWALL edges as graph edges.  In the
	// prune mode UNKNOWN edges are also treated as open, so a bridge remains a
	// single entrance even under the optimistic connectivity assumption.  A
	// bridge whose far side contains no protected cell is then safe to prune.
	// Existing virtual walls are deliberately ignored while building the graph
	// to avoid self-reinference.
	t_branch_workspace& ws = branch_workspace;
	for(uint16_t i = 0; i < MAZE_SIZE; i++) {
		ws.discover[i] = 0;
		ws.parent[i] = -1;
		ws.flags[i] = 0;
	}

	uint16_t discovered = 0;
	for(int root_x = 0; root_x < MAZE_SIZE_X; root_x++) {
		for(int root_y = 0; root_y < MAZE_SIZE_Y; root_y++) {
			if(is_protected(root_x, root_y, c) == False) continue;
			uint16_t root = cell_index(root_x, root_y);
			if(ws.discover[root] != 0) continue;

			uint16_t stack_size = 0;
			ws.discover[root] = ws.low[root] = ++discovered;
			ws.order[discovered - 1] = root;
			ws.next_dir[root] = 0;
			ws.flags[root] = BRANCH_PROTECTED;
			if(cell_blocks_branch_closure(wall_property, root_x, root_y, c.branch_mode) == True)
				ws.flags[root] |= BRANCH_INCOMPLETE;
			ws.stack[stack_size++] = root;

			while(stack_size != 0) {
				uint16_t v = ws.stack[stack_size - 1];
				int x, y;
				cell_position(v, &x, &y);
				if(ws.next_dir[v] < 4) {
					t_direction dir = dirs[ws.next_dir[v]++];
					int nx, ny;
					if(graph_edge_is_open(real_state(wall_property, x, y, dir), c.branch_mode) == False ||
						neighbor(x, y, dir, &nx, &ny) == False) continue;
					uint16_t next = cell_index(nx, ny);
					if(ws.discover[next] == 0) {
						ws.parent[next] = (int16_t)v;
						ws.discover[next] = ws.low[next] = ++discovered;
						ws.order[discovered - 1] = next;
						ws.next_dir[next] = 0;
						ws.flags[next] = is_protected(nx, ny, c) == True ? BRANCH_PROTECTED : 0;
						if(cell_blocks_branch_closure(wall_property, nx, ny, c.branch_mode) == True)
							ws.flags[next] |= BRANCH_INCOMPLETE;
						ws.stack[stack_size++] = next;
					}
					else if(ws.parent[v] != (int16_t)next && ws.discover[next] < ws.low[v]) {
						ws.low[v] = ws.discover[next];
					}
				}
				else {
					--stack_size;
					int16_t parent = ws.parent[v];
					if(parent >= 0) {
						if(ws.low[v] > ws.discover[(uint16_t)parent])
							ws.flags[v] |= BRANCH_BRIDGE_TO_PARENT;
						if(ws.low[v] < ws.low[(uint16_t)parent])
							ws.low[(uint16_t)parent] = ws.low[v];
						ws.flags[(uint16_t)parent] |=
							ws.flags[v] & (BRANCH_PROTECTED | BRANCH_INCOMPLETE);
					}
				}
			}
		}
	}

	t_bool changed = False;
	// Discovery order is parent-before-child.  Once the outermost entrance is
	// closed, suppress redundant virtual walls deeper inside that same branch.
	for(uint16_t i = 0; i < discovered; i++) {
		uint16_t v = ws.order[i];
		int16_t parent = ws.parent[v];
		if(parent < 0) continue;
		if((ws.flags[(uint16_t)parent] & BRANCH_DETACHED) != 0) {
			ws.flags[v] |= BRANCH_DETACHED;
			continue;
		}
		if((ws.flags[v] & BRANCH_BRIDGE_TO_PARENT) == 0 ||
			(ws.flags[v] & (BRANCH_PROTECTED | BRANCH_INCOMPLETE)) != 0) continue;

		int x, y, px, py;
		cell_position(v, &x, &y);
		cell_position((uint16_t)parent, &px, &py);
		t_direction dir = direction_between(px, py, x, y);
		if(dir == Dir_None) continue;
		if(wall_property->get_virtual_wall(px, py, dir) == True) {
			ws.flags[v] |= BRANCH_DETACHED;
		}
		else if(set_wall(px, py, dir, c) == True) {
			ws.flags[v] |= BRANCH_DETACHED;
			changed = True;
		}
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
	add_explored_branch_walls(context);
}
