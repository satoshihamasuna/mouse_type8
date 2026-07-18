#ifndef SUBSYS_INC_VIRTUAL_WALL_CLASS_H_
#define SUBSYS_INC_VIRTUAL_WALL_CLASS_H_

#include "wall_class.h"

typedef struct {
	t_position maze_start;
	t_position mouse;
	t_position maze_goal;
	uint8_t maze_goal_size;
} t_virtual_wall_context;

class virtual_wall_class
{
public:
	explicit virtual_wall_class(wall_class *wall_) : wall_property(wall_) {}
	void update(const t_virtual_wall_context& context);

private:
	wall_class *wall_property;
	void clear_protected_walls(const t_virtual_wall_context& context);
	void clear_cell_walls(int x,int y);
	t_bool add_pillar_walls(const t_virtual_wall_context& context);
	t_bool add_dead_end_walls(const t_virtual_wall_context& context);
	t_bool set_wall(int x,int y,t_direction dir,const t_virtual_wall_context& context);
	t_bool is_protected(int x,int y,const t_virtual_wall_context& context) const;
	t_bool edge_touches_protected(int x,int y,t_direction dir,const t_virtual_wall_context& context) const;
};

#endif
