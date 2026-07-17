#ifndef SUBSYS_INC_VIRTUAL_WALL_CLASS_H_
#define SUBSYS_INC_VIRTUAL_WALL_CLASS_H_

#include "wall_class.h"

typedef struct {
	t_position start;
	t_position mouse;
	t_position goal;
	uint8_t goal_size;
} t_virtual_wall_context;

class virtual_wall_class
{
public:
	explicit virtual_wall_class(wall_class *wall_) : wall_property(wall_) {}
	void update(const t_virtual_wall_context& context);

private:
	wall_class *wall_property;
	t_bool add_pillar_walls(const t_virtual_wall_context& context);
	t_bool add_dead_end_walls(const t_virtual_wall_context& context);
	t_bool add_bridge_walls(const t_virtual_wall_context& context,t_bool optimistic);
	t_bool set_wall(int x,int y,t_direction dir,const t_virtual_wall_context& context);
	t_bool is_protected(int x,int y,const t_virtual_wall_context& context) const;
	t_bool edge_touches_goal(int x,int y,t_direction dir,const t_virtual_wall_context& context) const;
};

#endif
