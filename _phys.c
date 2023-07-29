#include "_phys.h"
#include <stdio.h>

struct vector2 _v2_new(float x, float y) {
	struct vector2 ret = {x, y};
	return ret;
}
struct vector2 _v2_add(struct vector2 v, struct vector2 v2) {
	struct vector2 ret = {v.x + v2.x, v.y + v2.y};
	return ret;
}
struct vector2 _v2_sub(struct vector2 v, struct vector2 v2) {
	struct vector2 ret = {v.x - v2.x, v.y - v2.y};
	return ret;
}
struct vector2 _v2_negate(struct vector2 v) {
	struct vector2 ret = {-v.x, -v.y};
	return ret;
}
struct vector2 _v2_smul(struct vector2 v, float scalar) {
	struct vector2 ret = {v.x * scalar, v.y * scalar};
	return ret;
}
struct vector2 _v2_sdiv(struct vector2 v, float scalar) {
	struct vector2 ret = {v.x / scalar, v.y / scalar};
	return ret;
}
int _v2_equ(struct vector2 v, struct vector2 v2) {
	return v.x == v2.x && v.y == v2.y;
}
float _v2_mag(struct vector2 v) {
	return sqrt(v.x*v.x + v.y*v.y);
}
struct vector2 _v2_cpy(struct vector2 v) {
	struct vector2 ret = {v.x, v.y};
	return ret;
}

struct rigidbody _rb_new(struct vector2 position, float mass, float radius) {
	union col col; col.radius = radius;
	struct rigidbody rb = {_v2_cpy(position), _v2_new(0, 0), _v2_new(0, 0), _v2_new(-1, -1), _col_new(RADIUS, col), mass};
	return rb;
}
struct rigidbody _rb_new_aabb(struct vector2 position, float mass, struct vector2 size) {
	union col col; col.size = _v2_cpy(size);
	struct rigidbody rb = {_v2_cpy(position), _v2_new(0, 0), _v2_new(0, 0), _v2_new(-1, -1), _col_new(AABB, col), mass};
	return rb;
}
int _rb_add_bounds(struct rigidbody *rb, struct vector2 bounds) {
	rb->bounds = _v2_cpy(bounds);
}
int _rb_add_force(struct rigidbody *rb, struct vector2 force) {
	rb->velocity = _v2_add(force, rb->velocity);
}
int _rb_collision(struct rigidbody *rb, struct rigidbody *rb2) {
	rb->position = rb->lastposition;
	rb2->position = rb2->lastposition;
	struct vector2 final[2];
	struct vector2 fsmul = _v2_smul(rb->velocity, rb->mass),
				   ssmul = _v2_smul(rb2->velocity, rb2->mass);
	final[0] = _v2_sdiv(_v2_sub(_v2_add(_v2_smul(rb->velocity,
												 rb->mass),
							 			_v2_smul(rb2->velocity,
												 rb2->mass)),
					 			_v2_smul(_v2_sub(rb->velocity,
										 rb2->velocity),
								rb2->mass)),
						rb->mass + rb2->mass);
	final[1] = _v2_add(_v2_sub(rb->velocity,
							   rb2->velocity),
					   final[0]);
	rb->velocity = final[0];
	rb2->velocity = final[1];
	printf("%f %f\n", final[0].x, final[0].y);
}

int _rb_is_colliding(struct rigidbody rb, struct rigidbody rb2) {
	float x  = rb.position.x,
		  y  = rb.position.y,
		  x2 = rb2.position.x,
		  y2 = rb2.position.y;
	switch(rb.collider.type) {
	case RADIUS:
		switch(rb2.collider.type) {
		case RADIUS:
			return (_v2_mag(_v2_sub(rb.position, rb2.position)) <
					rb.collider.radius + rb2.collider.radius);
		case AABB: {
			float z = rb2.collider.size.x,
				  w = rb2.collider.size.y,
				  r = rb.collider.radius;
			return x - r < x2 + z &&
				   x + r > x2     &&
				   y - r < y2 + w &&
				   y + r > y2; }
		}
	case AABB:
		switch(rb2.collider.type) {
		case RADIUS: {
			float z = rb.collider.size.x,
				  w = rb.collider.size.y,
				  r = rb2.collider.radius;
			return x2 - r < x + z &&
				   x2 + r > x     &&
				   y2 - r < y + z &&
				   y2 + r > y; }
		case AABB: {
			float z = rb.collider.size.x,
				  w = rb.collider.size.y,
				  z2 = rb2.collider.size.x,
				  w2 = rb2.collider.size.y;
			return x < x2 + z2 &&
				   x + z > x2  &&
				   y > y2 + w2 &&
				   y + w < y2; }
		}
	}
}

enum out_of_bounds _rb_handle(struct rigidbody *rb, float time) {
	rb->lastposition = rb->position;
	rb->position = _v2_add(_v2_smul(rb->velocity,
                                    time),
                           rb->position);
	if(rb->bounds.x != -1) switch(rb->collider.type) {
		case RADIUS:
			if(rb->position.x + rb->collider.radius > rb->bounds.x - 1)
				return BOUND_OUT_X_MAX;
			break;
		case AABB:
			if(rb->position.x + rb->collider.size.x > rb->bounds.x - 1)
				return BOUND_OUT_X_MAX;
			break;
		}
	switch(rb->collider.type) {
	case RADIUS:
		if(rb->position.x - rb->collider.radius < 0)
			return BOUND_OUT_X_MIN;
	case AABB:
		if(rb->position.x < 0)
			return BOUND_OUT_X_MIN;
	}
	if(rb->bounds.y != -1) switch(rb->collider.type) {
		case RADIUS:
			if(rb->position.y + rb->collider.radius > rb->bounds.y - 1)
				return BOUND_OUT_Y_MAX;
			break;
		case AABB:
			if(rb->position.x + rb->collider.size.x > rb->bounds.x - 1)
				return BOUND_OUT_X_MAX;
			break;
		}
	switch(rb->collider.type) {
	case RADIUS:
		if(rb->position.y - rb->collider.radius < 0)
			return BOUND_OUT_Y_MIN;
	case AABB:
		if(rb->position.y < 0)
			return BOUND_OUT_Y_MIN;
	}
	return BOUND_IN;
}

struct collider _col_new(enum collider_types type, union col col) {
	struct collider ret;
	switch(type) {
	case RADIUS: ret.type = type;
				 ret.radius = col.radius;
				 break;
	case AABB:   ret.type = type;
				 ret.size = col.size;
				 break;
	}
	return ret;
}

int _rb_default_bounce_bound_handling(struct rigidbody *rb, enum out_of_bounds handled) {
    switch(handled) {
    case BOUND_IN: return 0;
    case BOUND_OUT_X_MAX:
        if(rb->collider.type == RADIUS)
			rb->position.x = rb->bounds.x - rb->collider.radius - 2;
		else if(rb->collider.type == AABB)
			rb->position.x = rb->bounds.x - rb->collider.size.x - 2;
		rb->velocity.x = -(rb->velocity.x);
        return 0;
    case BOUND_OUT_X_MIN:
		if(rb->collider.type == RADIUS) 
    		rb->position.x = rb->collider.radius + 2;
		else if(rb->collider.type == AABB)
			rb->position.x = 2;
		rb->velocity.x = -(rb->velocity.x);
        return 0;
    case BOUND_OUT_Y_MAX:
		if(rb->collider.type == RADIUS)
        	rb->position.y = rb->bounds.y - rb->collider.radius - 2;
		else if(rb->collider.type == AABB)
			rb->position.y = rb->bounds.x - rb->collider.size.y;
		rb->velocity.y = -(rb->velocity.y);
        return 0;
    case BOUND_OUT_Y_MIN:
		if(rb->collider.type == RADIUS)
        	rb->position.y = rb->collider.radius + 2;
		else if(rb->collider.type == AABB)
			rb->position.y = 2;
		rb->velocity.y = -(rb->velocity.y);
        return 0;
    }
	return 1;
}

int _rb_default_stop_bound_handling(struct rigidbody *rb, enum out_of_bounds handled) {
	switch(handled) {
		case BOUND_IN: return 0;
		case BOUND_OUT_X_MAX:
	        if(rb->collider.type == RADIUS)
    	        rb->position.x = rb->bounds.x - rb->collider.radius - 2;
        	else if(rb->collider.type == AABB)
            	rb->position.x = rb->bounds.x - rb->collider.size.x - 2;
	        rb->velocity.x = 0;
    	    return 0;
	    case BOUND_OUT_X_MIN:
    	    if(rb->collider.type == RADIUS)
        	    rb->position.x = rb->collider.radius + 2;
	        else if(rb->collider.type == AABB)
    	        rb->position.x = 1;
        	rb->velocity.x = 0;
	        return 0;
	    case BOUND_OUT_Y_MAX:
	        if(rb->collider.type == RADIUS)
	            rb->position.y = rb->bounds.y - rb->collider.radius - 2;
	        else if(rb->collider.type == AABB)
	            rb->position.y = rb->bounds.x - rb->collider.size.y;
	        rb->velocity.y = 0;
	        return 0;
	    case BOUND_OUT_Y_MIN:
	        if(rb->collider.type == RADIUS)
	            rb->position.y = rb->collider.radius + 2;
	        else if(rb->collider.type == AABB)
	            rb->position.y = 0; 
	        rb->velocity.y = 0;
	        return 0;
	}
	return 1;
}







