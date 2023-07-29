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
	struct rigidbody rb = {_v2_cpy(position), _v2_new(0, 0), _v2_new(-1, -1), _col_new(radius), mass};
	return rb;
}
int _rb_add_bounds(struct rigidbody *rb, struct vector2 bounds) {
	rb->bounds = _v2_cpy(bounds);
}
int _rb_add_force(struct rigidbody *rb, struct vector2 force) {
	rb->velocity = _v2_add(force, rb->velocity);
}
int _rb_collision(struct rigidbody *rb, struct rigidbody *rb2) {
	struct vector2 final[2];
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
}

int _rb_is_colliding(struct rigidbody rb, struct rigidbody rb2) {
	return (_v2_mag(_v2_sub(rb.position, rb2.position)) < rb.collider.radius + rb2.collider.radius);
}

enum out_of_bounds _rb_handle(struct rigidbody *rb, float time) {
	rb->position = _v2_add(_v2_smul(rb->velocity,
                                    time),
                           rb->position);
	if(rb->bounds.x != -1)
		if(rb->position.x + rb->collider.radius > rb->bounds.x - 1) return BOUND_OUT_X_MAX;
	if(rb->position.x - rb->collider.radius < 0)
		return BOUND_OUT_X_MIN;
	if(rb->bounds.y != -1)
		if(rb->position.y + rb->collider.radius > rb->bounds.y - 1) return BOUND_OUT_Y_MAX;
	if(rb->position.y - rb->collider.radius < 0)
		return BOUND_OUT_Y_MIN;
	return BOUND_IN;
}

struct collider _col_new(float radius) {
	struct collider ret = {radius};
	return ret;
}

int _rb_default_bounce_bound_handling(struct rigidbody *rb, enum out_of_bounds handled) {
    switch(handled) {
    case BOUND_IN: return 0;
    case BOUND_OUT_X_MAX:
                   rb->position.x = rb->bounds.x - rb->collider.radius - 1;
                   rb->velocity.x = -(rb->velocity.x);
                   return 0;
    case BOUND_OUT_X_MIN:
                   rb->position.x = rb->collider.radius + 1;
                   rb->velocity.x = -(rb->velocity.x);
                   return 0;
    case BOUND_OUT_Y_MAX:
                   rb->position.y = rb->bounds.y - rb->collider.radius - 1;
                   rb->velocity.y = -(rb->velocity.y);
                   return 0;
    case BOUND_OUT_Y_MIN:
                   rb->position.y = rb->collider.radius + 1;
                   rb->velocity.y = -(rb->velocity.y);
                   return 0;
    }
	return 1;
}

int _rb_default_stop_bound_handling(struct rigidbody *rb, enum out_of_bounds handled) {
	switch(handled) {
		case BOUND_IN: return 0;
		case BOUND_OUT_X_MAX:
					   rb->position.x = rb->bounds.x - rb->collider.radius - 1;
					   rb->velocity.x = 0;
					   return 0;
		case BOUND_OUT_X_MIN:
					   rb->position.x = rb->collider.radius;
					   rb->velocity.x = 0;
					   return 0;
		case BOUND_OUT_Y_MAX:
					   rb->position.y = rb->bounds.y - rb->collider.radius - 1;
					   rb->velocity.y = 0;
					   return 0;
		case BOUND_OUT_Y_MIN:
					   rb->position.y = rb->collider.radius;
					   rb->velocity.y = 0;
					   return 0;
	}
	return 1;
}







