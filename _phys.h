#include <math.h>


struct vector2 {
	float x, y;
};
struct vector2 _v2_new(float x, float y);
struct vector2 _v2_add(struct vector2 v, struct vector2 v2);
struct vector2 _v2_sub(struct vector2 v, struct vector2 v2);
struct vector2 _v2_smul(struct vector2 v, float scalar);
struct vector2 _v2_sdiv(struct vector2 v, float scalar);
struct vector2 _v2_negate(struct vector2 v);
float _v2_mag(struct vector2 v);
int _v2_equ(struct vector2 v, struct vector2 v2);
struct vector2 _v2_cpy(struct vector2 v);

enum collider_types {
	RADIUS,
	AABB
};
union col {
	float radius;
	struct vector2 size;
};
struct collider {
	enum collider_types type;
	struct vector2 size;
	float radius;
};
struct collider _col_new(enum collider_types type, union col col);

struct rigidbody {
	struct vector2 position;
	struct vector2 lastposition;
	struct vector2 velocity;
	struct vector2 bounds;
	struct collider collider;
	float          mass;
};
struct rigidbody _rb_new(struct vector2 position, float mass, float radius);
struct rigidbody _rb_new_aabb(struct vector2 position, float mass, struct vector2 size);
int _rb_add_bounds(struct rigidbody *rb, struct vector2 bounds);
int _rb_add_force(struct rigidbody *rb, struct vector2 force);
int _rb_collision(struct rigidbody *rb, struct rigidbody *rb2);
enum out_of_bounds {
	BOUND_IN,
	BOUND_OUT_X_MAX,
	BOUND_OUT_X_MIN,
	BOUND_OUT_Y_MAX,
	BOUND_OUT_Y_MIN,
};
enum out_of_bounds _rb_handle(struct rigidbody *rb, float time);
int _rb_is_colliding(struct rigidbody rb, struct rigidbody rb2);
int _rb_default_bounce_bound_handling(struct rigidbody *rb, enum out_of_bounds handled);
int _rb_default_stop_bound_handling(struct rigidbody *rb, enum out_of_bounds handled);






