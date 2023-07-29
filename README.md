# dophysics
a physics engine, in association with dopixels

Go to wiki for a complete documentation, due to its size being larger than should fit here but here is a "hello world" script that has two balls collding with each other (requires dopixels to work)

```C
#include <stdio.h>
#include <math.h>

#include "include/render/_ren.h" //replace with the path to your dopixels _ren.h
#include "_phys.h"

int main() {
    //setup screen
    struct screen screen;
    _ren_setup(&screen, 400, 400);

    //setup physics
    struct rigidbody rb = _rb_new(_v2_new(0, 200), 10, 15),
                     rb2 = _rb_new_aabb(_v2_new(398, 188.5), 20, _v2_new(25, 25));
    _rb_add_bounds(&rb, _v2_new(400, 400));
    _rb_add_bounds(&rb2, _v2_new(400, 400));
    _rb_add_force(&rb, _v2_new(30, 0));
    _rb_add_force(&rb2, _v2_new(-50, 0));
    do {
        _ren_clear(&screen, 0x0);
        _rb_default_bounce_bound_handling(&rb, _rb_handle(&rb, screen.deltatime));
        _rb_default_bounce_bound_handling(&rb2, _rb_handle(&rb2, screen.deltatime));

        if(_rb_is_colliding(rb, rb2))
            _rb_collision(&rb, &rb2);

        _ren_drawrect(&screen, rb.position.x - rb.collider.radius, rb.position.y - rb.collider.radius, rb.position.x + rb.collider.radius, rb.position.y + rb.collider.radius, 0xFFFFFFFF);
        _ren_drawrect(&screen, rb2.position.x, rb2.position.y, rb2.position.x + rb2.collider.size.x, rb2.position.y + rb2.collider.size.y, 0xFFFFFFFF);
    } while(!_ren_handle(&screen));
    _ren_quit(&screen);
}
```
