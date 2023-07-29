# dophysics
a physics engine, in association with dopixels

Go to wiki for a complete documentation, due to its size being larger than should fit here but here is a "hello world" script that has two balls collding with each other (requires dopixels to work)

```C
#include <stdio.h>
#include <math.h>

#include "include/render/_ren.h" //replace with the path to your dopixels _ren.h
#include "_phys.h"

int main() {
  //setup the screen (dopixels)
  struct screen screen;
  _ren_setup(&screen, 400, 400);

  //setup the physics objects
  struct rigidbody rb  = _rb_new(_v2_new(1, 200),   10, 15),
                   rb2 = _rb_new(_v2_new(398, 200), 20, 25);
  _rb_add_bounds(&rb,  _v2_new(400, 400)); //sets up a bound for a bound handle function to deal with
  _rb_add_bounds(&rb2, _v2_new(400, 400));

  //add initial force
  _rb_add_force(&rb,  _v2_new(30, 0));
  _rb_add_force(&rb2, _v2_new(-50, 0));

  //main loop
  do {
    _ren_clear(&screen, 0x0); //dopixels clear screen

    int didbound = _rb_handle(&rb, screen.deltatime); //makes the balls move returns the state of the ball being in bounds that we set above
    _rb_default_bounce_bound_handling(&rb, didbound); //makes the balls bounce off the bounds with full velocity
    didbound = _rb_handle(&rb2, screen.deltatime);
    _rb_default_bounce_bound_handling(&rb2, didbound);

    if(_rb_is_colliding(rb, rb2))
      _rb_collision(&rb, &rb2);

    //draw the colliders
    _ren_drawrect(&screen, rb.position.x - rb.collider.radius, rb.position.y - rb.collider.radius, rb.position.x + rb.collider.radius, rb.position.y + rb.collider.radius, 0xFFFFFFFF);
    _ren_drawrect(&screen, rb2.position.x - rb2.collider.radius, rb2.position.y - rb2.collider.radius, rb2.position.x + rb2.collider.radius, rb2.position.y + rb2.collider.radius, 0xFFFFFFFF);

  } while(!_ren_handle(&screen)); //dopixels screen handling stuff
  _ren_quit(&screen); //quit the program
}
```
