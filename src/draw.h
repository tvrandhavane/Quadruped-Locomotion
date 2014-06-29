#ifndef DRAW_H
#define DRAW_H

#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>
#include "imageLoader.h"

class draw
{
private:
	imageLoader *image_tex;

public:
    draw();
    void draw_sky();
    void draw_ground();

};

#endif // DRAW_H
