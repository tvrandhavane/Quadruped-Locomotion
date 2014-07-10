#ifndef DRAW_H
#define DRAW_H

#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>
#include <cmath>
#include "imageLoader.h"

class draw
{
private:
	//Private Objects
	imageLoader *image_tex;

	//Private Methods
	void draw_sky();
    void draw_ground();
    void draw_leg();
    void draw_tail();
    void draw_neck_head();
    void draw_back();
    void draw_dog();
    void draw_cube();
    
public:
    //Constructor
    draw();
    
    //Public Methods
    void draw_scene(float ball_location[]);

};

#endif // DRAW_H
