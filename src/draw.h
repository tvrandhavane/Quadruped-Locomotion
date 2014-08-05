#ifndef DRAW_H
#define DRAW_H

#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>
#include <cmath>
#include "imageLoader.h"
#include "ODEBodies.h"

using namespace std;

class draw
{
private:
	//Private Objects
	imageLoader * image_tex;
	ODEBodies * body_bag;

	//Private Methods
	void draw_sky();
    void draw_ground();
    void draw_leg();
    void draw_tail();
    void draw_nnh();
    void draw_back();
    void draw_dog();
    void draw_cube();

    void getOpenGLRotationMatrix(float * opengl, const float * ode);
    
public:
    //Constructor
    draw(ODEBodies * body_bag);
    
    //Public Methods
    void draw_scene();

};

#endif // DRAW_H
