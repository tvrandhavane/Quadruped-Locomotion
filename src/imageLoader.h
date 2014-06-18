#ifndef IMAGELOADER_H
#define IMAGELOADER_H

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

using namespace std;

class imageLoader {

	public:
	imageLoader();
	GLuint loadBMP_custom(const char * imagepath);

};

#endif // IMAGELOADER_H