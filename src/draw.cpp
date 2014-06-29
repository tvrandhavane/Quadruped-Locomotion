#include "draw.h"

draw::draw(){
    image_tex= new imageLoader();
}

void draw::draw_sky(){
    GLuint sky_texture;
    sky_texture = image_tex->loadBMP_custom("./textures/sky.bmp");

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, sky_texture);
    glBegin(GL_QUADS);
        glTexCoord2f(0.0, 0.0);
        glVertex3f(-1000, 375, -100);
        glTexCoord2f(0.0, 0.25);
        glVertex3f(-1000, -180, -100);
        glTexCoord2f(0.25, 0.25);
        glVertex3f(1000, -180, -100);
        glTexCoord2f(0.25, 0.0);
        glVertex3f(1000, 375, -100);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void draw::draw_ground(){
    GLuint ground_texture;
    ground_texture = image_tex->loadBMP_custom("./textures/floor.bmp");

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, ground_texture);
    glBegin(GL_QUADS);
        glTexCoord2f(0.0, 0.0);
        glVertex3f(-10000, -400, 10000);
        glTexCoord2f(0.0, 1.0);
        glVertex3f(-10000, -400, -10000);
        glTexCoord2f(1.0, 1.0);
        glVertex3f(10000, -400, -10000);
        glTexCoord2f(1.0, 0.0);
        glVertex3f(10000, -400, 10000);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}