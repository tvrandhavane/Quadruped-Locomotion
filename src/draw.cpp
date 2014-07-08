#include "draw.h"

draw::draw(){
    image_tex= new imageLoader();
}

void draw::draw_scene(float ball_location[]){
    draw_sky();
    draw_paw();
    glPushMatrix();
        glTranslatef (100.0, 200.0, 0.0);
        draw_ground();
        glTranslatef(ball_location[0], ball_location[1], ball_location[2]);
        glutSolidSphere(10, 10, 10);
    glPopMatrix();
}

void draw::draw_paw(){
    glColor3f(1.0, 0.0, 0.0);
    glPushMatrix();
        glTranslatef (100.0, 100.0, 0.0);
        glBegin(GL_TRIANGLES);
            glVertex3f(10, 0, -10);
            glVertex3f(-10, 0, -10);
            glVertex3f(-20, 0, 0);
        glEnd();
        glBegin(GL_TRIANGLES);
            glVertex3f(10, 0, -10);
            glVertex3f(0, 10, 0);
            glVertex3f(-20, 0, 0);
        glEnd();
        glBegin(GL_TRIANGLES);
            glVertex3f(0, 10, 0);
            glVertex3f(-10, 0, -10);
            glVertex3f(-20, 0, 0);
        glEnd();
        glBegin(GL_TRIANGLES);
            glVertex3f(10, 0, -10);
            glVertex3f(-10, 0, -10);
            glVertex3f(0, 10, 0);
        glEnd();
    glPopMatrix();
}

void draw::draw_sky(){
    //Load texture image
    GLuint sky_texture;
    sky_texture = image_tex->loadBMP_custom("./textures/sky.bmp");

    //Enable and bind texture
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, sky_texture);

    //Draw sky quad
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

    //Disable texture
    glDisable(GL_TEXTURE_2D);
}

void draw::draw_ground(){
    //Load texture image
    GLuint ground_texture;
    ground_texture = image_tex->loadBMP_custom("./textures/floor.bmp");

    //Enable and bind texture
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, ground_texture);

    //Draw ground quad
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

    //Disable texture
    glDisable(GL_TEXTURE_2D);
}