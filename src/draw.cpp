#include "draw.h"

draw::draw(){
    image_tex= new imageLoader();
}

void draw::draw_scene(float ball_location[]){
    draw_sky();
    draw_dog();
    glPushMatrix();
        glTranslatef (100.0, 200.0, 0.0);
        draw_ground();
        glTranslatef(ball_location[0], ball_location[1], ball_location[2]);
        glutSolidSphere(10, 10, 10);
    glPopMatrix();
}

void draw::draw_dog(){
    draw_back();
    draw_leg();
    draw_tail();
    draw_neck_head();
}

void draw::draw_cube(){
    glPushMatrix();
        //Base
        glBegin(GL_QUADS);
            glVertex3f(0, 0, 0);
            glVertex3f(1, 0, 0);
            glVertex3f(1, 0, 1);
            glVertex3f(0, 0, 1);
        glEnd();
        //Top
        glBegin(GL_QUADS);
            glVertex3f(0, 1, 0);
            glVertex3f(1, 1, 0);
            glVertex3f(1, 1, 1);
            glVertex3f(0, 1, 1);
        glEnd();
        //Front
        glBegin(GL_QUADS);
            glVertex3f(0, 0, 0);
            glVertex3f(1, 0, 0);
            glVertex3f(1, 1, 0);
            glVertex3f(0, 1, 0);
        glEnd();
        //Back
        glBegin(GL_QUADS);
            glVertex3f(0, 0, 1);
            glVertex3f(1, 0, 1);
            glVertex3f(1, 1, 1);
            glVertex3f(0, 1, 1);
        glEnd();
        //Left
        glBegin(GL_QUADS);
            glVertex3f(0, 0, 0);
            glVertex3f(0, 0, 1);
            glVertex3f(0, 1, 1);
            glVertex3f(0, 1, 0);
        glEnd();
        //Right
        glBegin(GL_QUADS);
            glVertex3f(1, 0, 0);
            glVertex3f(1, 0, 1);
            glVertex3f(1, 1, 1);
            glVertex3f(1, 1, 0);
        glEnd();
    glPopMatrix();
}

void draw::draw_back(){
    glPushMatrix();
        glTranslatef(30, 35, 0);
        //Link 1
        glPushMatrix();
            glTranslatef (0, 0, 0.0);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();

        //Link 2
        glPushMatrix();
            glTranslatef (60.0, 0, 0.0);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();

        //Link 3
        glPushMatrix();
            glTranslatef (120.0, 0, 0.0);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();

        //Link 4
        glPushMatrix();
            glTranslatef (180.0, 0, 0.0);
            glRotatef(5, 0, 0, 1);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();

        //Link 5
        glPushMatrix();
            glTranslatef (180 + 60*cos((5*3.14)/180), 60*sin((5*3.14)/180), 0.0);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();

        //Link 6
        glPushMatrix();
            glTranslatef (240 + 60*cos((5*3.14)/180), 60*sin((5*3.14)/180), 0.0);
            glRotatef((-asin((60*sin((5*3.14)/180) + 10)/60)) * (180/3.14), 0, 0, 1);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_neck_head(){
    int l = 40;
    glPushMatrix();
        glTranslatef(30, 35, 0);
        //Link 1
        glPushMatrix();
            glRotatef(120, 0, 0, 1);
            glScalef(l, 5, 5);
            draw_cube();
        glPopMatrix();
        //Link 2
        glPushMatrix();
            glTranslatef(-l/2, l*sin((60*3.14)/180), 0);
            glRotatef(90, 0, 0, 1);
            glScalef(l, 5, 5);
            draw_cube();
        glPopMatrix();
        //Link 3
        glPushMatrix();
            glTranslatef(-l/2, l + l*sin((60*3.14)/180), 0);
            glRotatef(120, 0, 0, 1);
            glScalef(l, 5, 5);
            draw_cube();
        glPopMatrix();
        //Link 4
        glPushMatrix();
            glTranslatef(-l, l + 2*l*sin((60*3.14)/180), 0);
            glRotatef(150, 0, 0, 1);
            glScalef(l, 5, 5);
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_tail(){
    int l = 60;
    glPushMatrix();
        glTranslatef (270 + 60*cos((5*3.14)/180) + 60*cos(asin((60*sin((5*3.14)/180) + 10)/60)), 25, 0.0);
        //Link 1
        glPushMatrix();
            glRotatef(-30, 0, 0, 1);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();
        //Link 2
        glPushMatrix();
            glTranslatef(l*cos((30*3.14)/180), -l/2, 0);
            glRotatef(-60, 0, 0, 1);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();
        //Link 3
        glPushMatrix();
            glTranslatef(l/2 + l*cos((30*3.14)/180), -l/2 - l*sin((60*3.14)/180), 0);
            glRotatef(-90, 0, 0, 1);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();
        //Link 4
        glPushMatrix();
            glTranslatef(l/2 + l*cos((30*3.14)/180), -(3*l)/2 - l*sin((60*3.14)/180), 0);
            glRotatef(-60, 0, 0, 1);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_leg(){
    glPushMatrix();
        //paw
        glPushMatrix();
            glTranslatef (80.0, -100.0, 0.0);
            glScalef(10, 5, 5);
            draw_cube();
        glPopMatrix();
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