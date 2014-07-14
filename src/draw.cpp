#include "draw.h"

draw::draw(ODEBodies * body_bag){
    image_tex= new imageLoader();
    this->body_bag = body_bag;
}

void draw::draw_scene(){
    const dReal *ball_location = dBodyGetPosition(body_bag->getBallBody());
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
    glPushMatrix();
        glTranslatef(30, 35, 0);
        draw_back();
        draw_leg();
        draw_tail();
        draw_neck_head();
    glPopMatrix();
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
        //Link 1
        glPushMatrix();
            const dReal *back_link_1_location = dBodyGetPosition(body_bag->getBackLink1Body());
            glTranslatef (back_link_1_location[0], back_link_1_location[1], back_link_1_location[2]);
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
        glTranslatef (240 + 60*cos((5*3.14)/180) + 60*cos(asin((60*sin((5*3.14)/180) + 10)/60)), -10, 0.0);
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
            glTranslatef(l/2 + l*cos((30*3.14)/180), -(3*l)/2 - l *sin((60*3.14)/180), 0);
            glRotatef(-60, 0, 0, 1);
            glScalef(60, 5, 5);
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_leg(){
    float front_x = 230/(5 + 4*cos((45*3.14)/180) + 3*cos((30*3.14)/180));

    //Front left leg
    glPushMatrix();
        glTranslatef(0, 0, -60);
        //thigh
        glPushMatrix();
            glRotatef(-90, 0, 0, 1);
            glScalef(5*front_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //midleg
        glPushMatrix();
            glTranslatef(0, -5*front_x, 0);
            glRotatef(-135, 0, 0, 1);
            glScalef(4*front_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //Ankle
        glPushMatrix();
            glTranslatef(-4*front_x*sin((45*3.14)/180), -5*front_x - 4*front_x*cos((45*3.14)/180), 0);
            glRotatef(-120, 0, 0, 1);
            glScalef(3*front_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //paw
        glPushMatrix();
            glTranslatef (-4*front_x*sin((45*3.14)/180) - (3*front_x)/2, -5*front_x - 4*front_x*cos((45*3.14)/180) - 3*front_x*cos((30*3.14)/180), 0.0);
            glRotatef(-180, 0, 0, 1);
            glScalef(5, 5, 5);
            draw_cube();
        glPopMatrix();
    glPopMatrix();

    //Front right leg
    glPushMatrix();
        glTranslatef(0, 0, 60);
        //thigh
        glPushMatrix();
            glRotatef(-90, 0, 0, 1);
            glScalef(5*front_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //midleg
        glPushMatrix();
            glTranslatef(0, -5*front_x, 0);
            glRotatef(-135, 0, 0, 1);
            glScalef(4*front_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //Ankle
        glPushMatrix();
            glTranslatef(-4*front_x*sin((45*3.14)/180), -5*front_x - 4*front_x*cos((45*3.14)/180), 0);
            glRotatef(-120, 0, 0, 1);
            glScalef(3*front_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //paw
        glPushMatrix();
            glTranslatef (-4*front_x*sin((45*3.14)/180) - (3*front_x)/2, -5*front_x - 4*front_x*cos((45*3.14)/180) - 3*front_x*cos((30*3.14)/180), 0.0);
            glRotatef(-180, 0, 0, 1);
            glScalef(5, 5, 5);
            draw_cube();
        glPopMatrix();
    glPopMatrix();

    float back_x = 220/(5 + 4*cos((45*3.14)/180) + 3*cos((30*3.14)/180));
    //Back left leg
    glPushMatrix();
        glTranslatef(240 + 60*cos((5*3.14)/180) + 60*cos(asin((60*sin((5*3.14)/180) + 10)/60)), -10, -60);
        //thigh
        glPushMatrix();
            glRotatef(-90, 0, 0, 1);
            glScalef(5*back_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //midleg
        glPushMatrix();
            glTranslatef(0, -5*back_x, 0);
            glRotatef(-135, 0, 0, 1);
            glScalef(4*back_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //Ankle
        glPushMatrix();
            glTranslatef(-4*back_x*sin((45*3.14)/180), -5*back_x - 4*back_x*cos((45*3.14)/180), 0);
            glRotatef(-120, 0, 0, 1);
            glScalef(3*back_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //paw
        glPushMatrix();
            glTranslatef (-4*back_x*sin((45*3.14)/180) - (3*back_x)/2, -5*back_x - 4*back_x*cos((45*3.14)/180) - 3*back_x*cos((30*3.14)/180), 0.0);
            glRotatef(-180, 0, 0, 1);
            glScalef(5, 5, 5);
            draw_cube();
        glPopMatrix();
    glPopMatrix();

    //Back right leg
    glPushMatrix();
        glTranslatef(240 + 60*cos((5*3.14)/180) + 60*cos(asin((60*sin((5*3.14)/180) + 10)/60)), -10, 60);
        //thigh
        glPushMatrix();
            glRotatef(-90, 0, 0, 1);
            glScalef(5*back_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //midleg
        glPushMatrix();
            glTranslatef(0, -5*back_x, 0);
            glRotatef(-135, 0, 0, 1);
            glScalef(4*back_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //Ankle
        glPushMatrix();
            glTranslatef(-4*back_x*sin((45*3.14)/180), -5*back_x - 4*back_x*cos((45*3.14)/180), 0);
            glRotatef(-120, 0, 0, 1);
            glScalef(3*back_x, 5, 5);
            draw_cube();
        glPopMatrix();
        //paw
        glPushMatrix();
            glTranslatef (-4*back_x*sin((45*3.14)/180) - (3*back_x)/2, -5*back_x - 4*back_x*cos((45*3.14)/180) - 3*back_x*cos((30*3.14)/180), 0.0);
            glRotatef(-180, 0, 0, 1);
            glScalef(5, 5, 5);
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