#include "draw.h"

draw::draw(ODEBodies * body_bag){
    image_tex= new imageLoader();
    this->body_bag = body_bag;
}

void draw::draw_scene(){
    const dReal *ball_location = dBodyGetPosition(body_bag->getBallBody());
    draw_sky();
    glPushMatrix();
        draw_ground();
    glPopMatrix();
    glPushMatrix();
        glTranslatef(ball_location[0], ball_location[1], ball_location[2]);
        glutSolidSphere(5, 10, 10);
    glPopMatrix();
    glPushMatrix();
        draw_dog();
    glPopMatrix();
}

void draw::draw_dog(){
    glPushMatrix();
        draw_back();
        //draw_leg();
        //draw_tail();
        //draw_neck_head();
    glPopMatrix();
}

void draw::draw_cube(){
    //Load texture image
    GLuint sky_texture;
    sky_texture = image_tex->loadBMP_custom("./textures/object.bmp");

    //Enable and bind texture
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, sky_texture);

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

    //Disable texture
    glDisable(GL_TEXTURE_2D);
}

void draw::draw_back(){
    glPushMatrix();
        //Link 1
        /*glPushMatrix();
            const dReal *back_link_1_location = dBodyGetPosition(body_bag->getBackLink1Body());
            glTranslatef (back_link_1_location[0], back_link_1_location[1], back_link_1_location[2]);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 2
        glPushMatrix();
            const dReal *back_link_2_location = dBodyGetPosition(body_bag->getBackLink2Body());
            glTranslatef (back_link_2_location[0], back_link_2_location[1], back_link_2_location[2]);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 3
        glPushMatrix();
            const dReal *back_link_3_location = dBodyGetPosition(body_bag->getBackLink3Body());
            glTranslatef (back_link_3_location[0], back_link_3_location[1], back_link_3_location[2]);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();*/

        //Link 4
        glPushMatrix();
            const dReal *back_link_4_location = dBodyGetPosition(body_bag->getBackLink4Body());
            glTranslatef (back_link_4_location[0], back_link_4_location[1], back_link_4_location[2]);
            const dReal *rotationMatrix =  dBodyGetRotation(body_bag->getBackLink4Body());


            float m[16];
            m[0] = rotationMatrix[0];
            m[1] = rotationMatrix[1];
            m[2] = rotationMatrix[2];
            m[3] = 0;

            m[4] = rotationMatrix[3];
            m[5] = rotationMatrix[4];
            m[6] = rotationMatrix[5];
            m[7] = 0;

            m[8] = rotationMatrix[6];
            m[9] = rotationMatrix[7];
            m[10] = rotationMatrix[8];
            m[11] = 0;

            m[12] = rotationMatrix[9];
            m[13] = rotationMatrix[10];
            m[14] = rotationMatrix[11];
            m[15] = 1;
            glMultMatrixf(rotationMatrix);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();

        /*//Link 5
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
        glPopMatrix();*/
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

    glPushMatrix();
        //Draw sky quad
        glBegin(GL_QUADS);
            glTexCoord2f(0, 0);
            glVertex3f(-1000, 500, -100);
            glTexCoord2f(0, 2);
            glVertex3f(-1000, -500, -100);
            glTexCoord2f(4, 2);
            glVertex3f(1000, -500, -100);
            glTexCoord2f(4, 0);
            glVertex3f(1000, 500, -100);
        glEnd();
    glPopMatrix();

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
        glVertex3f(-10000, -250, 10000);
        glTexCoord2f(0.0, 1.0);
        glVertex3f(-10000, -250, -10000);
        glTexCoord2f(1.0, 1.0);
        glVertex3f(10000, -250, -10000);
        glTexCoord2f(1.0, 0.0);
        glVertex3f(10000, -250, 10000);
    glEnd();

    //Disable texture
    glDisable(GL_TEXTURE_2D);
}