#include "draw.h"

draw::draw(ODEBodies * body_bag){
    image_tex= new imageLoader();
    this->body_bag = body_bag;
}

void draw::draw_scene(){
    //const dReal *ball_location = dBodyGetPosition(body_bag->getBallBody());
    //draw_sky();
    glPushMatrix();
        draw_ground();
    glPopMatrix();
    /*glPushMatrix();
        glTranslatef(ball_location[0], ball_location[1], ball_location[2]);
        glutSolidSphere(5, 10, 10);
    glPopMatrix();*/
    glPushMatrix();
        draw_dog();
    glPopMatrix();
}

void draw::draw_dog(){
    glPushMatrix();
        //draw_back();
        draw_leg();
        //draw_tail();
        //draw_nnh();
    glPopMatrix();
}

void draw::draw_cube(){
    //Load texture image
    GLuint cube_texture;
    cube_texture = image_tex->loadBMP_custom("./textures/object.bmp");

    //Enable and bind texture
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, cube_texture);

    glPushMatrix();
        //Base
        glBegin(GL_QUADS);
            glVertex3f(-0.5, -0.5, -0.5);
            glVertex3f(0.5, -0.5, -0.5);
            glVertex3f(0.5, -0.5, 0.5);
            glVertex3f(-0.5, -0.5, 0.5);
        glEnd();
        //Top
        glBegin(GL_QUADS);
            glVertex3f(-0.5, 0.5, -0.5);
            glVertex3f(0.5, 0.5, -0.5);
            glVertex3f(0.5, 0.5, 0.5);
            glVertex3f(-0.5, 0.5, 0.5);
        glEnd();
        //Front
        glBegin(GL_QUADS);
            glVertex3f(-0.5, -0.5, -0.5);
            glVertex3f(0.5, -0.5, -0.5);
            glVertex3f(0.5, 0.5, -0.5);
            glVertex3f(-0.5, 0.5, -0.5);
        glEnd();
        //Back
        glBegin(GL_QUADS);
            glVertex3f(-0.5, -0.5, 0.5);
            glVertex3f(0.5, -0.5, 0.5);
            glVertex3f(0.5, 0.5, 0.5);
            glVertex3f(-0.5, 0.5, 0.5);
        glEnd();
        //Left
        glBegin(GL_QUADS);
            glVertex3f(-0.5, -0.5, -0.5);
            glVertex3f(-0.5, -0.5, 0.5);
            glVertex3f(-0.5, 0.5, 0.5);
            glVertex3f(-0.5, 0.5, -0.5);
        glEnd();
        //Right
        glBegin(GL_QUADS);
            glVertex3f(0.5, -0.5, -0.5);
            glVertex3f(0.5, -0.5, 0.5);
            glVertex3f(0.5, 0.5, 0.5);
            glVertex3f(0.5, 0.5, -0.5);
        glEnd();
    glPopMatrix();

    //Disable texture
    glDisable(GL_TEXTURE_2D);
}

void draw::draw_back(){
    glPushMatrix();
        //Link 1
        glPushMatrix();
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
        glPopMatrix();

        //Link 4
        glPushMatrix();
            const dReal *back_link_4_location = dBodyGetPosition(body_bag->getBackLink4Body());
            glTranslatef (back_link_4_location[0], back_link_4_location[1], back_link_4_location[2]);
            const dReal *back_link_4_rotation_matrix_ode =  dBodyGetRotation(body_bag->getBackLink4Body());
            float back_link_4_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_link_4_rotation_matrix_openGL, back_link_4_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_link_4_rotation_matrix_openGL);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 5
        glPushMatrix();
            const dReal *back_link_5_location = dBodyGetPosition(body_bag->getBackLink5Body());
            glTranslatef (back_link_5_location[0], back_link_5_location[1], back_link_5_location[2]);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 6
        glPushMatrix();
            const dReal *back_link_6_location = dBodyGetPosition(body_bag->getBackLink6Body());
            glTranslatef (back_link_6_location[0], back_link_6_location[1], back_link_6_location[2]);

            const dReal *back_link_6_rotation_matrix_ode =  dBodyGetRotation(body_bag->getBackLink6Body());
            float back_link_6_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_link_6_rotation_matrix_openGL, back_link_6_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_link_6_rotation_matrix_openGL);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_nnh(){
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *nnh_link_1_location = dBodyGetPosition(body_bag->getNnhLink1Body());
            glTranslatef (nnh_link_1_location[0], nnh_link_1_location[1], nnh_link_1_location[2]);
            const dReal *nnh_link_1_rotation_matrix_ode =  dBodyGetRotation(body_bag->getNnhLink1Body());
            float nnh_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(nnh_link_1_rotation_matrix_openGL, nnh_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(nnh_link_1_rotation_matrix_openGL);
            glScalef(60, 4, 4);
            draw_cube();
        glPopMatrix();
        //Link 2
        glPushMatrix();
            const dReal *nnh_link_2_location = dBodyGetPosition(body_bag->getNnhLink2Body());
            glTranslatef (nnh_link_2_location[0], nnh_link_2_location[1], nnh_link_2_location[2]);
            const dReal *nnh_link_2_rotation_matrix_ode =  dBodyGetRotation(body_bag->getNnhLink2Body());
            float nnh_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(nnh_link_2_rotation_matrix_openGL, nnh_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(nnh_link_2_rotation_matrix_openGL);
            glScalef(60, 4, 4);
            draw_cube();
        glPopMatrix();
        //Link 3
        glPushMatrix();
            const dReal *nnh_link_3_location = dBodyGetPosition(body_bag->getNnhLink3Body());
            glTranslatef (nnh_link_3_location[0], nnh_link_3_location[1], nnh_link_3_location[2]);
            const dReal *nnh_link_3_rotation_matrix_ode =  dBodyGetRotation(body_bag->getNnhLink3Body());
            float nnh_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(nnh_link_3_rotation_matrix_openGL, nnh_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(nnh_link_3_rotation_matrix_openGL);
            glScalef(60, 4, 4);
            draw_cube();
        glPopMatrix();
        //Link 3
        glPushMatrix();
            const dReal *nnh_link_4_location = dBodyGetPosition(body_bag->getNnhLink4Body());
            glTranslatef (nnh_link_4_location[0], nnh_link_4_location[1], nnh_link_4_location[2]);
            const dReal *nnh_link_4_rotation_matrix_ode =  dBodyGetRotation(body_bag->getNnhLink4Body());
            float nnh_link_4_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(nnh_link_4_rotation_matrix_openGL, nnh_link_4_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(nnh_link_4_rotation_matrix_openGL);
            glScalef(60, 4, 4);
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_tail(){
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *tail_link_1_location = dBodyGetPosition(body_bag->getTailLink1Body());
            glTranslatef (tail_link_1_location[0], tail_link_1_location[1], tail_link_1_location[2]);
            const dReal *tail_link_1_rotation_matrix_ode =  dBodyGetRotation(body_bag->getTailLink1Body());
            float tail_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(tail_link_1_rotation_matrix_openGL, tail_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(tail_link_1_rotation_matrix_openGL);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();
        //Link 2
        glPushMatrix();
            const dReal *tail_link_2_location = dBodyGetPosition(body_bag->getTailLink2Body());
            glTranslatef (tail_link_2_location[0], tail_link_2_location[1], tail_link_2_location[2]);
            const dReal *tail_link_2_rotation_matrix_ode =  dBodyGetRotation(body_bag->getTailLink2Body());
            float tail_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(tail_link_2_rotation_matrix_openGL, tail_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(tail_link_2_rotation_matrix_openGL);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();
        //Link 3
        glPushMatrix();
            const dReal *tail_link_3_location = dBodyGetPosition(body_bag->getTailLink3Body());
            glTranslatef (tail_link_3_location[0], tail_link_3_location[1], tail_link_3_location[2]);
            const dReal *tail_link_3_rotation_matrix_ode =  dBodyGetRotation(body_bag->getTailLink3Body());
            float tail_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(tail_link_3_rotation_matrix_openGL, tail_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(tail_link_3_rotation_matrix_openGL);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();
        //Link 4
        glPushMatrix();
            const dReal *tail_link_4_location = dBodyGetPosition(body_bag->getTailLink4Body());
            glTranslatef (tail_link_4_location[0], tail_link_4_location[1], tail_link_4_location[2]);
            const dReal *tail_link_4_rotation_matrix_ode =  dBodyGetRotation(body_bag->getTailLink4Body());
            float tail_link_4_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(tail_link_4_rotation_matrix_openGL, tail_link_4_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(tail_link_4_rotation_matrix_openGL);
            glScalef(80, 4, 4);
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_leg(){
    draw_front_legs();
    //draw_back_legs();
}

void draw::draw_back_legs(){
    //left leg
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *back_left_foot_link_1_location = dBodyGetPosition(body_bag->getBackLeftFootLink1Body());
            glTranslatef (back_left_foot_link_1_location[0], back_left_foot_link_1_location[1], back_left_foot_link_1_location[2]);
            const dReal *back_left_foot_link_1_rotation_matrix_ode =  dBodyGetRotation(body_bag->getBackLeftFootLink1Body());
            float back_left_foot_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_left_foot_link_1_rotation_matrix_openGL, back_left_foot_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_left_foot_link_1_rotation_matrix_openGL);
            glScalef(body_bag->getBackFootLink1Length(), 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 2
        glPushMatrix();
            const dReal *back_left_foot_link_2_location = dBodyGetPosition(body_bag->getBackLeftFootLink2Body());
            glTranslatef (back_left_foot_link_2_location[0], back_left_foot_link_2_location[1], back_left_foot_link_2_location[2]);
            const dReal *back_left_foot_link_2_rotation_matrix_ode =  dBodyGetRotation(body_bag->getBackLeftFootLink2Body());
            float back_left_foot_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_left_foot_link_2_rotation_matrix_openGL, back_left_foot_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_left_foot_link_2_rotation_matrix_openGL);
            glScalef(body_bag->getBackFootLink2Length(), 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 3
        glPushMatrix();
            const dReal *back_left_foot_link_3_location = dBodyGetPosition(body_bag->getBackLeftFootLink3Body());
            glTranslatef (back_left_foot_link_3_location[0], back_left_foot_link_3_location[1], back_left_foot_link_3_location[2]);
            const dReal *back_left_foot_link_3_rotation_matrix_ode =  dBodyGetRotation(body_bag->getBackLeftFootLink3Body());
            float back_left_foot_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_left_foot_link_3_rotation_matrix_openGL, back_left_foot_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_left_foot_link_3_rotation_matrix_openGL);
            glScalef(body_bag->getBackFootLink3Length(), 4, 4);
            draw_cube();
        glPopMatrix();
    glPopMatrix();

    //right leg
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *back_right_foot_link_1_location = dBodyGetPosition(body_bag->getBackRightFootLink1Body());
            glTranslatef (back_right_foot_link_1_location[0], back_right_foot_link_1_location[1], back_right_foot_link_1_location[2]);
            const dReal *back_right_foot_link_1_rotation_matrix_ode =  dBodyGetRotation(body_bag->getBackRightFootLink1Body());
            float back_right_foot_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_right_foot_link_1_rotation_matrix_openGL, back_right_foot_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_right_foot_link_1_rotation_matrix_openGL);
            glScalef(body_bag->getBackFootLink1Length(), 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 2
        glPushMatrix();
            const dReal *back_right_foot_link_2_location = dBodyGetPosition(body_bag->getBackRightFootLink2Body());
            glTranslatef (back_right_foot_link_2_location[0], back_right_foot_link_2_location[1], back_right_foot_link_2_location[2]);
            const dReal *back_right_foot_link_2_rotation_matrix_ode =  dBodyGetRotation(body_bag->getBackRightFootLink2Body());
            float back_right_foot_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_right_foot_link_2_rotation_matrix_openGL, back_right_foot_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_right_foot_link_2_rotation_matrix_openGL);
            glScalef(body_bag->getBackFootLink2Length(), 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 3
        glPushMatrix();
            const dReal *back_right_foot_link_3_location = dBodyGetPosition(body_bag->getBackRightFootLink3Body());
            glTranslatef (back_right_foot_link_3_location[0], back_right_foot_link_3_location[1], back_right_foot_link_3_location[2]);
            const dReal *back_right_foot_link_3_rotation_matrix_ode =  dBodyGetRotation(body_bag->getBackRightFootLink3Body());
            float back_right_foot_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_right_foot_link_3_rotation_matrix_openGL, back_right_foot_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_right_foot_link_3_rotation_matrix_openGL);
            glScalef(body_bag->getBackFootLink3Length(), 4, 4);
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_front_legs(){
    //Front left leg
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *front_left_foot_link_1_location = dBodyGetPosition(body_bag->getFrontLeftFootLink1Body());
            glTranslatef (front_left_foot_link_1_location[0], front_left_foot_link_1_location[1], front_left_foot_link_1_location[2]);
            cout << "link 1 location = " << front_left_foot_link_1_location[0] << ", " << front_left_foot_link_1_location[1] << ", " << front_left_foot_link_1_location[2] << endl;
            const dReal *front_left_foot_link_1_rotation_matrix_ode = dBodyGetRotation(body_bag->getFrontLeftFootLink1Body());
            float front_left_foot_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_left_foot_link_1_rotation_matrix_openGL, front_left_foot_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_left_foot_link_1_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFrontFootLink1Length());
            cout << "getFrontFootLink1Length = " << body_bag->getFrontFootLink1Length() << endl;
            draw_cube();
        glPopMatrix();

        /*
        //Link 2
        glPushMatrix();
            const dReal *front_left_foot_link_2_location = dBodyGetPosition(body_bag->getFrontLeftFootLink2Body());
            glTranslatef (front_left_foot_link_2_location[0], front_left_foot_link_2_location[1], front_left_foot_link_2_location[2]);
            cout << "link 2 location = " << front_left_foot_link_2_location[0] << ", " << front_left_foot_link_2_location[1] << ", " << front_left_foot_link_2_location[2] << endl;
            const dReal *front_left_foot_link_2_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFrontLeftFootLink2Body());
            float front_left_foot_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_left_foot_link_2_rotation_matrix_openGL, front_left_foot_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_left_foot_link_2_rotation_matrix_openGL);
            glScalef(body_bag->getFrontFootLink2Length(), 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 3
        glPushMatrix();
            const dReal *front_left_foot_link_3_location = dBodyGetPosition(body_bag->getFrontLeftFootLink3Body());
            glTranslatef (front_left_foot_link_3_location[0], front_left_foot_link_3_location[1], front_left_foot_link_3_location[2]);            
            const dReal *front_left_foot_link_3_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFrontLeftFootLink3Body());
            float front_left_foot_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_left_foot_link_3_rotation_matrix_openGL, front_left_foot_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_left_foot_link_3_rotation_matrix_openGL);
            glScalef(body_bag->getFrontFootLink3Length(), 4, 4);
            draw_cube();
        glPopMatrix();
    glPopMatrix();

    //Front right leg
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *front_right_foot_link_1_location = dBodyGetPosition(body_bag->getFrontRightFootLink1Body());
            glTranslatef (front_right_foot_link_1_location[0], front_right_foot_link_1_location[1], front_right_foot_link_1_location[2]);
            const dReal *front_right_foot_link_1_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFrontRightFootLink1Body());
            float front_right_foot_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_right_foot_link_1_rotation_matrix_openGL, front_right_foot_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_right_foot_link_1_rotation_matrix_openGL);
            glScalef(body_bag->getFrontFootLink1Length(), 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 2
        glPushMatrix();
            const dReal *front_right_foot_link_2_location = dBodyGetPosition(body_bag->getFrontRightFootLink2Body());
            glTranslatef (front_right_foot_link_2_location[0], front_right_foot_link_2_location[1], front_right_foot_link_2_location[2]);
            const dReal *front_right_foot_link_2_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFrontRightFootLink2Body());
            float front_right_foot_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_right_foot_link_2_rotation_matrix_openGL, front_right_foot_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_right_foot_link_2_rotation_matrix_openGL);
            glScalef(body_bag->getFrontFootLink2Length(), 4, 4);
            draw_cube();
        glPopMatrix();

        //Link 3
        glPushMatrix();
            const dReal *front_right_foot_link_3_location = dBodyGetPosition(body_bag->getFrontRightFootLink3Body());
            glTranslatef (front_right_foot_link_3_location[0], front_right_foot_link_3_location[1], front_right_foot_link_3_location[2]);
            const dReal *front_right_foot_link_3_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFrontRightFootLink3Body());
            float front_right_foot_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_right_foot_link_3_rotation_matrix_openGL, front_right_foot_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_right_foot_link_3_rotation_matrix_openGL);
            glScalef(body_bag->getFrontFootLink3Length(), 4, 4);
            draw_cube();
        glPopMatrix();
    glPopMatrix();
        /*glTranslatef(0, 0, -60);
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
        glPopMatrix();*/
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

void draw::getOpenGLRotationMatrix(float * opengl, const float * ode){
    opengl[0] = ode[0];
    opengl[1] = ode[4];
    opengl[2] = ode[8];
    opengl[3] = 0;

    opengl[4] = ode[1];
    opengl[5] = ode[5];
    opengl[6] = ode[9];
    opengl[7] = 0;

    opengl[8] = ode[2];
    opengl[9] = ode[6];
    opengl[10] = ode[10];
    opengl[11] = 0;

    opengl[12] = ode[3];
    opengl[13] = ode[7];
    opengl[14] = ode[11];
    opengl[15] = 1;
}