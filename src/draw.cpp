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
        draw_back();
        draw_leg();
        draw_tail();
        draw_nnh();
    glPopMatrix();
}

void draw::draw_cube(){
    /*//Load texture image
    GLuint cube_texture;
    cube_texture = image_tex->loadBMP_custom("./textures/object.bmp");

    //Enable and bind texture
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, cube_texture);*/

    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
        glColor3f(1.0f, 0.0f, 0.0f);
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

    glEnable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    //Disable texture
    //glDisable(GL_TEXTURE_2D);
}

void draw::draw_back(){
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *back_link_1_location = dBodyGetPosition(body_bag->getBackLink1Body());
            glTranslatef (back_link_1_location[0], back_link_1_location[1], back_link_1_location[2]);
            const dReal *back_link_1_rotation_matrix_ode = dBodyGetRotation(body_bag->getBackLink1Body());
            float back_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_link_1_rotation_matrix_openGL, back_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_link_1_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getBackLink1Length());
            draw_cube();
        glPopMatrix();

        //Link 2
        glPushMatrix();
            const dReal *back_link_2_location = dBodyGetPosition(body_bag->getBackLink2Body());
            glTranslatef (back_link_2_location[0], back_link_2_location[1], back_link_2_location[2]);
            const dReal *back_link_2_rotation_matrix_ode = dBodyGetRotation(body_bag->getBackLink2Body());
            float back_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_link_2_rotation_matrix_openGL, back_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_link_2_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getBackLink2Length());
            draw_cube();
        glPopMatrix();

        //Link 3
        glPushMatrix();
            const dReal *back_link_3_location = dBodyGetPosition(body_bag->getBackLink3Body());
            glTranslatef (back_link_3_location[0], back_link_3_location[1], back_link_3_location[2]);
            const dReal *back_link_3_rotation_matrix_ode = dBodyGetRotation(body_bag->getBackLink3Body());
            float back_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_link_3_rotation_matrix_openGL, back_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_link_3_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getBackLink3Length());
            draw_cube();
        glPopMatrix();

        //Link 4
        glPushMatrix();
            const dReal *back_link_4_location = dBodyGetPosition(body_bag->getBackLink4Body());
            glTranslatef (back_link_4_location[0], back_link_4_location[1], back_link_4_location[2]);
            const dReal *back_link_4_rotation_matrix_ode = dBodyGetRotation(body_bag->getBackLink4Body());
            float back_link_4_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_link_4_rotation_matrix_openGL, back_link_4_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_link_4_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getBackLink4Length());
            draw_cube();
        glPopMatrix();

        //Link 5
        glPushMatrix();
            const dReal *back_link_5_location = dBodyGetPosition(body_bag->getBackLink5Body());
            glTranslatef (back_link_5_location[0], back_link_5_location[1], back_link_5_location[2]);
            const dReal *back_link_5_rotation_matrix_ode = dBodyGetRotation(body_bag->getBackLink5Body());
            float back_link_5_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_link_5_rotation_matrix_openGL, back_link_5_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_link_5_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getBackLink5Length());
            draw_cube();
        glPopMatrix();

        //Link 6
        glPushMatrix();
            const dReal *back_link_6_location = dBodyGetPosition(body_bag->getBackLink6Body());
            glTranslatef (back_link_6_location[0], back_link_6_location[1], back_link_6_location[2]);
            const dReal *back_link_6_rotation_matrix_ode = dBodyGetRotation(body_bag->getBackLink6Body());
            float back_link_6_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_link_6_rotation_matrix_openGL, back_link_6_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_link_6_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getBackLink6Length());
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_nnh(){
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *nnh_link_1_location = dBodyGetPosition(body_bag->getNnhLink1Body());
            glTranslatef(nnh_link_1_location[0], nnh_link_1_location[1], nnh_link_1_location[2]);
            const dReal *nnh_link_1_rotation_matrix_ode = dBodyGetRotation(body_bag->getNnhLink1Body());
            float nnh_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(nnh_link_1_rotation_matrix_openGL, nnh_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(nnh_link_1_rotation_matrix_openGL);
            glScalef(4, 4, 60);
            draw_cube();
        glPopMatrix();
        //Link 2
        glPushMatrix();
            const dReal *nnh_link_2_location = dBodyGetPosition(body_bag->getNnhLink2Body());
            glTranslatef(nnh_link_2_location[0], nnh_link_2_location[1], nnh_link_2_location[2]);
            const dReal *nnh_link_2_rotation_matrix_ode = dBodyGetRotation(body_bag->getNnhLink2Body());
            float nnh_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(nnh_link_2_rotation_matrix_openGL, nnh_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(nnh_link_2_rotation_matrix_openGL);
            glScalef(4, 4, 60);
            draw_cube();
        glPopMatrix();
        //Link 3
        glPushMatrix();
            const dReal *nnh_link_3_location = dBodyGetPosition(body_bag->getNnhLink3Body());
            glTranslatef(nnh_link_3_location[0], nnh_link_3_location[1], nnh_link_3_location[2]);
            const dReal *nnh_link_3_rotation_matrix_ode = dBodyGetRotation(body_bag->getNnhLink3Body());
            float nnh_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(nnh_link_3_rotation_matrix_openGL, nnh_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(nnh_link_3_rotation_matrix_openGL);
            glScalef(4, 4, 60);
            draw_cube();
        glPopMatrix();
        //Link 4
        glPushMatrix();
            const dReal *nnh_link_4_location = dBodyGetPosition(body_bag->getNnhLink4Body());
            glTranslatef(nnh_link_4_location[0], nnh_link_4_location[1], nnh_link_4_location[2]);
            const dReal *nnh_link_4_rotation_matrix_ode = dBodyGetRotation(body_bag->getNnhLink4Body());
            float nnh_link_4_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(nnh_link_4_rotation_matrix_openGL, nnh_link_4_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(nnh_link_4_rotation_matrix_openGL);
            glScalef(4, 4, 60);
            draw_cube();
        glPopMatrix();
        /*//Link 1
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
        glPopMatrix();*/
    glPopMatrix();
}

void draw::draw_tail(){
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *tail_link_1_location = dBodyGetPosition(body_bag->getTailLink1Body());
            glTranslatef(tail_link_1_location[0], tail_link_1_location[1], tail_link_1_location[2]);
            const dReal *tail_link_1_rotation_matrix_ode = dBodyGetRotation(body_bag->getTailLink1Body());
            float tail_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(tail_link_1_rotation_matrix_openGL, tail_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(tail_link_1_rotation_matrix_openGL);
            glScalef(4, 4, 60);
            draw_cube();
        glPopMatrix();
        //Link 2
        glPushMatrix();
            const dReal *tail_link_2_location = dBodyGetPosition(body_bag->getTailLink2Body());
            glTranslatef(tail_link_2_location[0], tail_link_2_location[1], tail_link_2_location[2]);
            const dReal *tail_link_2_rotation_matrix_ode = dBodyGetRotation(body_bag->getTailLink2Body());
            float tail_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(tail_link_2_rotation_matrix_openGL, tail_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(tail_link_2_rotation_matrix_openGL);
            glScalef(4, 4, 60);
            draw_cube();
        glPopMatrix();
        //Link 3
        glPushMatrix();
            const dReal *tail_link_3_location = dBodyGetPosition(body_bag->getTailLink3Body());
            glTranslatef(tail_link_3_location[0], tail_link_3_location[1], tail_link_3_location[2]);
            const dReal *tail_link_3_rotation_matrix_ode = dBodyGetRotation(body_bag->getTailLink3Body());
            float tail_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(tail_link_3_rotation_matrix_openGL, tail_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(tail_link_3_rotation_matrix_openGL);
            glScalef(4, 4, 60);
            draw_cube();
        glPopMatrix();
        //Link 4
        glPushMatrix();
            const dReal *tail_link_4_location = dBodyGetPosition(body_bag->getTailLink4Body());
            glTranslatef(tail_link_4_location[0], tail_link_4_location[1], tail_link_4_location[2]);
            const dReal *tail_link_4_rotation_matrix_ode = dBodyGetRotation(body_bag->getTailLink4Body());
            float tail_link_4_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(tail_link_4_rotation_matrix_openGL, tail_link_4_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(tail_link_4_rotation_matrix_openGL);
            glScalef(4, 4, 60);
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_leg(){
    draw_front_legs();
    draw_back_legs();
}

void draw::draw_front_legs(){
    //Front left leg
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *front_left_foot_link_1_location = dBodyGetPosition(body_bag->getFootLinkBody(0,0));
            glTranslatef (front_left_foot_link_1_location[0], front_left_foot_link_1_location[1], front_left_foot_link_1_location[2]);
            const dReal *front_left_foot_link_1_rotation_matrix_ode = dBodyGetRotation(body_bag->getFootLinkBody(0,0));
            float front_left_foot_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_left_foot_link_1_rotation_matrix_openGL, front_left_foot_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_left_foot_link_1_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(0, 0));
            draw_cube();
        glPopMatrix();
        
        //Link 2
        glPushMatrix();
            const dReal *front_left_foot_link_2_location = dBodyGetPosition(body_bag->getFootLinkBody(0,1));
            glTranslatef (front_left_foot_link_2_location[0], front_left_foot_link_2_location[1], front_left_foot_link_2_location[2]);
            const dReal *front_left_foot_link_2_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(0,1));
            float front_left_foot_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_left_foot_link_2_rotation_matrix_openGL, front_left_foot_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_left_foot_link_2_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(0, 1));
            draw_cube();
        glPopMatrix();
        
        //Link 3
        glPushMatrix();
            const dReal *front_left_foot_link_3_location = dBodyGetPosition(body_bag->getFootLinkBody(0,2));
            glTranslatef (front_left_foot_link_3_location[0], front_left_foot_link_3_location[1], front_left_foot_link_3_location[2]);            
            const dReal *front_left_foot_link_3_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(0,2));
            float front_left_foot_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_left_foot_link_3_rotation_matrix_openGL, front_left_foot_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_left_foot_link_3_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(0, 2));
            draw_cube();
        glPopMatrix();

        //Link 4
        glPushMatrix();
            const dReal *front_left_foot_link_4_location = dBodyGetPosition(body_bag->getFootLinkBody(0,3));
            glTranslatef (front_left_foot_link_4_location[0], front_left_foot_link_4_location[1], front_left_foot_link_4_location[2]);            
            const dReal *front_left_foot_link_4_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(0,3));
            float front_left_foot_link_4_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_left_foot_link_4_rotation_matrix_openGL, front_left_foot_link_4_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_left_foot_link_4_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(0, 3));
            draw_cube();
        glPopMatrix();
    glPopMatrix();
    
    //Front right leg
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *front_right_foot_link_1_location = dBodyGetPosition(body_bag->getFootLinkBody(1, 0));
            glTranslatef (front_right_foot_link_1_location[0], front_right_foot_link_1_location[1], front_right_foot_link_1_location[2]);
            const dReal *front_right_foot_link_1_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(1, 0));
            float front_right_foot_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_right_foot_link_1_rotation_matrix_openGL, front_right_foot_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_right_foot_link_1_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(1, 0));
            draw_cube();
        glPopMatrix();

        //Link 2
        glPushMatrix();
            const dReal *front_right_foot_link_2_location = dBodyGetPosition(body_bag->getFootLinkBody(1, 1));
            glTranslatef (front_right_foot_link_2_location[0], front_right_foot_link_2_location[1], front_right_foot_link_2_location[2]);
            const dReal *front_right_foot_link_2_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(1, 1));
            float front_right_foot_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_right_foot_link_2_rotation_matrix_openGL, front_right_foot_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_right_foot_link_2_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(1, 1));
            draw_cube();
        glPopMatrix();

        //Link 3
        glPushMatrix();
            const dReal *front_right_foot_link_3_location = dBodyGetPosition(body_bag->getFootLinkBody(1, 2));
            glTranslatef (front_right_foot_link_3_location[0], front_right_foot_link_3_location[1], front_right_foot_link_3_location[2]);
            const dReal *front_right_foot_link_3_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(1, 2));
            float front_right_foot_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_right_foot_link_3_rotation_matrix_openGL, front_right_foot_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_right_foot_link_3_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(1, 2));
            draw_cube();
        glPopMatrix();

        //Link 4
        glPushMatrix();
            const dReal *front_right_foot_link_4_location = dBodyGetPosition(body_bag->getFootLinkBody(1, 3));
            glTranslatef (front_right_foot_link_4_location[0], front_right_foot_link_4_location[1], front_right_foot_link_4_location[2]);
            const dReal *front_right_foot_link_4_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(1, 3));
            float front_right_foot_link_4_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(front_right_foot_link_4_rotation_matrix_openGL, front_right_foot_link_4_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(front_right_foot_link_4_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(1, 3));
            draw_cube();
        glPopMatrix();
    glPopMatrix();
}

void draw::draw_back_legs(){
    //left leg
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *back_left_foot_link_1_location = dBodyGetPosition(body_bag->getFootLinkBody(2, 0));
            glTranslatef (back_left_foot_link_1_location[0], back_left_foot_link_1_location[1], back_left_foot_link_1_location[2]);
            const dReal *back_left_foot_link_1_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(2, 0));
            float back_left_foot_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_left_foot_link_1_rotation_matrix_openGL, back_left_foot_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_left_foot_link_1_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(2, 0));
            draw_cube();
        glPopMatrix();
        
        //Link 2
        glPushMatrix();
            const dReal *back_left_foot_link_2_location = dBodyGetPosition(body_bag->getFootLinkBody(2, 1));
            glTranslatef (back_left_foot_link_2_location[0], back_left_foot_link_2_location[1], back_left_foot_link_2_location[2]);
            const dReal *back_left_foot_link_2_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(2, 1));
            float back_left_foot_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_left_foot_link_2_rotation_matrix_openGL, back_left_foot_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_left_foot_link_2_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(2, 1));
            draw_cube();
        glPopMatrix();
        
        //Link 3
        glPushMatrix();
            const dReal *back_left_foot_link_3_location = dBodyGetPosition(body_bag->getFootLinkBody(2, 2));
            glTranslatef (back_left_foot_link_3_location[0], back_left_foot_link_3_location[1], back_left_foot_link_3_location[2]);
            const dReal *back_left_foot_link_3_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(2, 2));
            float back_left_foot_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_left_foot_link_3_rotation_matrix_openGL, back_left_foot_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_left_foot_link_3_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(2, 2));
            draw_cube();
        glPopMatrix();

        //Link 4
        glPushMatrix();
            const dReal *back_left_foot_link_4_location = dBodyGetPosition(body_bag->getFootLinkBody(2, 3));
            glTranslatef (back_left_foot_link_4_location[0], back_left_foot_link_4_location[1], back_left_foot_link_4_location[2]);
            const dReal *back_left_foot_link_4_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(2, 3));
            float back_left_foot_link_4_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_left_foot_link_4_rotation_matrix_openGL, back_left_foot_link_4_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_left_foot_link_4_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(2, 3));
            draw_cube();
        glPopMatrix();
    glPopMatrix();
    
    //right leg
    glPushMatrix();
        //Link 1
        glPushMatrix();
            const dReal *back_right_foot_link_1_location = dBodyGetPosition(body_bag->getFootLinkBody(3, 0));
            glTranslatef (back_right_foot_link_1_location[0], back_right_foot_link_1_location[1], back_right_foot_link_1_location[2]);
            const dReal *back_right_foot_link_1_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(3, 0));
            float back_right_foot_link_1_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_right_foot_link_1_rotation_matrix_openGL, back_right_foot_link_1_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_right_foot_link_1_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(3, 0));
            draw_cube();
        glPopMatrix();

        //Link 2
        glPushMatrix();
            const dReal *back_right_foot_link_2_location = dBodyGetPosition(body_bag->getFootLinkBody(3, 1));
            glTranslatef (back_right_foot_link_2_location[0], back_right_foot_link_2_location[1], back_right_foot_link_2_location[2]);
            const dReal *back_right_foot_link_2_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(3, 1));
            float back_right_foot_link_2_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_right_foot_link_2_rotation_matrix_openGL, back_right_foot_link_2_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_right_foot_link_2_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(3, 1));
            draw_cube();
        glPopMatrix();

        //Link 3
        glPushMatrix();
            const dReal *back_right_foot_link_3_location = dBodyGetPosition(body_bag->getFootLinkBody(3, 2));
            glTranslatef (back_right_foot_link_3_location[0], back_right_foot_link_3_location[1], back_right_foot_link_3_location[2]);
            const dReal *back_right_foot_link_3_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(3, 2));
            float back_right_foot_link_3_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_right_foot_link_3_rotation_matrix_openGL, back_right_foot_link_3_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_right_foot_link_3_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(3, 2));
            draw_cube();
        glPopMatrix();

        //Link 4
        glPushMatrix();
            const dReal *back_right_foot_link_4_location = dBodyGetPosition(body_bag->getFootLinkBody(3, 3));
            glTranslatef (back_right_foot_link_4_location[0], back_right_foot_link_4_location[1], back_right_foot_link_4_location[2]);
            const dReal *back_right_foot_link_4_rotation_matrix_ode =  dBodyGetRotation(body_bag->getFootLinkBody(3, 3));
            float back_right_foot_link_4_rotation_matrix_openGL[16];
            getOpenGLRotationMatrix(back_right_foot_link_4_rotation_matrix_openGL, back_right_foot_link_4_rotation_matrix_ode);
            //rotate the link
            glMultMatrixf(back_right_foot_link_4_rotation_matrix_openGL);
            glScalef(4, 4, body_bag->getFootLinkLength(3, 3));
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
    /*//Load texture image
    GLuint ground_texture;
    ground_texture = image_tex->loadBMP_custom("./textures/floor.bmp");

    //Enable and bind texture
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, ground_texture);*/

    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    //Draw ground quad
    glBegin(GL_QUADS);
        glColor3f(0.5f, 0.5f, 0.5f);
        glTexCoord2f(0.0, 0.0);
        glVertex3f(-10000, -250, 10000);
        glTexCoord2f(0.0, 1.0);
        glVertex3f(-10000, -250, -10000);
        glTexCoord2f(1.0, 1.0);
        glVertex3f(10000, -250, -10000);
        glTexCoord2f(1.0, 0.0);
        glVertex3f(10000, -250, 10000);
    glEnd();

    glEnable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    //Disable texture
    //glDisable(GL_TEXTURE_2D);
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