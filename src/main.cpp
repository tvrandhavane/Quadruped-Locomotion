#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>
#include "helper.h"
#include "draw.h"
#include "imageLoader.h"
#include "ODEBodies.h"
#include "controller.h"
#include "quadrupedFramework.h"

using namespace std;

//Declare global objects
quadrupedFramework * quadFramework;
draw * draw_obj;
bool step;

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    //Set up maximum number of contact points and contact array
    const int N = 12;
    dContact contact[N];

    //Check if collision is between ball and plane
    bool isGround = ((quadFramework->getBodyBag()->getPlaneGeom() == o1) || (quadFramework->getBodyBag()->getPlaneGeom() == o2));

    //If collision, do the following
    if (isGround)  {
        //Set up contact parameters
        for (int i=0;i<N;i++) {
            contact[i].surface.mode = dContactBounce | dContactSoftCFM;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.mu2 = 5000;
            contact[i].surface.bounce = 0.8;
            contact[i].surface.bounce_vel = 0.0;
            contact[i].surface.soft_cfm = 0.01;
        }
        
        //Generate n number of contacts between object o1 and object o2,
        //Contact array is used to store contacts
        //Maximum N number of contacts can be created
        //Give the start of contact geom, skip size of one contact to store the next contact location in array
        int n =  dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
        
        //For each contact create a joint        
        for (int i = 0; i < n; i++) {
            dJointID c = dJointCreateContact(quadFramework->getBodyBag()->getGlobalHelper()->getWorld(), quadFramework->getBodyBag()->getGlobalHelper()->getCgroup(), &contact[i]);
            dJointAttach (c, dGeomGetBody(o1), dGeomGetBody(o2));
        }
    }
}

void Draw() {
    quadFramework->takeStep();
    
    //Collides all objects in space
    dSpaceCollide(quadFramework->getBodyBag()->getGlobalHelper()->getSpace(), 0 ,&nearCallback);

    //Set stepsize and use quick step, computationally cheaper
    dWorldStep(quadFramework->getBodyBag()->getGlobalHelper()->getWorld(), 0.1);

    dJointGroupEmpty(quadFramework->getBodyBag()->getGlobalHelper()->getCgroup());

    //Clear openGL buffers
    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);    

    //Draw scene  
    draw_obj->draw_scene();

    //Glut swap buffers
    glutSwapBuffers();
}

void Initialize() {
    //Set up background color
    glClearColor(0.2, 0.2, 0.7, 0.0);

    //Set up depth parameters
    glDepthFunc(GL_LESS);
    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);    

    // Lighting set up
    glEnable(GL_LIGHTING);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glShadeModel(GL_SMOOTH);

    // Set lighting intensity, color and position
    GLfloat qaAmbientLight[]    = {0.2, 0.2, 0.2, 1.0};
    GLfloat qaDiffuseLight[]    = {1.0, 1.0, 1.0, 1.0};
    GLfloat qaSpecularLight[]   = {1.0, 1.0, 1.0, 1.0};
    GLfloat qaLightPosition[]   = {0.0, 0.0, 10.0, 0.0};
    glLightfv(GL_LIGHT0, GL_AMBIENT, qaAmbientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, qaDiffuseLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, qaSpecularLight);
    glLightfv(GL_LIGHT0, GL_POSITION, qaLightPosition);    
    
    //Enable light
    glEnable(GL_LIGHT0);

    //Set up viewing perspective
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0,                    //The camera angle
                   1000.0 / 600.0,           //The width-to-height ratio
                   1.0,                     //The near z clipping coordinate
                   2000.0);                 //The far z clipping coordinate    

    // Set material properties
    GLfloat qaBlack[] = {0.0, 0.0, 0.0, 0.8};
    GLfloat qaGreen[] = {0.0, 1.0, 0.0, 0.8};
    GLfloat qaWhite[] = {1.0, 1.0, 1.0, 0.8};
    glMaterialfv(GL_FRONT, GL_AMBIENT, qaWhite);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, qaWhite);
    glMaterialfv(GL_FRONT, GL_SPECULAR, qaGreen);
    glMaterialf(GL_FRONT, GL_SHININESS, 120.0);

    //Texture parameters
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_REPEAT);   

    //Set up look at vector
    gluLookAt(0.0, 0.0, 1500.0, 0.0, 0.0, 0.0, 0.0f, 1.0f, 0.0f);    
}

void Keyboard(unsigned char key, int x, int y)
{
    switch (key){
        case 27:             // ESCAPE key
            exit (0);
            break;
        case 'w':
            glutPostRedisplay();
            break;
    }   
}

void Timer(int iUnused){
    //Call the display function (Draw() function)
    glutPostRedisplay();

    //Set up next timer
    if(step == false){
        glutTimerFunc(1, Timer, 0);
    }    
}


int main(int argc, char** argv) {
    if(argv[1][0] == '0'){
        step = false;
    }
    else{
        step = true;
    }

    //Set up glut parameters
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE |  GLUT_DEPTH);
    glutInitWindowSize(1000, 600);
    glutInitWindowPosition(200, 100);
    glutCreateWindow("Quadruped Locomotion");

    //Initialize the scene
    
    Initialize();

    //Initialize ODE
    dInitODE();    

    //Set up quadruped framework
    quadFramework = new quadrupedFramework();
    

    draw_obj = new draw(quadFramework->getBodyBag());

    //Set up display function which will be called in loop
    glutDisplayFunc(Draw);
    glutKeyboardFunc (Keyboard);

    //Set up timer
    Timer(0);
    glutMainLoop();

    //Destroy ODE World and close ODE
    dWorldDestroy(quadFramework->getBodyBag()->getGlobalHelper()->getWorld());
    dCloseODE();

    return 0;
}