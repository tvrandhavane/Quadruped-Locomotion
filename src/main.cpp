#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>
#include "helper.h"
#include "imageLoader.h"

dBodyID ball_body;
dGeomID ball_geom;
dGeomID plane_geom;
dMass ball_mass;

helper * global_helper;

using namespace std;

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    const int N = 12;
    dContact contact[N];

    bool isGround = ((plane_geom == o1) || (plane_geom == o2));

    if (isGround)  {
        for (int i=0;i<N;i++) {
            contact[i].surface.mode = dContactBounce | dContactSoftCFM;
            contact[i].surface.mu = dInfinity;
            contact[i].surface.mu2 = 5000;
            contact[i].surface.bounce = 0.8;
            contact[i].surface.bounce_vel = 0.0;
            contact[i].surface.soft_cfm = 0.01;
        }
        int n =  dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
        
        for (int i = 0; i < n; i++) {
            dJointID c = dJointCreateContact(global_helper->getWorld(), global_helper->getCgroup(), &contact[i]);
            dJointAttach (c,dGeomGetBody(o1),dGeomGetBody(o2));
        }
    }
}

void Draw() {
    //texture
    imageLoader * image_tex= new imageLoader();
    GLuint ground_texture;
    ground_texture = image_tex->loadBMP_custom("./textures/002.bmp");
    //

    dSpaceCollide(global_helper->getSpace(), 0 ,&nearCallback);

    dWorldQuickStep (global_helper->getWorld(), 0.1);

    dJointGroupEmpty(global_helper->getCgroup());
    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
   
    
    glNormal3f(0.5, 0.5, 0.1);
    glPushMatrix();
        glTranslatef (100.0, 200.0, 0.0);

        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, ground_texture);
        //glColor3f(0.5, 0.1, 0.6);

        glBegin(GL_QUADS);
            glTexCoord2f(0.0, 0.0);
            glVertex3f(-1000, 10, 1000);
            glTexCoord2f(0.0, 1.0);
            glVertex3f(-1000, 10, -1000);
            glTexCoord2f(1.0, 1.0);
            glVertex3f(1000, 10, -1000);
            glTexCoord2f(1.0, 0.0);
            glVertex3f(1000, 10, 1000);
        glEnd();
        glDisable(GL_TEXTURE_2D);
        const dReal *realP = dBodyGetPosition(ball_body);
        glTranslatef(realP[0], realP[1], realP[2]);
        glutSolidSphere(10, 10, 10);
    glPopMatrix();
    glutSwapBuffers();
}

void Initialize() {
    glClearColor(0.8, 0.9, 0.8, 0.0);       //background colour

    glDepthFunc(GL_LESS);                   // The Type Of Depth Test To Do
    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);                // Enables Depth Testing
    
    glEnable(GL_LIGHTING);

    // Lighting set up
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glShadeModel(GL_SMOOTH);

    // Set lighting intensity and color
    GLfloat qaAmbientLight[]    = {0.2, 0.2, 0.2, 1.0};
    GLfloat qaDiffuseLight[]    = {1.0, 1.0, 1.0, 1.0};
    GLfloat qaSpecularLight[]   = {1.0, 1.0, 1.0, 1.0};
    glLightfv(GL_LIGHT0, GL_AMBIENT, qaAmbientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, qaDiffuseLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, qaSpecularLight);

    // Set the light position
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0,                  //The camera angle
                   800.0 / 600.0, //The width-to-height ratio
                   1.0,                   //The near z clipping coordinate
                   1000.0);               //The far z clipping coordinate

    GLfloat qaLightPosition[]   = {0.0, 0.0, 10.0, 0.0};
    glLightfv(GL_LIGHT0, GL_POSITION, qaLightPosition);
    glEnable(GL_LIGHT0);

    // Set material properties
    GLfloat qaBlack[] = {0.0, 0.0, 0.0, 0.8};
    GLfloat qaGreen[] = {0.0, 1.0, 0.0, 0.8};
    GLfloat qaWhite[] = {1.0, 1.0, 1.0, 0.8};
    glMaterialfv(GL_FRONT, GL_AMBIENT, qaWhite);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, qaWhite);
    glMaterialfv(GL_FRONT, GL_SPECULAR, qaGreen);
    glMaterialf(GL_FRONT, GL_SHININESS, 120.0);

    gluLookAt(0.0, 0.0, 400.0, 100.0, 100.0, 0.0, 0.0f, -1.0f, 0.0f);
}

void Timer(int iUnused)
{
    glutPostRedisplay();
    glutTimerFunc(1, Timer, 0);
}


int main(int argc, char** argv) {
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_REPEAT);    

    global_helper = new helper();

    glutInit(&argc, argv);  //initialize glut library
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE |  GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutInitWindowPosition(200, 200);
    glutCreateWindow("Quadruped Locomotion");
    Initialize();
    dInitODE();

    global_helper->initWorld();
    global_helper->initSpace();
    global_helper->initCgroup();

    ball_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&ball_mass);
    dMassSetSphereTotal(&ball_mass, 0.1, 0.2);
    dBodySetMass(ball_body, &ball_mass);

    ball_geom = dCreateSphere(global_helper->getSpace(), 0.2);
    dGeomSetData(ball_geom, (void *)"ball");
    dGeomSetBody(ball_geom, ball_body);


    dGeomSetPosition(ball_geom, 0.0, -200.0, 0.0);

    plane_geom = dCreatePlane(global_helper->getSpace(), 0.0, -0.01, 0.0, 0.0);


    glutDisplayFunc(Draw);
    Timer(0);

    glutMainLoop();
    dWorldDestroy(global_helper->getWorld());

    dCloseODE();
    return 0;
}