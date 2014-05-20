#include <GL/glut.h>
#include <ode/ode.h>
#include <iostream>

dWorldID world;
dBodyID ball_body;
dSpaceID space;
dGeomID ball_geom;
dGeomID plane_geom;
dMass ball_mass;
dJointGroupID cgroup;
float angle=0.0;

using namespace std;

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    const int N = 12;
    dContact contact[N];

    int isGround = ((plane_geom == o1) || (plane_geom == o2));

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
        cout<<n<<endl;
        for (int i = 0; i < n; i++) {
            dJointID c = dJointCreateContact(world,cgroup,&contact[i]);
            dJointAttach (c,dGeomGetBody(o1),dGeomGetBody(o2));
        }
    }
}

void Draw() {
    dSpaceCollide(space,0,&nearCallback);

    dWorldQuickStep (world,0.1);

    dJointGroupEmpty(cgroup);
    glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
    //glColor3f(1.0, 0.0, 1.0);

    //glLoadIdentity();
    //glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

    if (angle>360.0) {
        angle=0.0;

    }
    angle+=1;
    glNormal3f(0.5, 0.5, 0.1);
    glPushMatrix();
    glutSolidSphere(5, 10, 10);
        glTranslatef (100.0, 200.0, 0.0);
        const dReal *realP = dBodyGetPosition(ball_body);
        glTranslatef(realP[0], realP[1], realP[2]);
        glutSolidSphere(10, 10, 10);
    glPopMatrix();
    glutSwapBuffers();
}

void Initialize() {
    glClearColor(0.8, 0.9, 0.8, 0.0);
    //glClearDepth(1.0);                // Enables Clearing Of The Depth Buffer
    glDepthFunc(GL_LESS);                   // The Type Of Depth Test To Do
    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);                // Enables Depth Testing
    glDisable(GL_LIGHTING);

    glEnable(GL_LIGHTING);

    //glMatrixMode(GL_MODELVIEW);
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
    glOrtho(0.0, 200.0, 200.0, 0.0, -10.0, 1000.0);
    //gluPerspective(45.0f,1.0f,0.1f,100.0f);   // Calculate The Aspect Ratio Of The Window
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
    gluLookAt (100.0, 100.0, 500, 0.5, 0.5, 0, 0, 1, 0);
}
void Timer(int iUnused)
{
    glutPostRedisplay();
    glutTimerFunc(1, Timer, 0);
}


int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE |  GLUT_DEPTH);
    glutInitWindowSize(480, 480);
    glutInitWindowPosition(200, 200);
    glutCreateWindow("test");
    Initialize();
    dInitODE();

    world = dWorldCreate();
    dWorldSetGravity(world, 0.0, 10.0, 0.0);
    dWorldSetERP(world, 0.9);
    dWorldSetCFM(world, 1e-4);
    dWorldSetLinearDamping(world, 0.00001);
    dWorldSetAngularDamping(world, 0.005);
    dWorldSetMaxAngularSpeed(world, 200);

    dWorldSetContactMaxCorrectingVel (world,0.1);
    dWorldSetContactSurfaceLayer (world,0.1);

    space = dSimpleSpaceCreate(0);
    cgroup = dJointGroupCreate(0);

    ball_body = dBodyCreate(world);
    dMassSetZero(&ball_mass);
    dMassSetSphereTotal(&ball_mass, 0.1, 0.2);
    dBodySetMass(ball_body, &ball_mass);

    ball_geom = dCreateSphere(space, 0.2);
    dGeomSetData(ball_geom, (void *)"ball");
    dGeomSetBody(ball_geom, ball_body);


    dGeomSetPosition(ball_geom, 0.0, -200.0, 0.0);

    plane_geom =dCreatePlane(space, 0.0, -0.01, 0.0, 0.0);


    glutDisplayFunc(Draw);
    Timer(0);

    glutMainLoop();
    dWorldDestroy(world);

    dCloseODE();
    return 0;
}