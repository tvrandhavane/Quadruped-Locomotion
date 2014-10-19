#include "ODEBodies.h"

ODEBodies::ODEBodies(helper * global_helper, float * root_position){
	this->global_helper = global_helper;
    this->root_position = root_position;
}

void ODEBodies::step(){
    //Apply forces
    //dBodyAddForce(back_link_2_body, 0.0, 98.1, 0.0);
}

vector<float> ODEBodies::getLengths(int leg_id){
    vector<float> ret(4);
    //front
    if(leg_id == 0){
        ret[0] = front_foot_link_1_length;
        ret[1] = front_foot_link_2_length;
        ret[2] = front_foot_link_3_length;
        ret[3] = front_foot_link_4_length;
    }
    //back
    else{
        ret[0] = back_foot_link_1_length;
        ret[1] = back_foot_link_2_length;
        ret[2] = back_foot_link_3_length;
        ret[3] = back_foot_link_4_length;
    }
    return ret;
}

vector<float> ODEBodies::getAngles(int leg_id){
    vector<float> ret(4);
    //front
    if(leg_id == 0){
        ret[0] = front_foot_link_1_theta;
        ret[1] = front_foot_link_2_theta;
        ret[2] = front_foot_link_3_theta;
        ret[3] = front_foot_link_4_theta;
    }
    //back
    else{
        ret[0] = back_foot_link_1_theta;
        ret[1] = back_foot_link_2_theta;
        ret[2] = back_foot_link_3_theta;
        ret[3] = back_foot_link_4_theta;
    }
    return ret;
}

void ODEBodies::init(){
	//set_ball();
    set_back();
    set_nnh();
    set_tail();
    set_leg();
	set_plane();
}

void ODEBodies::extendMatrixTo4x3(dReal * inp_R, dReal * outp_R){
    outp_R[0] = inp_R[0];
    outp_R[1] = inp_R[1];
    outp_R[2] = inp_R[2];
    outp_R[3] = 0;

    outp_R[4] = inp_R[3];
    outp_R[5] = inp_R[4];
    outp_R[6] = inp_R[5];
    outp_R[7] = 0;

    outp_R[8] = inp_R[6];
    outp_R[9] = inp_R[7];
    outp_R[10] = inp_R[8];
    outp_R[11] = 0;
}

void ODEBodies::multiplyMatrices(dReal* matC, dReal* matA, dReal* matB, int rC, int cC, int rA, int cA, int rB, int cB) {
    for (int i = 0; i < rA; i++) {
        for (int j = 0; j < cB; j++) {
            dReal sum = 0.0;
            for (int k = 0; k < rB; k++)
                sum = sum + matA[k * cA + i] * matB[j * cB + k];
            matC[j * cC + i] = sum;
        }
    }
}

void ODEBodies::setLink(dBodyID * link_body, dMass * link_mass, dReal link_length, dReal z_rotation_angle, dGeomID *link_geom, dReal * position){
    *link_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(link_mass);
    dMassSetCylinderTotal(link_mass, 10, 2, 2.0, link_length);
    dBodySetMass(*link_body, link_mass);
    //dBodySetLinearVel(*link_body, 0.0, 0.0, 0.0);
    
    dReal rotation_matrix_x[9];
    dReal rotation_matrix_z[9];
    dReal rotation_matrix_final_3x3[9];
    dReal rotation_matrix_final_4x3[12];

    setRotationMatrixXAxis(rotation_matrix_x, (90*M_PI)/180);    
    setRotationMatrixZAxis(rotation_matrix_z, z_rotation_angle);
    multiplyMatrices(rotation_matrix_final_3x3, rotation_matrix_x, rotation_matrix_z, 3, 3, 3, 3, 3, 3);
    extendMatrixTo4x3(rotation_matrix_final_3x3, rotation_matrix_final_4x3);
    dBodySetRotation(*link_body, rotation_matrix_final_4x3);

    *link_geom = dCreateCylinder(global_helper->getSpace(), 2.0, link_length);
    dGeomSetData(*link_geom, (void *)"front_left_foot_link_1");
    dGeomSetBody(*link_geom, *link_body);
    dGeomSetPosition(*link_geom, position[0], position[1], position[2]);
}

void ODEBodies::setRotationMatrixXAxis(dReal * R,dReal theta){
    R[0] = 1;
    R[1] = 0;
    R[2] = 0;

    R[3] = 0;
    R[4] = cos(theta);
    R[5] = sin(theta);

    R[6] = 0;
    R[7] = (-1)*sin(theta);
    R[8] = cos(theta);
}

void ODEBodies::setRotationMatrixZAxis(dReal * R,dReal theta){
    R[0] = cos(theta);
    R[1] = sin(theta);
    R[2] = 0;

    R[3] = (-1)*sin(theta);
    R[4] = cos(theta);
    R[5] = 0;

    R[6] = 0;
    R[7] = 0;
    R[8] = 1;
}

void ODEBodies::set_ball(){
	ball_body = dBodyCreate(global_helper->getWorld());
    dMassSetZero(&ball_mass);
    dMassSetCylinderTotal(&ball_mass, 1, 2, 1.0, 10);
    dBodySetMass(ball_body, &ball_mass);
    dBodySetLinearVel(ball_body, -7.0, 0.0, 0.0);

    ball_geom = dCreateCylinder(global_helper->getSpace(), 1.0, 10.0);
    dGeomSetData(ball_geom, (void *)"ball");
    dGeomSetBody(ball_geom, ball_body);
    dGeomSetPosition(ball_geom, 0.0, 0.0, 0.0);
}

void ODEBodies::set_back(){
    back_link_1_length = 80;
    back_link_2_length = 80;
    back_link_3_length = 80;
    back_link_4_length = 80;
    back_link_5_length = 80;
    back_link_6_length = 80;

    back_link_1_theta = (90*M_PI)/180;
    back_link_2_theta = (90*M_PI)/180;
    back_link_3_theta = (90*M_PI)/180;
    back_link_4_theta = (85*M_PI)/180;
    back_link_5_theta = (90*M_PI)/180;
    back_link_6_theta = asin((back_link_4_length*cos(back_link_4_theta) + 20)/back_link_6_length) + (90*M_PI)/180;

    //Link 1    
    dReal back_link_1_position[3];
    back_link_1_position[0] = root_position[0] + back_link_1_length/2;
    back_link_1_position[1] = root_position[1];
    back_link_1_position[2] = root_position[2];

    setLink(&back_link_1_body, &back_link_1_mass, back_link_1_length, back_link_1_theta, &back_link_1_geom, back_link_1_position);

    //Link 2    
    dReal back_link_2_position[3];
    back_link_2_position[0] = root_position[0] + back_link_1_length + back_link_2_length/2;
    back_link_2_position[1] = root_position[1];
    back_link_2_position[2] = root_position[2];

    setLink(&back_link_2_body, &back_link_2_mass, back_link_2_length, back_link_2_theta, &back_link_2_geom, back_link_2_position);

    //Link 3    
    dReal back_link_3_position[3];
    back_link_3_position[0] = root_position[0] + back_link_1_length + back_link_2_length + back_link_3_length/2;
    back_link_3_position[1] = root_position[1];
    back_link_3_position[2] = root_position[2];


    setLink(&back_link_3_body, &back_link_3_mass, back_link_3_length, back_link_3_theta, &back_link_3_geom, back_link_3_position);

    //Link 4    
    dReal back_link_4_position[3];
    back_link_4_position[0] = root_position[0] + back_link_1_length + back_link_2_length + back_link_3_length + (back_link_4_length/2)*sin(back_link_4_theta);
    back_link_4_position[1] = root_position[1] + (back_link_4_length/2)*cos(back_link_4_theta);
    back_link_4_position[2] = root_position[2];


    setLink(&back_link_4_body, &back_link_4_mass, back_link_4_length, back_link_4_theta, &back_link_4_geom, back_link_4_position);

    //Link 5    
    dReal back_link_5_position[3];
    back_link_5_position[0] = root_position[0] + back_link_1_length + back_link_2_length + back_link_3_length + back_link_4_length*sin(back_link_4_theta) + back_link_5_length/2;
    back_link_5_position[1] = root_position[1] + back_link_4_length*cos(back_link_4_theta);
    back_link_5_position[2] = root_position[2];


    setLink(&back_link_5_body, &back_link_5_mass, back_link_5_length, back_link_5_theta, &back_link_5_geom, back_link_5_position);

    //Link 6    
    dReal back_link_6_position[3];
    back_link_6_position[0] = root_position[0] + back_link_1_length + back_link_2_length + back_link_3_length + back_link_4_length*sin(back_link_4_theta) + back_link_5_length + (back_link_6_length/2)*sin(back_link_6_theta);
    back_link_6_position[1] = root_position[1] + back_link_4_length*cos(back_link_4_theta) + (back_link_6_length/2)*cos(back_link_6_theta);
    back_link_6_position[2] = root_position[2];


    setLink(&back_link_6_body, &back_link_6_mass, back_link_6_length, back_link_6_theta, &back_link_6_geom, back_link_6_position);

    //Joints
    //Link 1 and link 2
    dReal back_joint_1_2_position[3];
    back_joint_1_2_position[0] = root_position[0] + back_link_1_length;
    back_joint_1_2_position[1] = root_position[1];
    back_joint_1_2_position[2] = root_position[2];

    dJointID back_joint_1_2 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(back_joint_1_2, back_link_1_body, back_link_2_body);
    dJointSetBallAnchor (back_joint_1_2, back_joint_1_2_position[0], back_joint_1_2_position[1], back_joint_1_2_position[2]);

    //Link 2 and link 3
    dReal back_joint_2_3_position[3];
    back_joint_2_3_position[0] = root_position[0] + back_link_1_length + back_link_2_length;
    back_joint_2_3_position[1] = root_position[1];
    back_joint_2_3_position[2] = root_position[2];

    dJointID back_joint_2_3 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(back_joint_2_3, back_link_2_body, back_link_3_body);
    dJointSetBallAnchor (back_joint_2_3, back_joint_2_3_position[0], back_joint_2_3_position[1], back_joint_2_3_position[2]);

    //Link 3 and link 4
    dReal back_joint_3_4_position[3];
    back_joint_3_4_position[0] = root_position[0] + back_link_1_length + back_link_2_length + back_link_3_length;
    back_joint_3_4_position[1] = root_position[1];
    back_joint_3_4_position[2] = root_position[2];

    dJointID back_joint_3_4 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(back_joint_3_4, back_link_3_body, back_link_4_body);
    dJointSetBallAnchor (back_joint_3_4, back_joint_3_4_position[0], back_joint_3_4_position[1], back_joint_3_4_position[2]);

    //Link 4 and link 5
    dReal back_joint_4_5_position[3];
    back_joint_4_5_position[0] = root_position[0] + back_link_1_length + back_link_2_length + back_link_3_length + back_link_4_length*sin(back_link_4_theta);
    back_joint_4_5_position[1] = root_position[1] + back_link_4_length*cos(back_link_4_theta);
    back_joint_4_5_position[2] = root_position[2];

    dJointID back_joint_4_5 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(back_joint_4_5, back_link_4_body, back_link_5_body);
    dJointSetBallAnchor (back_joint_4_5, back_joint_4_5_position[0], back_joint_4_5_position[1], back_joint_4_5_position[2]);

    //Link 5 and link 6
    dReal back_joint_5_6_position[3];
    back_joint_5_6_position[0] = root_position[0] + back_link_1_length + back_link_2_length + back_link_3_length + back_link_4_length*sin(back_link_4_theta) + back_link_5_length*sin(back_link_5_theta) ;
    back_joint_5_6_position[1] = root_position[1] + back_link_4_length*cos(back_link_4_theta);
    back_joint_5_6_position[2] = root_position[2];

    dJointID back_joint_5_6 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(back_joint_5_6, back_link_5_body, back_link_6_body);
    dJointSetBallAnchor (back_joint_5_6, back_joint_5_6_position[0], back_joint_5_6_position[1], back_joint_5_6_position[2]);
}

void ODEBodies::set_nnh(){
    dReal nnh_link_1_theta = (-30*M_PI)/180;
    dReal nnh_link_2_theta = (0*M_PI)/180;
    dReal nnh_link_3_theta = (-30*M_PI)/180;
    dReal nnh_link_4_theta = (-60*M_PI)/180;

    dReal nnh_link_1_length = 60.0;
    dReal nnh_link_2_length = 60.0;
    dReal nnh_link_3_length = 60.0;
    dReal nnh_link_4_length = 60.0;

    //Link 1    
    dReal nnh_link_1_position[3];
    nnh_link_1_position[0] = root_position[0] + (nnh_link_1_length/2)*sin(nnh_link_1_theta);
    nnh_link_1_position[1] = root_position[1] + (nnh_link_1_length/2)*cos(nnh_link_1_theta);
    nnh_link_1_position[2] = root_position[2];

    setLink(&nnh_link_1_body, &nnh_link_1_mass, nnh_link_1_length, nnh_link_1_theta, &nnh_link_1_geom, nnh_link_1_position);

    //Link 2    
    dReal nnh_link_2_position[3];
    nnh_link_2_position[0] = root_position[0] + nnh_link_1_length*sin(nnh_link_1_theta) + (nnh_link_2_length/2)*sin(nnh_link_2_theta);
    nnh_link_2_position[1] = root_position[1] + nnh_link_1_length*cos(nnh_link_1_theta) + (nnh_link_2_length/2)*cos(nnh_link_2_theta); 
    nnh_link_2_position[2] = root_position[2];

    setLink(&nnh_link_2_body, &nnh_link_2_mass, nnh_link_2_length, nnh_link_2_theta, &nnh_link_2_geom, nnh_link_2_position);

    //Link 3  
    dReal nnh_link_3_position[3];
    nnh_link_3_position[0] = root_position[0] + nnh_link_1_length*sin(nnh_link_1_theta) + nnh_link_2_length*sin(nnh_link_2_theta) + (nnh_link_3_length/2)*sin(nnh_link_3_theta);
    nnh_link_3_position[1] = root_position[1] + nnh_link_1_length*cos(nnh_link_1_theta) + nnh_link_2_length*cos(nnh_link_2_theta) + (nnh_link_3_length/2)*cos(nnh_link_3_theta); 
    nnh_link_3_position[2] = root_position[2];

    setLink(&nnh_link_3_body, &nnh_link_3_mass, nnh_link_3_length, nnh_link_3_theta, &nnh_link_3_geom, nnh_link_3_position);

    //Link 4    
    dReal nnh_link_4_position[3];
    nnh_link_4_position[0] = root_position[0] + nnh_link_1_length*sin(nnh_link_1_theta) + nnh_link_2_length*sin(nnh_link_2_theta) + nnh_link_3_length*sin(nnh_link_3_theta) + (nnh_link_4_length/2)*sin(nnh_link_4_theta);
    nnh_link_4_position[1] = root_position[1] + nnh_link_1_length*cos(nnh_link_1_theta) + nnh_link_2_length*cos(nnh_link_2_theta) + nnh_link_3_length*cos(nnh_link_3_theta) + (nnh_link_4_length/2)*cos(nnh_link_4_theta); 
    nnh_link_4_position[2] = root_position[2];

    setLink(&nnh_link_4_body, &nnh_link_4_mass, nnh_link_4_length, nnh_link_4_theta, &nnh_link_4_geom, nnh_link_4_position);

    //Joints
    //Back and link 1
    dReal nnh_joint_back_1_nnh_1_position[3];
    nnh_joint_back_1_nnh_1_position[0] = root_position[0];
    nnh_joint_back_1_nnh_1_position[1] = root_position[1];
    nnh_joint_back_1_nnh_1_position[2] = root_position[2];

    dJointID nnh_joint_back_1_nnh_1 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(nnh_joint_back_1_nnh_1, back_link_1_body, nnh_link_1_body);
    dJointSetBallAnchor (nnh_joint_back_1_nnh_1, nnh_joint_back_1_nnh_1_position[0], nnh_joint_back_1_nnh_1_position[1], nnh_joint_back_1_nnh_1_position[2]);

    //Link 1 and link 2
    dReal nnh_joint_1_2_position[3];
    nnh_joint_1_2_position[0] = root_position[0] + nnh_link_1_length*sin(nnh_link_1_theta);
    nnh_joint_1_2_position[1] = root_position[1] + nnh_link_1_length*cos(nnh_link_1_theta);
    nnh_joint_1_2_position[2] = root_position[2];

    dJointID nnh_joint_1_2 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(nnh_joint_1_2, nnh_link_1_body, nnh_link_2_body);
    dJointSetBallAnchor (nnh_joint_1_2, nnh_joint_1_2_position[0], nnh_joint_1_2_position[1], nnh_joint_1_2_position[2]);

    //Link 2 and link 3
    dReal nnh_joint_2_3_position[3];
    nnh_joint_2_3_position[0] = root_position[0] + nnh_link_1_length*sin(nnh_link_1_theta) + nnh_link_2_length*sin(nnh_link_2_theta);
    nnh_joint_2_3_position[1] = root_position[1] + nnh_link_1_length*cos(nnh_link_1_theta) + nnh_link_2_length*cos(nnh_link_2_theta);
    nnh_joint_2_3_position[2] = root_position[2];

    dJointID nnh_joint_2_3 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(nnh_joint_2_3, nnh_link_2_body, nnh_link_3_body);
    dJointSetBallAnchor (nnh_joint_2_3, nnh_joint_2_3_position[0], nnh_joint_2_3_position[1], nnh_joint_2_3_position[2]);

    //Link 3 and link 4
    dReal nnh_joint_3_4_position[3];
    nnh_joint_3_4_position[0] = root_position[0] + nnh_link_1_length*sin(nnh_link_1_theta) + nnh_link_2_length*sin(nnh_link_2_theta) + nnh_link_3_length*sin(nnh_link_3_theta);
    nnh_joint_3_4_position[1] = root_position[1] + nnh_link_1_length*cos(nnh_link_1_theta) + nnh_link_2_length*cos(nnh_link_2_theta) + nnh_link_3_length*cos(nnh_link_3_theta);
    nnh_joint_3_4_position[2] = root_position[2];

    dJointID nnh_joint_3_4 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(nnh_joint_3_4, nnh_link_3_body, nnh_link_4_body);
    dJointSetBallAnchor (nnh_joint_3_4, nnh_joint_3_4_position[0], nnh_joint_3_4_position[1], nnh_joint_3_4_position[2]);
}

void ODEBodies::set_tail(){
    dReal tail_link_1_theta = (120*M_PI)/180;
    dReal tail_link_2_theta = (150*M_PI)/180;
    dReal tail_link_3_theta = (180*M_PI)/180;
    dReal tail_link_4_theta = (150*M_PI)/180;

    dReal tail_link_1_length = 60.0;
    dReal tail_link_2_length = 60.0;
    dReal tail_link_3_length = 60.0;
    dReal tail_link_4_length = 60.0;

    dReal start_location[3];
    start_location[0] = root_position[0] + back_link_1_length + back_link_2_length + back_link_3_length + back_link_4_length*sin(back_link_4_theta) + back_link_5_length + back_link_6_length*sin(back_link_6_theta);
    start_location[1] = root_position[1] + back_link_4_length*cos(back_link_4_theta) + back_link_6_length*cos(back_link_6_theta);
    start_location[2] = root_position[2];

    //Link 1    
    dReal tail_link_1_position[3];
    tail_link_1_position[0] = start_location[0] + (tail_link_1_length/2)*sin(tail_link_1_theta);
    tail_link_1_position[1] = start_location[1] + (tail_link_1_length/2)*cos(tail_link_1_theta);
    tail_link_1_position[2] = start_location[2];

    setLink(&tail_link_1_body, &tail_link_1_mass, tail_link_1_length, tail_link_1_theta, &tail_link_1_geom, tail_link_1_position);

    //Link 2    
    dReal tail_link_2_position[3];
    tail_link_2_position[0] = start_location[0] + tail_link_1_length*sin(tail_link_1_theta) + (tail_link_2_length/2)*sin(tail_link_2_theta);
    tail_link_2_position[1] = start_location[1] + tail_link_1_length*cos(tail_link_1_theta) + (tail_link_2_length/2)*cos(tail_link_2_theta); 
    tail_link_2_position[2] = start_location[2];

    setLink(&tail_link_2_body, &tail_link_2_mass, tail_link_2_length, tail_link_2_theta, &tail_link_2_geom, tail_link_2_position);

    //Link 3  
    dReal tail_link_3_position[3];
    tail_link_3_position[0] = start_location[0] + tail_link_1_length*sin(tail_link_1_theta) + tail_link_2_length*sin(tail_link_2_theta) + (tail_link_3_length/2)*sin(tail_link_3_theta);
    tail_link_3_position[1] = start_location[1] + tail_link_1_length*cos(tail_link_1_theta) + tail_link_2_length*cos(tail_link_2_theta) + (tail_link_3_length/2)*cos(tail_link_3_theta); 
    tail_link_3_position[2] = start_location[2];

    setLink(&tail_link_3_body, &tail_link_3_mass, tail_link_3_length, tail_link_3_theta, &tail_link_3_geom, tail_link_3_position);

    //Link 4    
    dReal tail_link_4_position[3];
    tail_link_4_position[0] = start_location[0] + tail_link_1_length*sin(tail_link_1_theta) + tail_link_2_length*sin(tail_link_2_theta) + tail_link_3_length*sin(tail_link_3_theta) + (tail_link_4_length/2)*sin(tail_link_4_theta);
    tail_link_4_position[1] = start_location[1] + tail_link_1_length*cos(tail_link_1_theta) + tail_link_2_length*cos(tail_link_2_theta) + tail_link_3_length*cos(tail_link_3_theta) + (tail_link_4_length/2)*cos(tail_link_4_theta); 
    tail_link_4_position[2] = start_location[2];

    setLink(&tail_link_4_body, &tail_link_4_mass, tail_link_4_length, tail_link_4_theta, &tail_link_4_geom, tail_link_4_position);

    //Joints
    //Back and link 1
    dReal tail_joint_back_6_tail_1_position[3];
    tail_joint_back_6_tail_1_position[0] = start_location[0];
    tail_joint_back_6_tail_1_position[1] = start_location[1];
    tail_joint_back_6_tail_1_position[2] = start_location[2];

    dJointID tail_joint_back_6_tail_1 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(tail_joint_back_6_tail_1, back_link_6_body, tail_link_1_body);
    dJointSetBallAnchor (tail_joint_back_6_tail_1, tail_joint_back_6_tail_1_position[0], tail_joint_back_6_tail_1_position[1], tail_joint_back_6_tail_1_position[2]);

    //Link 1 and link 2
    dReal tail_joint_1_2_position[3];
    tail_joint_1_2_position[0] = start_location[0] + tail_link_1_length*sin(tail_link_1_theta);
    tail_joint_1_2_position[1] = start_location[1] + tail_link_1_length*cos(tail_link_1_theta);
    tail_joint_1_2_position[2] = start_location[2];

    dJointID tail_joint_1_2 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(tail_joint_1_2, tail_link_1_body, tail_link_2_body);
    dJointSetBallAnchor (tail_joint_1_2, tail_joint_1_2_position[0], tail_joint_1_2_position[1], tail_joint_1_2_position[2]);

    //Link 2 and link 3
    dReal tail_joint_2_3_position[3];
    tail_joint_2_3_position[0] = start_location[0] + tail_link_1_length*sin(tail_link_1_theta) + tail_link_2_length*sin(tail_link_2_theta);
    tail_joint_2_3_position[1] = start_location[1] + tail_link_1_length*cos(tail_link_1_theta) + tail_link_2_length*cos(tail_link_2_theta);
    tail_joint_2_3_position[2] = start_location[2];

    dJointID tail_joint_2_3 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(tail_joint_2_3, tail_link_2_body, tail_link_3_body);
    dJointSetBallAnchor (tail_joint_2_3, tail_joint_2_3_position[0], tail_joint_2_3_position[1], tail_joint_2_3_position[2]);

    //Link 3 and link 4
    dReal tail_joint_3_4_position[3];
    tail_joint_3_4_position[0] = start_location[0] + tail_link_1_length*sin(tail_link_1_theta) + tail_link_2_length*sin(tail_link_2_theta) + tail_link_3_length*sin(tail_link_3_theta);
    tail_joint_3_4_position[1] = start_location[1] + tail_link_1_length*cos(tail_link_1_theta) + tail_link_2_length*cos(tail_link_2_theta) + tail_link_3_length*cos(tail_link_3_theta);
    tail_joint_3_4_position[2] = start_location[2];

    dJointID tail_joint_3_4 = dJointCreateBall(global_helper->getWorld(), 0);
    dJointAttach(tail_joint_3_4, tail_link_3_body, tail_link_4_body);
    dJointSetBallAnchor (tail_joint_3_4, tail_joint_3_4_position[0], tail_joint_3_4_position[1], tail_joint_3_4_position[2]);
}

void ODEBodies::set_leg(){
    set_front_legs();
    set_back_legs();
}

void ODEBodies::set_front_legs(){
    front_foot_link_1_theta = (0*M_PI)/180;
    front_foot_link_2_theta = (45*M_PI)/180;
    front_foot_link_3_theta = (30*M_PI)/180;
    front_foot_link_4_theta = (90*M_PI)/180;
    dReal length_multiplier = 462/(5 + 4*abs(cos(front_foot_link_2_theta)) + 3*abs(cos(front_foot_link_3_theta)));    
    front_foot_link_1_length = 5*length_multiplier;
    front_foot_link_2_length = 4*length_multiplier;
    front_foot_link_3_length = 3*length_multiplier;
    front_foot_link_4_length = 40;
    
    dReal start_location[3];
    start_location[0] = root_position[0];
    start_location[1] = root_position[1] - front_foot_link_1_length/2;
    start_location[2] = root_position[2];

    /***** Left Leg ******/
    //Link 1    
    dReal front_left_foot_link_1_position[3];
    front_left_foot_link_1_position[0] = start_location[0];
    front_left_foot_link_1_position[1] = start_location[1];
    front_left_foot_link_1_position[2] = start_location[2] + 120;

    setLink(&front_left_foot_link_1_body, &front_left_foot_link_1_mass, front_foot_link_1_length, front_foot_link_1_theta, &front_left_foot_link_1_geom, front_left_foot_link_1_position);

    //Link 2    
    dReal front_left_foot_link_2_position[3];
    front_left_foot_link_2_position[0] = start_location[0] - abs((front_foot_link_2_length/2)*sin(front_foot_link_2_theta));
    front_left_foot_link_2_position[1] = start_location[1] - front_foot_link_1_length/2 - abs((front_foot_link_2_length/2)*cos(front_foot_link_2_theta));
    front_left_foot_link_2_position[2] = start_location[2] + 120;

    setLink(&front_left_foot_link_2_body, &front_left_foot_link_2_mass, front_foot_link_2_length, front_foot_link_2_theta, &front_left_foot_link_2_geom, front_left_foot_link_2_position);

    //Link 3
    dReal front_left_foot_link_3_position[3];
    front_left_foot_link_3_position[0] = start_location[0] - abs(front_foot_link_2_length*sin(front_foot_link_2_theta)) - abs((front_foot_link_3_length/2)*sin(front_foot_link_3_theta));
    front_left_foot_link_3_position[1] = start_location[1] - front_foot_link_1_length/2 - abs(front_foot_link_2_length*cos(front_foot_link_2_theta)) - abs((front_foot_link_3_length/2)*cos(front_foot_link_3_theta));
    front_left_foot_link_3_position[2] = start_location[2] + 120;

    setLink(&front_left_foot_link_3_body, &front_left_foot_link_3_mass, front_foot_link_3_length, front_foot_link_3_theta, &front_left_foot_link_3_geom, front_left_foot_link_3_position);

    //Link 4
    dReal front_left_foot_link_4_position[3];
    front_left_foot_link_4_position[0] = start_location[0] - abs(front_foot_link_2_length*sin(front_foot_link_2_theta)) - abs(front_foot_link_3_length*sin(front_foot_link_3_theta)) - front_foot_link_4_length/2;
    front_left_foot_link_4_position[1] = start_location[1] - front_foot_link_1_length/2 - abs(front_foot_link_2_length*cos(front_foot_link_2_theta)) - abs(front_foot_link_3_length*cos(front_foot_link_3_theta)) - 2;
    front_left_foot_link_4_position[2] = start_location[2] + 120;

    setLink(&front_left_foot_link_4_body, &front_left_foot_link_4_mass, front_foot_link_4_length, front_foot_link_4_theta, &front_left_foot_link_4_geom, front_left_foot_link_4_position);

    //Joints
    //back and Link 1
    dReal front_left_foot_joint_back_1_left_1_position[3];
    front_left_foot_joint_back_1_left_1_position[0] = start_location[0];
    front_left_foot_joint_back_1_left_1_position[1] = start_location[1] + front_foot_link_1_length/2;
    front_left_foot_joint_back_1_left_1_position[2] = start_location[2] + 120;

    dJointID front_left_foot_joint_back_1_left_1 = dJointCreateUniversal(global_helper->getWorld(), 0);
    dJointAttach(front_left_foot_joint_back_1_left_1, back_link_1_body, front_left_foot_link_1_body);
    dJointSetUniversalAxis1(front_left_foot_joint_back_1_left_1, 1, 0, 0);
    dJointSetUniversalAxis2(front_left_foot_joint_back_1_left_1, 0, 1, 0);
    dJointSetUniversalAnchor(front_left_foot_joint_back_1_left_1, front_left_foot_joint_back_1_left_1_position[0], front_left_foot_joint_back_1_left_1_position[1], front_left_foot_joint_back_1_left_1_position[2]);

    //Link 1 and link 2
    dReal front_left_foot_joint_left_1_left_2_position[3];
    front_left_foot_joint_left_1_left_2_position[0] = start_location[0];
    front_left_foot_joint_left_1_left_2_position[1] = start_location[1] - front_foot_link_1_length/2;
    front_left_foot_joint_left_1_left_2_position[2] = start_location[2] + 120;

    dJointID front_left_foot_joint_left_1_left_2 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(front_left_foot_joint_left_1_left_2, front_left_foot_link_1_body, front_left_foot_link_2_body);
    dJointSetHingeAnchor(front_left_foot_joint_left_1_left_2, front_left_foot_joint_left_1_left_2_position[0], front_left_foot_joint_left_1_left_2_position[1], front_left_foot_joint_left_1_left_2_position[2]);
    dJointSetHingeAxis(front_left_foot_joint_left_1_left_2, 0, 0, 1);

    //Link 2 and link 3
    dReal front_left_foot_joint_left_2_left_3_position[3];
    front_left_foot_joint_left_2_left_3_position[0] = start_location[0] - abs(front_foot_link_2_length*sin(front_foot_link_2_theta));
    front_left_foot_joint_left_2_left_3_position[1] = start_location[1] - front_foot_link_1_length/2 - abs(front_foot_link_2_length*cos(front_foot_link_2_theta));
    front_left_foot_joint_left_2_left_3_position[2] = start_location[2] + 120;

    dJointID front_left_foot_joint_left_2_left_3 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(front_left_foot_joint_left_2_left_3, front_left_foot_link_2_body, front_left_foot_link_3_body);
    dJointSetHingeAnchor(front_left_foot_joint_left_2_left_3, front_left_foot_joint_left_2_left_3_position[0], front_left_foot_joint_left_2_left_3_position[1], front_left_foot_joint_left_2_left_3_position[2]);
    dJointSetHingeAxis(front_left_foot_joint_left_2_left_3, 0, 0, 1);

    //Link 3 and link 4
    dReal front_left_foot_joint_left_3_left_4_position[3];
    front_left_foot_joint_left_3_left_4_position[0] = start_location[0] - abs(front_foot_link_2_length*sin(front_foot_link_2_theta)) - abs(front_foot_link_3_length*sin(front_foot_link_3_theta));
    front_left_foot_joint_left_3_left_4_position[1] = start_location[1] - front_foot_link_1_length/2 - abs(front_foot_link_2_length*cos(front_foot_link_2_theta)) - abs(front_foot_link_3_length*cos(front_foot_link_3_theta));
    front_left_foot_joint_left_3_left_4_position[2] = start_location[2] + 120;

    dJointID front_left_foot_joint_left_3_left_4 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(front_left_foot_joint_left_3_left_4, front_left_foot_link_3_body, front_left_foot_link_4_body);
    dJointSetHingeAnchor(front_left_foot_joint_left_3_left_4, front_left_foot_joint_left_3_left_4_position[0], front_left_foot_joint_left_3_left_4_position[1], front_left_foot_joint_left_3_left_4_position[2]);
    dJointSetHingeAxis(front_left_foot_joint_left_3_left_4, 0, 0, 1);

    /***** Right Leg ******/
    //Link 1    
    dReal front_right_foot_link_1_position[3];
    front_right_foot_link_1_position[0] = start_location[0];
    front_right_foot_link_1_position[1] = start_location[1];
    front_right_foot_link_1_position[2] = start_location[2] - 120;

    setLink(&front_right_foot_link_1_body, &front_right_foot_link_1_mass, front_foot_link_1_length, front_foot_link_1_theta, &front_right_foot_link_1_geom, front_right_foot_link_1_position);

    //Link 2    
    dReal front_right_foot_link_2_position[3];
    front_right_foot_link_2_position[0] = start_location[0] - abs((front_foot_link_2_length/2)*sin(front_foot_link_2_theta));
    front_right_foot_link_2_position[1] = start_location[1] - front_foot_link_1_length/2 - abs((front_foot_link_2_length/2)*cos(front_foot_link_2_theta));
    front_right_foot_link_2_position[2] = start_location[2] - 120;

    setLink(&front_right_foot_link_2_body, &front_right_foot_link_2_mass, front_foot_link_2_length, front_foot_link_2_theta, &front_right_foot_link_2_geom, front_right_foot_link_2_position);

    //Link 3
    dReal front_right_foot_link_3_position[3];
    front_right_foot_link_3_position[0] = start_location[0] - abs(front_foot_link_2_length*sin(front_foot_link_2_theta)) - abs((front_foot_link_3_length/2)*sin(front_foot_link_3_theta));
    front_right_foot_link_3_position[1] = start_location[1] - front_foot_link_1_length/2 - abs(front_foot_link_2_length*cos(front_foot_link_2_theta)) - abs((front_foot_link_3_length/2)*cos(front_foot_link_3_theta));
    front_right_foot_link_3_position[2] = start_location[2] - 120;

    setLink(&front_right_foot_link_3_body, &front_right_foot_link_3_mass, front_foot_link_3_length, front_foot_link_3_theta, &front_right_foot_link_3_geom, front_right_foot_link_3_position);

    //Link 4
    dReal front_right_foot_link_4_position[3];
    front_right_foot_link_4_position[0] = start_location[0] - abs(front_foot_link_2_length*sin(front_foot_link_2_theta)) - abs(front_foot_link_3_length*sin(front_foot_link_3_theta)) - front_foot_link_4_length/2;
    front_right_foot_link_4_position[1] = start_location[1] - front_foot_link_1_length/2 - abs(front_foot_link_2_length*cos(front_foot_link_2_theta)) - abs(front_foot_link_3_length*cos(front_foot_link_3_theta)) - 2;
    front_right_foot_link_4_position[2] = start_location[2] - 120;

    setLink(&front_right_foot_link_4_body, &front_right_foot_link_4_mass, front_foot_link_4_length, front_foot_link_4_theta, &front_right_foot_link_4_geom, front_right_foot_link_4_position);

    //Joints
    //back and Link 1
    dReal front_right_foot_joint_back_1_right_1_position[3];
    front_right_foot_joint_back_1_right_1_position[0] = start_location[0];
    front_right_foot_joint_back_1_right_1_position[1] = start_location[1] + front_foot_link_1_length/2;
    front_right_foot_joint_back_1_right_1_position[2] = start_location[2] - 120;

    dJointID front_right_foot_joint_back_1_right_1 = dJointCreateUniversal(global_helper->getWorld(), 0);
    dJointAttach(front_right_foot_joint_back_1_right_1, back_link_1_body, front_right_foot_link_1_body);
    dJointSetUniversalAxis1(front_right_foot_joint_back_1_right_1, 1, 0, 0);
    dJointSetUniversalAxis2(front_right_foot_joint_back_1_right_1, 0, 1, 0);
    dJointSetUniversalAnchor(front_right_foot_joint_back_1_right_1, front_right_foot_joint_back_1_right_1_position[0], front_right_foot_joint_back_1_right_1_position[1], front_right_foot_joint_back_1_right_1_position[2]);

    //Link 1 and link 2
    dReal front_right_foot_joint_right_1_right_2_position[3];
    front_right_foot_joint_right_1_right_2_position[0] = start_location[0];
    front_right_foot_joint_right_1_right_2_position[1] = start_location[1] - front_foot_link_1_length/2;
    front_right_foot_joint_right_1_right_2_position[2] = start_location[2] - 120;

    dJointID front_right_foot_joint_right_1_right_2 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(front_right_foot_joint_right_1_right_2, front_right_foot_link_1_body, front_right_foot_link_2_body);
    dJointSetHingeAnchor(front_right_foot_joint_right_1_right_2, front_right_foot_joint_right_1_right_2_position[0], front_right_foot_joint_right_1_right_2_position[1], front_right_foot_joint_right_1_right_2_position[2]);
    dJointSetHingeAxis(front_right_foot_joint_right_1_right_2, 0, 0, 1);

    //Link 2 and link 3
    dReal front_right_foot_joint_right_2_right_3_position[3];
    front_right_foot_joint_right_2_right_3_position[0] = start_location[0] - abs(front_foot_link_2_length*sin(front_foot_link_2_theta));
    front_right_foot_joint_right_2_right_3_position[1] = start_location[1] - front_foot_link_1_length/2 - abs(front_foot_link_2_length*cos(front_foot_link_2_theta));
    front_right_foot_joint_right_2_right_3_position[2] = start_location[2] - 120;

    dJointID front_right_foot_joint_right_2_right_3 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(front_right_foot_joint_right_2_right_3, front_right_foot_link_2_body, front_right_foot_link_3_body);
    dJointSetHingeAnchor(front_right_foot_joint_right_2_right_3, front_right_foot_joint_right_2_right_3_position[0], front_right_foot_joint_right_2_right_3_position[1], front_right_foot_joint_right_2_right_3_position[2]);
    dJointSetHingeAxis(front_right_foot_joint_right_2_right_3, 0, 0, 1);

    //Link 3 and link 4
    dReal front_right_foot_joint_right_3_right_4_position[3];
    front_right_foot_joint_right_3_right_4_position[0] = start_location[0] - abs(front_foot_link_2_length*sin(front_foot_link_2_theta)) - abs(front_foot_link_3_length*sin(front_foot_link_3_theta));
    front_right_foot_joint_right_3_right_4_position[1] = start_location[1] - front_foot_link_1_length/2 - abs(front_foot_link_2_length*cos(front_foot_link_2_theta)) - abs(front_foot_link_3_length*cos(front_foot_link_3_theta));
    front_right_foot_joint_right_3_right_4_position[2] = start_location[2] - 120;

    dJointID front_right_foot_joint_right_3_right_4 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(front_right_foot_joint_right_3_right_4, front_right_foot_link_3_body, front_right_foot_link_4_body);
    dJointSetHingeAnchor(front_right_foot_joint_right_3_right_4, front_right_foot_joint_right_3_right_4_position[0], front_right_foot_joint_right_3_right_4_position[1], front_right_foot_joint_right_3_right_4_position[2]);
    dJointSetHingeAxis(front_right_foot_joint_right_3_right_4, 0, 0, 1);
}

void ODEBodies::set_back_legs(){
    back_foot_link_1_theta = (0*M_PI)/180;
    back_foot_link_2_theta = (45*M_PI)/180;
    back_foot_link_3_theta = (30*M_PI)/180;
    back_foot_link_4_theta = (90*M_PI)/180;
    dReal length_multiplier = 446/(5 + 4*abs(sin(back_foot_link_2_theta)) + 3*abs(sin(back_foot_link_3_theta)));   
  
    back_foot_link_1_length = 5*length_multiplier;
    back_foot_link_2_length = 4*length_multiplier;
    back_foot_link_3_length = 3*length_multiplier;
    back_foot_link_4_length = 40;

    dReal start_location[3];
    start_location[0] = root_position[0] + back_link_1_length + back_link_2_length + back_link_3_length + back_link_4_length*sin(back_link_4_theta) + back_link_5_length + back_link_6_length*sin(back_link_6_theta) ;
    start_location[1] = root_position[1] + back_link_4_length*cos(back_link_4_theta) + back_link_6_length*cos(back_link_6_theta) - back_foot_link_1_length/2;
    start_location[2] = root_position[2];
    
    /***** Left Leg ******/
    //Link 1    
    dReal back_left_foot_link_1_position[3];
    back_left_foot_link_1_position[0] = start_location[0];
    back_left_foot_link_1_position[1] = start_location[1];
    back_left_foot_link_1_position[2] = start_location[2] + 120;

    setLink(&back_left_foot_link_1_body, &back_left_foot_link_1_mass, back_foot_link_1_length, back_foot_link_1_theta, &back_left_foot_link_1_geom, back_left_foot_link_1_position);

    
    //Link 2    
    dReal back_left_foot_link_2_position[3];
    back_left_foot_link_2_position[0] = start_location[0] - abs((back_foot_link_2_length/2)*sin(back_foot_link_2_theta));
    back_left_foot_link_2_position[1] = start_location[1] - back_foot_link_1_length/2 - abs((back_foot_link_2_length/2)*cos(back_foot_link_2_theta));
    back_left_foot_link_2_position[2] = start_location[2] + 120;

    setLink(&back_left_foot_link_2_body, &back_left_foot_link_2_mass, back_foot_link_2_length, back_foot_link_2_theta, &back_left_foot_link_2_geom, back_left_foot_link_2_position);
    
    //Link 3
    dReal back_left_foot_link_3_position[3];
    back_left_foot_link_3_position[0] = start_location[0] - abs(back_foot_link_2_length*sin(back_foot_link_2_theta)) - abs((back_foot_link_3_length/2)*sin(back_foot_link_3_theta));
    back_left_foot_link_3_position[1] = start_location[1] - back_foot_link_1_length/2 - abs(back_foot_link_2_length*cos(back_foot_link_2_theta)) - abs((back_foot_link_3_length/2)*cos(back_foot_link_3_theta));
    back_left_foot_link_3_position[2] = start_location[2] + 120;

    setLink(&back_left_foot_link_3_body, &back_left_foot_link_3_mass, back_foot_link_3_length, back_foot_link_3_theta, &back_left_foot_link_3_geom, back_left_foot_link_3_position);

    //Link 4
    dReal back_left_foot_link_4_position[3];
    back_left_foot_link_4_position[0] = start_location[0] - abs(back_foot_link_2_length*sin(back_foot_link_2_theta)) - abs(back_foot_link_3_length*sin(back_foot_link_3_theta)) - back_foot_link_4_length/2;
    back_left_foot_link_4_position[1] = start_location[1] - back_foot_link_1_length/2 - abs(back_foot_link_2_length*cos(back_foot_link_2_theta)) - abs(back_foot_link_3_length*cos(back_foot_link_3_theta)) - 2;
    back_left_foot_link_4_position[2] = start_location[2] + 120;

    setLink(&back_left_foot_link_4_body, &back_left_foot_link_4_mass, back_foot_link_4_length, back_foot_link_4_theta, &back_left_foot_link_4_geom, back_left_foot_link_4_position);

    //Joints
    //back and Link 1
    dReal back_left_foot_joint_back_6_left_1_position[3];
    back_left_foot_joint_back_6_left_1_position[0] = start_location[0];
    back_left_foot_joint_back_6_left_1_position[1] = start_location[1] + back_foot_link_1_length/2;
    back_left_foot_joint_back_6_left_1_position[2] = start_location[2] + 120;

    dJointID back_left_foot_joint_back_6_left_1 = dJointCreateUniversal(global_helper->getWorld(), 0);
    dJointAttach(back_left_foot_joint_back_6_left_1, back_link_6_body, back_left_foot_link_1_body);
    dJointSetUniversalAxis1(back_left_foot_joint_back_6_left_1, 1, 0, 0);
    dJointSetUniversalAxis2(back_left_foot_joint_back_6_left_1, 0, 1, 0);
    dJointSetUniversalAnchor(back_left_foot_joint_back_6_left_1, back_left_foot_joint_back_6_left_1_position[0], back_left_foot_joint_back_6_left_1_position[1], back_left_foot_joint_back_6_left_1_position[2]);

    //Link 1 and link 2
    dReal back_left_foot_joint_left_1_left_2_position[3];
    back_left_foot_joint_left_1_left_2_position[0] = start_location[0];
    back_left_foot_joint_left_1_left_2_position[1] = start_location[1] - back_foot_link_1_length/2;
    back_left_foot_joint_left_1_left_2_position[2] = start_location[2] + 120;

    dJointID back_left_foot_joint_left_1_left_2 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(back_left_foot_joint_left_1_left_2, back_left_foot_link_1_body, back_left_foot_link_2_body);
    dJointSetHingeAnchor(back_left_foot_joint_left_1_left_2, back_left_foot_joint_left_1_left_2_position[0], back_left_foot_joint_left_1_left_2_position[1], back_left_foot_joint_left_1_left_2_position[2]);
    dJointSetHingeAxis(back_left_foot_joint_left_1_left_2, 0, 0, 1);

    //Link 2 and link 3
    dReal back_left_foot_joint_left_2_left_3_position[3];
    back_left_foot_joint_left_2_left_3_position[0] = start_location[0] - abs(back_foot_link_2_length*sin(back_foot_link_2_theta));
    back_left_foot_joint_left_2_left_3_position[1] = start_location[1] - back_foot_link_1_length/2 - abs(back_foot_link_2_length*cos(back_foot_link_2_theta));
    back_left_foot_joint_left_2_left_3_position[2] = start_location[2] + 120;

    dJointID back_left_foot_joint_left_2_left_3 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(back_left_foot_joint_left_2_left_3, back_left_foot_link_2_body, back_left_foot_link_3_body);
    dJointSetHingeAnchor(back_left_foot_joint_left_2_left_3, back_left_foot_joint_left_2_left_3_position[0], back_left_foot_joint_left_2_left_3_position[1], back_left_foot_joint_left_2_left_3_position[2]);
    dJointSetHingeAxis(back_left_foot_joint_left_2_left_3, 0, 0, 1);

    //Link 3 and link 4
    dReal back_left_foot_joint_left_3_left_4_position[3];
    back_left_foot_joint_left_3_left_4_position[0] = start_location[0] - abs(back_foot_link_2_length*sin(back_foot_link_2_theta)) - abs(back_foot_link_3_length*sin(back_foot_link_3_theta));
    back_left_foot_joint_left_3_left_4_position[1] = start_location[1] - back_foot_link_1_length/2 - abs(back_foot_link_2_length*cos(back_foot_link_2_theta)) - abs(back_foot_link_3_length*cos(back_foot_link_3_theta));
    back_left_foot_joint_left_3_left_4_position[2] = start_location[2] + 120;

    dJointID back_left_foot_joint_left_3_left_4 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(back_left_foot_joint_left_3_left_4, back_left_foot_link_3_body, back_left_foot_link_4_body);
    dJointSetHingeAnchor(back_left_foot_joint_left_3_left_4, back_left_foot_joint_left_3_left_4_position[0], back_left_foot_joint_left_3_left_4_position[1], back_left_foot_joint_left_3_left_4_position[2]);
    dJointSetHingeAxis(back_left_foot_joint_left_3_left_4, 0, 0, 1);

    
    /***** Right Leg ******/
    //Link 1    
    dReal back_right_foot_link_1_position[3];
    back_right_foot_link_1_position[0] = start_location[0];
    back_right_foot_link_1_position[1] = start_location[1];
    back_right_foot_link_1_position[2] = start_location[2] - 120;

    setLink(&back_right_foot_link_1_body, &back_right_foot_link_1_mass, back_foot_link_1_length, back_foot_link_1_theta, &back_right_foot_link_1_geom, back_right_foot_link_1_position);

    //Link 2    
    dReal back_right_foot_link_2_position[3];
    back_right_foot_link_2_position[0] = start_location[0] - abs((back_foot_link_2_length/2)*sin(back_foot_link_2_theta));
    back_right_foot_link_2_position[1] = start_location[1] - back_foot_link_1_length/2 - abs((back_foot_link_2_length/2)*cos(back_foot_link_2_theta));
    back_right_foot_link_2_position[2] = start_location[2] - 120;

    setLink(&back_right_foot_link_2_body, &back_right_foot_link_2_mass, back_foot_link_2_length, back_foot_link_2_theta, &back_right_foot_link_2_geom, back_right_foot_link_2_position);

    //Link 3
    dReal back_right_foot_link_3_position[3];
    back_right_foot_link_3_position[0] = start_location[0] - abs(back_foot_link_2_length*sin(back_foot_link_2_theta)) - abs((back_foot_link_3_length/2)*sin(back_foot_link_3_theta));
    back_right_foot_link_3_position[1] = start_location[1] - back_foot_link_1_length/2 - abs(back_foot_link_2_length*cos(back_foot_link_2_theta)) - abs((back_foot_link_3_length/2)*cos(back_foot_link_3_theta));
    back_right_foot_link_3_position[2] = start_location[2] - 120;

    setLink(&back_right_foot_link_3_body, &back_right_foot_link_3_mass, back_foot_link_3_length, back_foot_link_3_theta, &back_right_foot_link_3_geom, back_right_foot_link_3_position);

    //Link 4
    dReal back_right_foot_link_4_position[3];
    back_right_foot_link_4_position[0] = start_location[0] - abs(back_foot_link_2_length*sin(back_foot_link_2_theta)) - abs(back_foot_link_3_length*sin(back_foot_link_3_theta)) - back_foot_link_4_length/2;
    back_right_foot_link_4_position[1] = start_location[1] - back_foot_link_1_length/2 - abs(back_foot_link_2_length*cos(back_foot_link_2_theta)) - abs(back_foot_link_3_length*cos(back_foot_link_3_theta)) - 2;
    back_right_foot_link_4_position[2] = start_location[2] - 120;

    setLink(&back_right_foot_link_4_body, &back_right_foot_link_4_mass, back_foot_link_4_length, back_foot_link_4_theta, &back_right_foot_link_4_geom, back_right_foot_link_4_position);

    //Joints
    //back and Link 1
    dReal back_right_foot_joint_back_6_right_1_position[3];
    back_right_foot_joint_back_6_right_1_position[0] = start_location[0];
    back_right_foot_joint_back_6_right_1_position[1] = start_location[1] + back_foot_link_1_length/2;
    back_right_foot_joint_back_6_right_1_position[2] = start_location[2] - 120;

    dJointID back_right_foot_joint_back_6_right_1 = dJointCreateUniversal(global_helper->getWorld(), 0);
    dJointAttach(back_right_foot_joint_back_6_right_1, back_link_6_body, back_right_foot_link_1_body);
    dJointSetUniversalAxis1(back_right_foot_joint_back_6_right_1, 1, 0, 0);
    dJointSetUniversalAxis2(back_right_foot_joint_back_6_right_1, 0, 1, 0);
    dJointSetUniversalAnchor(back_right_foot_joint_back_6_right_1, back_right_foot_joint_back_6_right_1_position[0], back_right_foot_joint_back_6_right_1_position[1], back_right_foot_joint_back_6_right_1_position[2]);

    //Link 1 and link 2
    dReal back_right_foot_joint_right_1_right_2_position[3];
    back_right_foot_joint_right_1_right_2_position[0] = start_location[0];
    back_right_foot_joint_right_1_right_2_position[1] = start_location[1] - back_foot_link_1_length/2;
    back_right_foot_joint_right_1_right_2_position[2] = start_location[2] - 120;

    dJointID back_right_foot_joint_right_1_right_2 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(back_right_foot_joint_right_1_right_2, back_right_foot_link_1_body, back_right_foot_link_2_body);
    dJointSetHingeAnchor(back_right_foot_joint_right_1_right_2, back_right_foot_joint_right_1_right_2_position[0], back_right_foot_joint_right_1_right_2_position[1], back_right_foot_joint_right_1_right_2_position[2]);
    dJointSetHingeAxis(back_right_foot_joint_right_1_right_2, 0, 0, 1);

    //Link 2 and link 3
    dReal back_right_foot_joint_right_2_right_3_position[3];
    back_right_foot_joint_right_2_right_3_position[0] = start_location[0] - abs(back_foot_link_2_length*sin(back_foot_link_2_theta));
    back_right_foot_joint_right_2_right_3_position[1] = start_location[1] - back_foot_link_1_length/2 - abs(back_foot_link_2_length*cos(back_foot_link_2_theta));
    back_right_foot_joint_right_2_right_3_position[2] = start_location[2] - 120;

    dJointID back_right_foot_joint_right_2_right_3 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(back_right_foot_joint_right_2_right_3, back_right_foot_link_2_body, back_right_foot_link_3_body);
    dJointSetHingeAnchor(back_right_foot_joint_right_2_right_3, back_right_foot_joint_right_2_right_3_position[0], back_right_foot_joint_right_2_right_3_position[1], back_right_foot_joint_right_2_right_3_position[2]);
    dJointSetHingeAxis(back_right_foot_joint_right_2_right_3, 0, 0, 1);

    //Link 3 and link 4
    dReal back_right_foot_joint_right_3_right_4_position[3];
    back_right_foot_joint_right_3_right_4_position[0] = start_location[0] - abs(back_foot_link_2_length*sin(back_foot_link_2_theta)) - abs(back_foot_link_3_length*sin(back_foot_link_3_theta));
    back_right_foot_joint_right_3_right_4_position[1] = start_location[1] - back_foot_link_1_length/2 - abs(back_foot_link_2_length*cos(back_foot_link_2_theta)) - abs(back_foot_link_3_length*cos(back_foot_link_3_theta));
    back_right_foot_joint_right_3_right_4_position[2] = start_location[2] - 120;

    dJointID back_right_foot_joint_right_3_right_4 = dJointCreateHinge(global_helper->getWorld(), 0);
    dJointAttach(back_right_foot_joint_right_3_right_4, back_right_foot_link_3_body, back_right_foot_link_4_body);
    dJointSetHingeAnchor(back_right_foot_joint_right_3_right_4, back_right_foot_joint_right_3_right_4_position[0], back_right_foot_joint_right_3_right_4_position[1], back_right_foot_joint_right_3_right_4_position[2]);
    dJointSetHingeAxis(back_right_foot_joint_right_3_right_4, 0, 0, 1);
}

void ODEBodies::set_plane(){
	plane_geom = dCreatePlane(global_helper->getSpace(), 0.0, 1.0, 0.0, -250.0);	
}

helper * ODEBodies::getGlobalHelper(){
    return global_helper;
}

dBodyID ODEBodies::getBallBody(){
	return ball_body;
}

dBodyID ODEBodies::getBackLink1Body(){
    return back_link_1_body;
}

dBodyID ODEBodies::getBackLink2Body(){
    return back_link_2_body;
}

dBodyID ODEBodies::getBackLink3Body(){
    return back_link_3_body;
}

dBodyID ODEBodies::getBackLink4Body(){
    return back_link_4_body;
}

dBodyID ODEBodies::getBackLink5Body(){
    return back_link_5_body;
}

dBodyID ODEBodies::getBackLink6Body(){
    return back_link_6_body;
}

dBodyID ODEBodies::getNnhLink1Body(){
    return nnh_link_1_body;
}

dBodyID ODEBodies::getNnhLink2Body(){
    return nnh_link_2_body;
}

dBodyID ODEBodies::getNnhLink3Body(){
    return nnh_link_3_body;
}

dBodyID ODEBodies::getNnhLink4Body(){
    return nnh_link_4_body;
}

dBodyID ODEBodies::getTailLink1Body(){
    return tail_link_1_body;
}

dBodyID ODEBodies::getTailLink2Body(){
    return tail_link_2_body;
}

dBodyID ODEBodies::getTailLink3Body(){
    return tail_link_3_body;
}

dBodyID ODEBodies::getTailLink4Body(){
    return tail_link_4_body;
}

dBodyID ODEBodies::getFrontLeftFootLink1Body(){
    return front_left_foot_link_1_body;
}

dBodyID ODEBodies::getFrontLeftFootLink2Body(){
    return front_left_foot_link_2_body;
}

dBodyID ODEBodies::getFrontLeftFootLink3Body(){
    return front_left_foot_link_3_body;
}

dBodyID ODEBodies::getFrontLeftFootLink4Body(){
    return front_left_foot_link_4_body;
}

dBodyID ODEBodies::getFrontRightFootLink1Body(){
    return front_right_foot_link_1_body;
}

dBodyID ODEBodies::getFrontRightFootLink2Body(){
    return front_right_foot_link_2_body;
}

dBodyID ODEBodies::getFrontRightFootLink3Body(){
    return front_right_foot_link_3_body;
}

dBodyID ODEBodies::getFrontRightFootLink4Body(){
    return front_right_foot_link_4_body;
}

dReal ODEBodies::getFrontFootLink1Length(){
    return front_foot_link_1_length;
}

dReal ODEBodies::getFrontFootLink2Length(){
    return front_foot_link_2_length;
}

dReal ODEBodies::getFrontFootLink3Length(){
    return front_foot_link_3_length;
}

dReal ODEBodies::getFrontFootLink4Length(){
    return front_foot_link_4_length;
}

dBodyID ODEBodies::getBackLeftFootLink1Body(){
    return back_left_foot_link_1_body;
}

dBodyID ODEBodies::getBackLeftFootLink2Body(){
    return back_left_foot_link_2_body;
}

dBodyID ODEBodies::getBackLeftFootLink3Body(){
    return back_left_foot_link_3_body;
}

dBodyID ODEBodies::getBackLeftFootLink4Body(){
    return back_left_foot_link_4_body;
}

dBodyID ODEBodies::getBackRightFootLink1Body(){
    return back_right_foot_link_1_body;
}

dBodyID ODEBodies::getBackRightFootLink2Body(){
    return back_right_foot_link_2_body;
}

dBodyID ODEBodies::getBackRightFootLink3Body(){
    return back_right_foot_link_3_body;
}

dBodyID ODEBodies::getBackRightFootLink4Body(){
    return back_right_foot_link_4_body;
}

dReal ODEBodies::getBackFootLink1Length(){
    return back_foot_link_1_length;
}

dReal ODEBodies::getBackFootLink2Length(){
    return back_foot_link_2_length;
}

dReal ODEBodies::getBackFootLink3Length(){
    return back_foot_link_3_length;
}

dReal ODEBodies::getBackFootLink4Length(){
    return back_foot_link_4_length;
}

dReal ODEBodies::getBackLink1Length(){
    return back_link_1_length;
}

dReal ODEBodies::getBackLink2Length(){
    return back_link_2_length;
}

dReal ODEBodies::getBackLink3Length(){
    return back_link_3_length;
}

dReal ODEBodies::getBackLink4Length(){
    return back_link_4_length;
}

dReal ODEBodies::getBackLink5Length(){
    return back_link_5_length;
}

dReal ODEBodies::getBackLink6Length(){
    return back_link_6_length;
}

dGeomID ODEBodies::getPlaneGeom(){
	return plane_geom;
}




