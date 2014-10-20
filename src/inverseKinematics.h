#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <iostream>
#include <cmath>
#include <vector>
#include "QSMatrix.h"
#include <iostream>
#include <Eigen/Dense>

using namespace std;

class inverseKinematics{
private:
	int numLinks;
	vector<float> lengths;
	vector<float> angles;
	QSMatrix<float> jacobian;
	QSMatrix<float> inverseJacobian;
	QSMatrix<float> jointAngleChange;
	QSMatrix<float> endEffector;

	QSMatrix<float> crossProduct(QSMatrix<float> m1, QSMatrix<float> m2);

public:
	QSMatrix<float> getTransformationMatrix(float l, float theta);
	inverseKinematics(vector<float> lengths, vector<float> angles, vector<float> targetPosition, QSMatrix<float> transformationMatrix, QSMatrix<float> axis);
	void createJacobian(QSMatrix<float> Pn, QSMatrix<float> T0, QSMatrix<float> Z0);
	void invertJacobian();
	void computeJointAngleChange(QSMatrix<float> target_pos);
};
#endif // INVERSE_KINEMATICS_H