#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <iostream>
#include <cmath>
#include <vector>
#include "QSMatrix.h"

using namespace std;

class inverseKinematics{
private:
	int numLinks;
	vector<float> lengths;
	vector<float> angles;
	QSMatrix<float> jacobian;

	QSMatrix<float> getTransformationMatrix(float l, float theta);
	QSMatrix<float> crossProduct(QSMatrix<float> m1, QSMatrix<float> m2);

public:
	inverseKinematics(vector<float> lengths, vector<float> angles, vector<float> endEffector);
	void createJacobian(QSMatrix<float> Pn);
	void invertJacobian();
};
#endif // INVERSE_KINEMATICS_H