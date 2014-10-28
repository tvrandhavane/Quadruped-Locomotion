#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <iostream>
#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

using namespace std;

class inverseKinematics{
private:
	int numLinks;
	vector<float> lengths;
	vector<float> angles;
	Eigen::MatrixXf jacobian;
	Eigen::MatrixXf inverseJacobian;
	Eigen::MatrixXf jointAngleChange;
	Eigen::MatrixXf endEffector;

	Eigen::MatrixXf crossProduct(Eigen::MatrixXf m1, Eigen::MatrixXf m2);

public:
	inverseKinematics(vector<float> lengths, vector<Eigen::MatrixXf> transformationMatrices, vector<float> angles, vector<float> targetPosition, Eigen::MatrixXf translationMatrix, Eigen::MatrixXf axis);
	void createJacobian(Eigen::MatrixXf Pn, vector<Eigen::MatrixXf> transformationMatrices, Eigen::MatrixXf translationMatrix, Eigen::MatrixXf axis);
	void invertJacobian();
	void computeJointAngleChange(Eigen::MatrixXf target_pos);
	Eigen::MatrixXf getJointAngleChange();
};
#endif // INVERSE_KINEMATICS_H