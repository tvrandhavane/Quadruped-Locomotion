#include "inverseKinematics.h"

inverseKinematics::inverseKinematics(vector<float> lengths, vector<Eigen::MatrixXf> transformationMatrices, vector<float> angles, vector<float> targetPosition, Eigen::MatrixXf translationMatrix, Eigen::MatrixXf axis){
	this->angles = angles;
	this->lengths = lengths;
	numLinks = lengths.size();
	jacobian = Eigen::MatrixXf::Random(6, numLinks);

	Eigen::MatrixXf Pn = Eigen::MatrixXf::Random(4, 1);
	Pn(0, 0) = targetPosition[0];
	Pn(1, 0) = targetPosition[1];
	Pn(2, 0) = targetPosition[2];
	Pn(3, 0) = 1;
	createJacobian(Pn, transformationMatrices, translationMatrix, axis);
	invertJacobian();
	computeJointAngleChange(Pn);	
}

Eigen::MatrixXf inverseKinematics::crossProduct(Eigen::MatrixXf m1, Eigen::MatrixXf m2){
	Eigen::MatrixXf cross = Eigen::MatrixXf::Random(3, 1);
	cross(0, 0) = m1(1, 0)*m2(2, 0) - m1(2, 0)*m2(1, 0);
	cross(1, 0) = m1(2, 0)*m2(0, 0) - m1(0, 0)*m2(2, 0);
	cross(2, 0) = m1(0, 0)*m2(1, 0) - m1(1, 0)*m2(0, 0);
	return cross;
}

void inverseKinematics::createJacobian(Eigen::MatrixXf Pn, vector<Eigen::MatrixXf> transformationMatrices, Eigen::MatrixXf translationMatrix, Eigen::MatrixXf axis){

	Eigen::MatrixXf Tj = translationMatrix;
	Tj = transformationMatrices[0]*Tj;

	Eigen::MatrixXf Zj = Eigen::MatrixXf::Random(4, 1);
	Zj(0, 0) = 0;
	Zj(0, 0) = 0;
	Zj(2, 0) = 1;
	Zj(3, 0) = 1;

	Eigen::MatrixXf origin = Eigen::MatrixXf::Random(4, 1);
	origin(0, 0) = 0;
	origin(0, 0) = 0;
	origin(2, 0) = 0;
	origin(3, 0) = 1;

	Eigen::MatrixXf Pj = Tj*origin;
	Eigen::MatrixXf TjZj = Tj*axis;
	Eigen::MatrixXf cross = crossProduct(TjZj, Pn - Pj);

	jacobian(0, 0) = cross(0, 0);
	jacobian(1, 0) = cross(1, 0);
	jacobian(2, 0) = cross(2, 0);
	jacobian(3, 0) = TjZj(0, 0);
	jacobian(4, 0) = TjZj(1, 0);
	jacobian(5, 0) = TjZj(2, 0);


	for(int i = 1; i < numLinks; i++){
		Eigen::MatrixXf Tr = Eigen::MatrixXf::Identity(4, 4);
	    Tr(0, 3) = lengths[i-1];

		Tj = Tj*transformationMatrices[i]*Tr;
		Zj = transformationMatrices[i]*Tr*Zj;

		Eigen::MatrixXf Pj = Tj*origin;
		Eigen::MatrixXf TjZj = Tj*Zj;
		Eigen::MatrixXf cross = crossProduct(TjZj, Pn - Pj);

		jacobian(0, i) = cross(0, 0);
		jacobian(1, i) = cross(1, 0);
		jacobian(2, i) = cross(2, 0);
		jacobian(3, i) = TjZj(0, 0);
		jacobian(4, i) = TjZj(1, 0);
		jacobian(5, i) = TjZj(2, 0);
	}

	Eigen::MatrixXf Tr = Eigen::MatrixXf::Identity(4, 4);
	Tr(0, 3) = lengths[numLinks-1];
	Tj = Tj*Tr;
	endEffector = Tj*origin;
}

void inverseKinematics::invertJacobian(){
	float lambda = 0.0;
	Eigen::MatrixXf m = Eigen::MatrixXf::Random(jacobian.rows(),jacobian.cols());

	for (int i=0; i<jacobian.rows(); i++) {
	    for (int j=0; j<jacobian.cols(); j++) {
	      	m(i, j) = jacobian(i,j);
	    }
	}
 
	Eigen::MatrixXf S = Eigen::MatrixXf::Zero(jacobian.rows(), jacobian.cols());

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeFullU | Eigen::ComputeThinV);

	for (int i=0; i< svd.singularValues().size(); i++) {
		lambda += svd.singularValues()(i);
	    S(i, i) = svd.singularValues()(i);
	}	
	lambda /= svd.singularValues().size();

	inverseJacobian = Eigen::MatrixXf::Zero(numLinks, 6);
	for(int i = 0; i < svd.singularValues().size(); i++){
		Eigen::MatrixXf vi = Eigen::MatrixXf::Zero(svd.matrixV().rows(), 1);
		for(int j = 0; j < svd.matrixV().rows(); j++){
			vi(j, 0) = svd.matrixV()(j, i);
		}
		Eigen::MatrixXf ui = Eigen::MatrixXf::Zero(1, svd.matrixU().rows());
		for(int j = 0; j < svd.matrixU().rows(); j++){
			ui(0, j) = svd.matrixU()(j, i);
		}
		Eigen::MatrixXf temp = Eigen::MatrixXf::Zero(numLinks, 6);
		temp = vi*ui;
		for (int k=0; k<temp.rows(); k++) {
		    for (int j=0; j<temp.cols(); j++) {
		      	temp(k, j) = (S(i,i)/(S(i, i)*S(i, i) + lambda*lambda))*temp(k, j);
		    }
		}
		inverseJacobian = inverseJacobian + temp;
	}
}

void inverseKinematics::computeJointAngleChange(Eigen::MatrixXf target_pos){
	Eigen::MatrixXf x = Eigen::MatrixXf::Zero(6, 1);
	Eigen::MatrixXf temp = target_pos - endEffector;
	x(0, 0) = temp(0, 0);
	x(1, 0) = temp(1, 0);
	x(2, 0) = temp(2, 0);
	jointAngleChange = inverseJacobian*x;
}

Eigen::MatrixXf inverseKinematics::getJointAngleChange(){
	return jointAngleChange;
}