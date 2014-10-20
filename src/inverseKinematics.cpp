#include "inverseKinematics.h"

inverseKinematics::inverseKinematics(vector<float> lengths, vector<float> angles, vector<float> targetPosition, QSMatrix<float> transformationMatrix, QSMatrix<float> axis){
	this->angles = angles;
	this->lengths = lengths;
	numLinks = lengths.size();
	jacobian = QSMatrix<float> (6, numLinks, 0.0);

	QSMatrix<float> Pn(4, 1, 1.0);
	Pn(0, 0) = targetPosition[0];
	Pn(1, 0) = targetPosition[1];
	Pn(2, 0) = targetPosition[2];
	createJacobian(Pn, transformationMatrix, axis);
	cout << "endEffector = " << endl;
	for (int i=0; i<endEffector.get_rows(); i++) {
	    for (int j=0; j<endEffector.get_cols(); j++) {
	      	cout << endEffector(i, j) << " ";
	    }
	    cout << endl;
	}
	/*
	cout << "targetPosition = " << endl;
	for (int i=0; i<targetPosition.size(); i++) {
	    cout << targetPosition[i] << " ";
	}
	invertJacobian();
	Pn(0, 0) = endEffector(0,0);
	Pn(1, 0) = endEffector(1,0) + 1.0;
	Pn(2, 0) = endEffector(2,0);
	computeJointAngleChange(Pn);
	//computeJointAngleChange(endEffector);
	cout << "jointAngleChange = " << endl;
	for (int i=0; i<jointAngleChange.get_rows(); i++) {
	    for (int j=0; j<jointAngleChange.get_cols(); j++) {
	      	cout << jointAngleChange(i, j) << " ";
	    }
	    cout << endl;
	}*/
}

QSMatrix<float> inverseKinematics::getTransformationMatrix(float l, float theta){
	QSMatrix<float> * matrix = new QSMatrix<float> (4, 4, 0.0);
	for(int i = 0; i < matrix->get_rows(); i++){
	    (*matrix)(i,i) = 1;
	}
	(*matrix)(0, 3) = l;

	QSMatrix<float> * matrix2 = new QSMatrix<float> (4, 4, 0.0);
	(*matrix2)(0, 0) = cos(theta);
	(*matrix2)(0, 1) = -sin(theta);
	(*matrix2)(1, 0) = sin(theta);
	(*matrix2)(1, 1) = cos(theta);
	(*matrix2)(2, 2) = 1;
	(*matrix2)(3, 3) = 1;

	QSMatrix<float> matrix3 = (*matrix) * (*matrix2);

	return matrix3;
}

QSMatrix<float> inverseKinematics::crossProduct(QSMatrix<float> m1, QSMatrix<float> m2){
	QSMatrix<float> cross = QSMatrix<float> (3, 1, 0.0);
	cross(0, 0) = m1(1, 0)*m2(2, 0) - m1(2, 0)*m2(1, 0);
	cross(1, 0) = m1(2, 0)*m2(0, 0) - m1(0, 0)*m2(2, 0);
	cross(2, 0) = m1(0, 0)*m2(1, 0) - m1(1, 0)*m2(0, 0);
	return cross;
}

void inverseKinematics::createJacobian(QSMatrix<float> Pn, QSMatrix<float> T0, QSMatrix<float> Z0){
	QSMatrix<float> Tj(4, 4, 0.0);

	QSMatrix<float> Zj(4, 1, 0.0);
	Zj(2, 0) = 1;
	Zj(3, 0) = 1;

	QSMatrix<float> Oj(4, 1, 0.0);
	Oj(3, 0) = 1;

	Tj = T0;
	QSMatrix<float> TjZj = Tj*Z0;
	QSMatrix<float> Pj = Tj*Oj;
	QSMatrix<float> cross = crossProduct(TjZj, Pn - Pj);
	jacobian(0, 0) = cross(0, 0);
	jacobian(1, 0) = cross(1, 0);
	jacobian(2, 0) = cross(2, 0);
	jacobian(3, 0) = TjZj(0, 0);
	jacobian(4, 0) = TjZj(1, 0);
	jacobian(5, 0) = TjZj(2, 0);

 	for(int i = 1; i < numLinks; i++){
		Tj = Tj*getTransformationMatrix(lengths[i-1], angles[i-1] - angles[i]);
		Zj = getTransformationMatrix(lengths[i-1], angles[i-1] - angles[i])*Zj;
		QSMatrix<float> TjZj = Tj*Zj;
		QSMatrix<float> Pj = Tj*Oj;
		QSMatrix<float> cross = crossProduct(TjZj, Pn - Pj);
		jacobian(0, i) = cross(0, 0);
		jacobian(1, i) = cross(1, 0);
		jacobian(2, i) = cross(2, 0);
		jacobian(3, i) = TjZj(0, 0);
		jacobian(4, i) = TjZj(1, 0);
		jacobian(5, i) = TjZj(2, 0);
	}

	Tj = Tj*getTransformationMatrix(lengths[numLinks-1], 0);
	endEffector = Tj*Oj;
}

void inverseKinematics::invertJacobian(){
	Eigen::MatrixXf m = Eigen::MatrixXf::Random(jacobian.get_rows(),jacobian.get_cols());

	for (int i=0; i<jacobian.get_rows(); i++) {
	    for (int j=0; j<jacobian.get_cols(); j++) {
	      	m(i, j) = jacobian(i,j);
	    }
	}
 
	QSMatrix<float> U(jacobian.get_rows(), jacobian.get_rows(), 0.0);
	QSMatrix<float> V(jacobian.get_cols(), jacobian.get_cols(), 0.0);
	QSMatrix<float> S(jacobian.get_rows(), jacobian.get_cols(), 0.0);

	Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeFullU | Eigen::ComputeThinV);

	for (int i=0; i<U.get_rows(); i++) {
	    for (int j=0; j<U.get_cols(); j++) {
	      	U(i,j) = svd.matrixU()(i, j);
	    }
	}

	for (int i=0; i<V.get_rows(); i++) {
	    for (int j=0; j<V.get_cols(); j++) {
	      	V(i,j) = svd.matrixV()(i, j);
	    }
	}

	for (int i=0; i< svd.singularValues().size(); i++) {
	    S(i, i) = svd.singularValues()(i);
	}	

	inverseJacobian = QSMatrix<float> (numLinks-1, 6, 0.0);
	QSMatrix<float> Sdagger = QSMatrix<float>(S.get_cols(), S.get_rows(), 0.0);
	for (int i=0; i< svd.singularValues().size(); i++) {
	    Sdagger(i, i) = S(i, i);
	}

	for (int i=0; i<Sdagger.get_rows(); i++) {
	    for (int j=0; j<Sdagger.get_cols(); j++) {
	      	if(Sdagger(i,j) != 0){
	      		Sdagger(i, j) = 1/Sdagger(i, j);
	      	}
	    }
	}

	QSMatrix<float> Utranspose(U.get_cols(), U.get_rows(), 0.0);
	for (int i=0; i<Utranspose.get_rows(); i++) {
	    for (int j=0; j<Utranspose.get_cols(); j++) {
	      	Utranspose(i, j) = U(j, i);
	    }
	}

	QSMatrix<float> temp = Sdagger*Utranspose;
	inverseJacobian = V*temp;
}

void inverseKinematics::computeJointAngleChange(QSMatrix<float> target_pos){
	QSMatrix<float> x(6, 1, 0.0);
	QSMatrix<float> temp = target_pos - endEffector;
	x(0, 0) = temp(0, 0);
	x(1, 0) = temp(1, 0);
	x(2, 0) = temp(2, 0);
	jointAngleChange = inverseJacobian*x;
}