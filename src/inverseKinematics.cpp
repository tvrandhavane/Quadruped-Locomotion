#include "inverseKinematics.h"

inverseKinematics::inverseKinematics(vector<float> lengths, vector<float> angles, vector<float> endEffector){
	this->angles = angles;
	this->lengths = lengths;
	numLinks = lengths.size();
	jacobian = QSMatrix<float> (6, numLinks-1, 0.0);

	QSMatrix<float> Pn(4, 1, 1.0);
	Pn(0, 0) = endEffector[0];
	Pn(1, 0) = endEffector[1];
	Pn(2, 0) = endEffector[2];
	createJacobian(Pn);
	invertJacobian();
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

void inverseKinematics::createJacobian(QSMatrix<float> Pn){
	QSMatrix<float> Tj(4, 4, 0.0);
	for(int i = 0; i < Tj.get_rows(); i++){
	    Tj(i,i) = 1;
	}
	QSMatrix<float> Zj(4, 1, 0.0);
	Zj(2, 0) = 1;
	Zj(3, 0) = 1;

	QSMatrix<float> Oj(4, 1, 0.0);
	Oj(3, 0) = 1;

 	for(int i = 0; i < numLinks-1; i++){
		Tj = Tj*getTransformationMatrix(lengths[i], angles[i+1] - angles[i]);
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