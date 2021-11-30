#include "types.hpp"



Eigen::MatrixXd AshkanPseudoinverse(Eigen::MatrixXd A, double singularMinToMaxRatio)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV ); // computes the SVD
    Eigen::MatrixXd S_inv(svd.matrixV().cols(),svd.matrixU().rows());
    S_inv.setZero();
    for( int i = 0; i < std::min(A.rows(),A.cols()); i++)
    {
        double val = 0;
        if( svd.singularValues()[i] > svd.singularValues()[0]*singularMinToMaxRatio )// threashold singular values anything less than 1/1000 of the max is set to 0
            val = 1.0 / svd.singularValues()[i];
        S_inv(i,i) = val;
    }
    Eigen::MatrixXd answer;
    answer = svd.matrixV()*(S_inv*(svd.matrixU().transpose()));
    return answer;
};


Eigen::MatrixXd Pseudoinverse(Eigen::MatrixXd A)
{
    Eigen::MatrixXd B = A.completeOrthogonalDecomposition().pseudoInverse();
    return B;
};

double CurrentD2A(double current)   //Converts the current to D2A values:
{
    // std::cout<<(16383.0/(30.0))*(current+15.0)<<"\n";
    return (16383.0/(30.0))*(current+15.0);
};

TRANSMAT ArmFK(DHPARAMS dh, TRANSMAT base2center = Eigen::MatrixXd::Identity(4,4)) { 
    double d1 = dh[0];
    double theta1 = dh[1];
    double a2 = dh[2];
    double d2 = dh[3];
    double d3 = dh[4];
    double theta4 = dh[5];
    TRANSMAT transmat = Eigen::MatrixXd::Identity(4,4);
    transmat(0,0) = cos(theta1)*cos(theta4) - sin(theta1)*sin(theta4);
    transmat(0,1) = -cos(theta1)*sin(theta4) - sin(theta1)*cos(theta4);
    transmat(0,3) = a2*cos(theta1) - d3*sin(theta1);
    transmat(1,0) = sin(theta1)*cos(theta4)+cos(theta1)*sin(theta4);
    transmat(1,1) = cos(theta1)*cos(theta4)-sin(theta1)*sin(theta4);
    transmat(1,3) = d3*cos(theta1)+a2*sin(theta1);
    transmat(2,3) = d1+d2;

    transmat = base2center * transmat;

    return transmat;
}