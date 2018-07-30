#include <iostream>
#include <Eigen/Dense>


using namespace std;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Vector3d;
using Eigen::JacobiSVD;
using namespace Eigen;

void test1();
void test2();
void test3();
void test4();
void test5();
void test6();
void testSVD();
void testSVD2();

int main()
{
    /*
    test1();
    test2();
    test3();
    test4();
    test5();
    test6();
    cout << "Привет!" << endl;
    */
    //testSVD();
    testSVD2();
    return 0;
    //abort();
}

void test1(){
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;
}


void test2()
{
    MatrixXd m = MatrixXd::Random(3,3);
    m = (m + MatrixXd::Constant(3,3,1.2)) * 50;
    cout << "MatrixXd::Constant(3,3,1.2) =" << endl << MatrixXd::Constant(3,3,1.2) << endl;
    cout << "m =" << endl << m << endl;
    VectorXd v(3);
    v << 1, 2, 3;
    cout << "m * v =" << endl << m * v << endl;
}


void test3()
{
    Matrix3d m = Matrix3d::Random();
    m = (m + Matrix3d::Constant(1.2)) * 50;
    cout << "m =" << endl << m << endl;
    Vector3d v(1,2,3);

    cout << "m * v =" << endl << m * v << endl;
}

void test4()
{
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << "Here is the matrix m:\n" << m << std::endl;
    VectorXd v(2);
    v(0) = 4;
    v(1) = v(0) - 1;
    std::cout << "Here is the vector v:\n" << v << std::endl;
    {


    Matrix3f m;
    m << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    std::cout << m << std::endl;
    }

}

void test5()
{
    MatrixXd m(2,5);
    m.resize(4,3);
    std::cout << "The matrix m is of size "
        << m.rows() << "x" << m.cols() << std::endl;
    std::cout << "It has " << m.size() << " coefficients" << std::endl;
    VectorXd v(2);
    v.resize(5);
    std::cout << "The vector v is of size " << v.size() << std::endl;
    std::cout << "As a matrix, v is of size "
        << v.rows() << "x" << v.cols() << std::endl;
}


void test6()
{
    MatrixXf a(2,2);
    std::cout << "a is of size " << a.rows() << "x" << a.cols() << std::endl;
    MatrixXf b(3,3);
    a = b;
    std::cout << "a is now of size " << a.rows() << "x" << a.cols() << std::endl;
}

void testSVD()
{
    MatrixXd m(13,3);
    m <<  0.16, 0.49, 0.02,
    0.02, 	0.93 ,	0.83,
    2.36, 	0.63 ,	1.74,
    1.09 ,	-0.35, 	3.12,
    -0.93,	-3.04, 	-1.34,
    -1.07 ,	-2.03, 	-4.64,
    -0.42 ,	-0.47 ,	-4.53,
    -0.11 ,	-0.14, 	-1.92,
    0.06 ,	1.43 ,	0.01,
    -1.29 ,	-3.97 ,	-9.54,
    -0.45 ,	2.04 ,	20.1,
    1.12 ,	2.53, 	-5.59,
    0.88 ,	-0.96 ,	-0.88 ;
    cout << m << endl;
    JacobiSVD<MatrixXd> svd(m, ComputeThinU | ComputeThinV);
    cout << "U matrix:" << endl << svd.matrixU() << endl;
    cout << "V matrix:" << endl << svd.matrixV() << endl;

    MatrixXd r = svd.matrixU();
    double res = 0.0;
    for (int i = 0; i < r.rows(); i++) {
        res+= r(i,0)*r(i,1);
    }
    cout << "check orto u1*u2 = " << res << endl;
    //SVDBase<MatrixXd> svdb;



}


void testSVD2()
{
    MatrixXd m(4,5);
    m <<  1,0,0,0,2,
    0,0,3,0,0,
    0,0,0,0,0,
    0,4,0,0,0;
    cout << m << endl;
    JacobiSVD<MatrixXd> svd(m, ComputeFullU | ComputeFullV/*ComputeThinU | ComputeThinV*/);
    cout << "U matrix:" << endl << -svd.matrixU() << endl;
    cout << "V matrix:" << endl << -svd.matrixV() << endl;
    Index nsv = svd.nonzeroSingularValues();
    cout << "nsv = " << nsv << endl;
    cout << "singular values = " << svd.singularValues() << endl;

}
