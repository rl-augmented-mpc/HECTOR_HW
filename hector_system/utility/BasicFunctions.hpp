//
// Created by Junny on 7/1/20.
//

#ifndef SRC_BASICFUNCTIONS_HPP
#define SRC_BASICFUNCTIONS_HPP

#include <cmath>
#include <cassert>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <cstring>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <boost/algorithm/string.hpp>
const double TOLERANCE_ = 1e-3;

class BasicFunctions {

public:


    template <typename T>
    static std::string to_string_n_signficant_figures(const T a_value, const int n = 6)
    {
        std::ostringstream out;
        out << std::setprecision(n) << a_value;
        return out.str();
    }

    template <typename T>
    static std::string to_string_n_figures_under_0(const T a_value, const int n = 6)
    {
        std::ostringstream out;
        out << std::fixed << std::setprecision(n) << a_value;
        return out.str();
    }


    template <typename Derived>
    static bool NearZero(const Eigen::MatrixBase<Derived> &vec, double tolerance)
    {
        bool check = (vec.norm() < tolerance);
        return check;
    }

    template <typename T>
    static bool NearZero_value(const T &val, double tolerance)
    {
        bool check = (fabs(val) < tolerance);
        return check;
    }










    template <typename Derived>
    static void Hessian_S_update(Eigen::MatrixBase<Derived> &Original_Hess, Eigen::MatrixBase<Derived> new_Hess, double threshold)
    {

//        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> X_operated;
//        X_operated.resize(Original_Hess.rows(),Original_Hess.cols());
//        X_operated.setZero();

        for (int i=0; i<new_Hess.cols(); i++){
            for (int j=0; j<new_Hess.rows(); j++){
                if (Original_Hess(i,j)==1 || fabs(new_Hess(i,j))>=threshold){
                    Original_Hess(i,j)=1;
                }else{
                    Original_Hess(i,j)=0;
                }
            }
        }
    }

    template <typename Derived>
    static void Show_mtrx(const Eigen::MatrixBase<Derived> &X)
    {
//        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> X_operated;
//        X_operated.resize(X.rows(),X.cols());
//        X_operated.setZero();

//        double truncation_threshold;
//        truncation_threshold = 1e-7;

//        for (int i=0; i<X.cols(); i++){
//            for (int j=0; j<X.rows(); j++){
//                if (fabs(X(i,j))<truncation_threshold){
//                    X_operated(i,j)=0;
//                }else{
//                    X_operated(i,j)=X(i,j);
//                }
//            }
//        }

//        std::cout<<X_operated<<std::endl;


        int i=0;
        while(X.cols() - 12*i >= 12){
            std::cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
            <<std::endl<<X.middleCols(12*i,12)<<std::endl;
            i++;
        }

        std::cout<<std::endl<<X.rightCols(X.cols()-12*i)<<std::endl;

    }






    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Hat_so3(const Eigen::MatrixBase<Derived> &vec)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> hat_matrix;

        hat_matrix << typename Derived::Scalar(0), -vec(2), vec(1),
                vec(2), typename Derived::Scalar(0), -vec(0),
                -vec(1), vec(0), typename Derived::Scalar(0);
        return hat_matrix;
    }



    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> Vee_so3(const Eigen::MatrixBase<Derived> &so3)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 1> vee_vec;
        vee_vec << so3(2,1), so3(0,2), so3(1,0);
        return vee_vec;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Expm_so3(const Eigen::MatrixBase<Derived> &so3)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> exponential_map;

        Eigen::Matrix<typename Derived::Scalar, 3, 1> vec_tmp = Vee_so3(so3);
        typename Derived::Scalar norm = vec_tmp.norm();

        if (NearZero(vec_tmp, TOLERANCE_))
        {
            exponential_map.setIdentity();
        }
        else
        {
            exponential_map = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + (sin(norm)/norm)*so3 + ((1.0-cos(norm))/(norm*norm))*so3*so3;
        }


//        exponential_map = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + so3;


        return exponential_map;

    }



//    template <typename Derived>
//    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Logm_so3(const Eigen::MatrixBase<Derived> &R)
//    {
//        Eigen::Matrix<typename Derived::Scalar, 3, 1> omg;
//        Eigen::Matrix<typename Derived::Scalar, 3, 3> so3matrix;
//        typename Derived::Scalar trace = R(0,0)+R(1,1)+R(2,2);
//        typename Derived::Scalar acosinput = (trace - 1.0)/2.0;


//        if (acosinput>=1.0)
//        {
//            so3matrix = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Zero();
//        }
//        else if(acosinput<=(-1.0))
//        {
//            if(!NearZero_value(1.0+R(2,2)))
//            {
//                Eigen::Matrix<typename Derived::Scalar, 3, 1> tmp_vec(R(0,2),R(1,2),(1.0+R(2,2)));
//                omg = (1.0/sqrt(2.0*(1.0+R(2,2))))*tmp_vec;
//            }
//            else if(!NearZero_value(1.0+R(1,1)))
//            {
//                Eigen::Matrix<typename Derived::Scalar, 3, 1> tmp_vec(R(0,1),(1.0+R(1,1)),R(2,1));
//                omg = (1.0/sqrt(2.0*(1.0+R(1,1))))*tmp_vec;


//            }
//            else
//            {
//                Eigen::Matrix<typename Derived::Scalar, 3, 1> tmp_vec((1.0+R(0,0)),R(1,0),R(2,0));
//                omg = (1.0/sqrt(2.0*(1.0+R(0,0))))*tmp_vec;


//            }

//            so3matrix = Hat_so3(M_PI*omg);

//        }
//        else
//        {
//            typename Derived::Scalar theta = acos(acosinput);
//            so3matrix = (theta/(2.0*sin(theta)))*(R-R.transpose());
//        }

//        return so3matrix;

//    }


    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Logm_so3(const Eigen::MatrixBase<Derived> &R)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> logarithm_so3;
        logarithm_so3 = Hat_so3(Logm_Vec(R));
        return logarithm_so3;
    }






    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> Logm_Vec(const Eigen::MatrixBase<Derived> &R)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 1> omega;
        typename Derived::Scalar tr = R.trace();
        typename Derived::Scalar acosinput = (tr - 1.0)/2.0;

          // note switch to base 1

          const typename Derived::Scalar R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
          const typename Derived::Scalar R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
          const typename Derived::Scalar R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);


          // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
          // we do something special
          if ((tr + 1.0) < TOLERANCE_) {
            if (std::abs(R33 + 1.0) > 1e-5)
            {
              Eigen::Matrix<typename Derived::Scalar, 3, 1> tmpvec(R13, R23, 1.0 + R33);
              omega = (M_PI / sqrt(2.0 + 2.0 * R33)) * tmpvec;
            }
            else if (std::abs(R22 + 1.0) > 1e-5)
            {
              Eigen::Matrix<typename Derived::Scalar, 3, 1> tmpvec(R12, 1.0 + R22, R32);
              omega = (M_PI / sqrt(2.0 + 2.0 * R22)) * tmpvec;
            }
            else
            {
              // if(std::abs(R.r1_.x()+1.0) > 1e-5)  This is implicit
              Eigen::Matrix<typename Derived::Scalar, 3, 1> tmpvec(1.0 + R11, R21, R31);
              omega = (M_PI / sqrt(2.0 + 2.0 * R11)) * tmpvec;
            }
          } else {
            typename Derived::Scalar magnitude;
            const typename Derived::Scalar tr_3 = tr - 3.0;  // always negative
            if (tr_3 < -1e-7) {
              typename Derived::Scalar theta = acos((tr - 1.0) / 2.0);
              magnitude = theta / (2.0 * sin(theta));
            } else {
              // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
              // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
              magnitude = 0.5 - tr_3 * tr_3 / 12.0;
            }


            Eigen::Matrix<typename Derived::Scalar, 3, 1> tmpvec2(R32 - R23, R13 - R31, R21 - R12);
            omega = magnitude * tmpvec2;
          }

          return omega;
        }




    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Expm_Vec(const Eigen::MatrixBase<Derived> &vec)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> exponential_map;
        exponential_map = Expm_so3(Hat_so3(vec));
        return exponential_map;
    }



    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> Logm_seK_Vec(const Eigen::MatrixBase<Derived> &X, const int k)
    {
//        Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
//        R = X.block(0,0,3,3);
//        Eigen::Matrix<typename Derived::Scalar, 3, 1> phi;
//        phi = Logm_Vec(R);

        Eigen::Matrix<typename Derived::Scalar, 3, 1> phi;
        phi = Logm_Vec(X.block(0,0,3,3));
        Eigen::Matrix<typename Derived::Scalar, 3, 3> inv_Left_Jacobian;
//        inv_Left_Jacobian = Inv_Left_Jacobian_SO3(phi);
        inv_Left_Jacobian = Inv_Left_Jacobian_SO3(phi);

        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> Xi_seK;
        Xi_seK.resize(3+3*k, 1);
        Xi_seK.setZero();

        Xi_seK.block(0,0, 3,1) = phi;

        for (int i=0; i<k; i++){
            Xi_seK.block(3+3*i,0, 3,1) = inv_Left_Jacobian * X.block(0,3 + i, 3,1);

        }

        return Xi_seK;
    }


    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> Expm_seK_Vec(const Eigen::MatrixBase<Derived> &Xi, const int k)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 1> phi;
        phi = Xi.block(0,0, 3,1);
        Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
        Eigen::Matrix<typename Derived::Scalar, 3, 3> jacobian_L_phi;

        R = Expm_Vec(phi);
        jacobian_L_phi = Left_Jacobian_SO3(phi);

        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> X_seK;
        X_seK.resize(3+k, 3+k);
        X_seK.setZero();

        X_seK.block(0,0, 3,3) = R;

        for (int i=0; i<k; i++){
            X_seK.block(0, 3+i, 3,1) = jacobian_L_phi * Xi.block(3+3*i,0, 3,1);
            X_seK(3+i,3+i) = 1;
        }

        return X_seK;
    }






    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Right_Jacobian_SO3(const Eigen::MatrixBase<Derived> &vec)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> jacobian;
        Eigen::Matrix<typename Derived::Scalar, 3, 3> so3;
        so3 = Hat_so3(vec);
        typename Derived::Scalar norm = vec.norm();

        if (NearZero(vec, TOLERANCE_))
        {
            jacobian.setIdentity();
        }
        else
        {
            Eigen::Matrix<typename Derived::Scalar, 3, 1> vec_pow3;
            vec_pow3 << vec(0)*vec(0)*vec(0),vec(1)*vec(1)*vec(1),vec(2)*vec(2)*vec(2);
            jacobian = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - ((1.0-cos(norm))/(norm*norm))*so3 + ((norm-sin(norm))/vec_pow3.norm())*so3*so3;
        }

        return jacobian;
    }



    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Left_Jacobian_SO3(const Eigen::MatrixBase<Derived> &vec)
    {

        Eigen::Matrix<typename Derived::Scalar, 3, 3> left_jacobian;
        Eigen::Matrix<typename Derived::Scalar, 3, 3> so3;
        so3 = Hat_so3(vec);
        typename Derived::Scalar norm = vec.norm();
        Eigen::Matrix<typename Derived::Scalar, 3, 1> a = vec/norm;

        if (NearZero(vec, TOLERANCE_))
        {
            left_jacobian.setIdentity();
        }
        else
        {

            Eigen::Matrix<typename Derived::Scalar, 3, 1> vec_pow3;
            vec_pow3 << vec(0)*vec(0)*vec(0),vec(1)*vec(1)*vec(1),vec(2)*vec(2)*vec(2);
            left_jacobian = (sin(norm)/norm)*Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - (1.0-sin(norm)/norm)*a*a.transpose() + ((1-cos(norm))/norm)*Hat_so3(a);

        }

        return left_jacobian;
    }



    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Inv_Left_Jacobian_SO3(const Eigen::MatrixBase<Derived> &vec)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> jacobian;
        Eigen::Matrix<typename Derived::Scalar, 3, 3> so3;
        so3 = Hat_so3(vec);
        typename Derived::Scalar norm = vec.norm();
        Eigen::Matrix<typename Derived::Scalar, 3, 1> a = vec/norm;

        if (NearZero(vec, TOLERANCE_))
        {
            jacobian.setIdentity();
        }
        else
        {
            jacobian = (0.5*norm/tan(0.5*norm))*Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + (1-0.5*norm/tan(0.5*norm))*a*a.transpose() - 0.5*norm*Hat_so3(a);
        }
        return jacobian;
    }





    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> Left_Jacobian_SEk(const Eigen::MatrixBase<Derived> &Xi, const int k)
    {
        //Right_Jacobia_SEk(Xi) = Left_Jacobian_SEk(-Xi)


        Eigen::Matrix<typename Derived::Scalar, 3, 1> phi;
        phi = Xi.block(0,0, 3,1);

        Eigen::Matrix<typename Derived::Scalar, 3, 3> jacobian_L_phi;
        jacobian_L_phi = Left_Jacobian_SO3(phi);
        typename Derived::Scalar norm = phi.norm();

        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> J_left_SEk;
        J_left_SEk.resize(3+3*k, 3+3*k);
        J_left_SEk.setZero();

        if (NearZero(phi, TOLERANCE_))
        {
            J_left_SEk.setIdentity();

        }else{

            J_left_SEk.block(0,0, 3,3) = jacobian_L_phi;

            for (int i=1; i<=k; i++){
                J_left_SEk.block(3*i,3*i, 3,3) = jacobian_L_phi;

                Eigen::Matrix<typename Derived::Scalar, 3, 1> tk;
                tk = Xi.block(3*i,0, 3,1);

                J_left_SEk.block(3*i,0, 3,3) = 0.5*Hat_so3(tk)
                        + ( (norm - sin(norm))/pow(norm,3) )*(Hat_so3(phi)*Hat_so3(tk) + Hat_so3(tk)*Hat_so3(phi) + Hat_so3(phi)*Hat_so3(tk)*Hat_so3(phi) )
                        + ( (norm*norm+2*cos(norm)-2)/(2*pow(norm,4)) )*( Hat_so3(phi)*Hat_so3(phi)*Hat_so3(tk) + Hat_so3(tk)*Hat_so3(phi)*Hat_so3(phi) - 3*Hat_so3(phi)*Hat_so3(tk)*Hat_so3(phi) )
                        + ( (2*norm-3*sin(norm)+norm*cos(norm))/(2*pow(norm,5)) )*( Hat_so3(phi)*Hat_so3(tk)*Hat_so3(phi)*Hat_so3(phi) + Hat_so3(phi)*Hat_so3(phi)*Hat_so3(tk)*Hat_so3(phi) );
            }

        }


        return J_left_SEk;
    }



    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> Inv_Left_Jacobian_SEk(const Eigen::MatrixBase<Derived> &Xi, const int k)
    {
        //Right_Jacobia_SEk(Xi) = Left_Jacobian_SEk(-Xi)


        Eigen::Matrix<typename Derived::Scalar, 3, 1> phi;
        phi = Xi.block(0,0, 3,1);
        Eigen::Matrix3d W = Hat_so3(phi);

        typename Derived::Scalar norm = phi.norm();

        Eigen::Matrix<typename Derived::Scalar, 3, 3> inv_jacobian_L_phi;
        inv_jacobian_L_phi = Inv_Left_Jacobian_SO3(phi);

        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> inv_J_left_SEk;
        inv_J_left_SEk.resize(3+3*k, 3+3*k);
        inv_J_left_SEk.setZero();



        if (NearZero(phi, TOLERANCE_))
        {
            inv_J_left_SEk.setIdentity();

        }else{

            inv_J_left_SEk.block(0,0, 3,3) = inv_jacobian_L_phi;

            for (int i=1; i<=k; i++){
                inv_J_left_SEk.block(3*i,3*i, 3,3) = inv_jacobian_L_phi;

                Eigen::Matrix<typename Derived::Scalar, 3, 1> tk;
                tk = Xi.block(3*i,0, 3,1);
                Eigen::Matrix3d V = Hat_so3(tk);

                Eigen::Matrix<typename Derived::Scalar, 3, 3> Qk;

                double A, B, C, D;

//              //Barfoot
                 A = 0.5;
                 B = (norm - sin(norm))/pow(norm,3);
                 C = - (1 - 0.5*pow(norm,2) - cos(norm))/(pow(norm,4));
                 D = 0.5 * (2*norm - 3*sin(norm) + norm*cos(norm) )/(pow(norm,5));

                Qk = A*V
                    + B * (W*V + V*W + W*V*W )
                    + C * ( W*W*V + V*W*W - 3*W*V*W )
                    + D * ( W*V*W*W + W*W*V*W );

//                //gtsam
//                A = -0.5;
//                B = (norm - sin(norm))/pow(norm,3);
//                C = (1 - 0.5*pow(norm,2) - cos(norm))/(pow(norm,4));
//                D = - 0.5 * ( C - 3 * (norm - sin(norm) - pow(norm,3)/6. )/pow(norm,5) );
//
//                Qk = A*V
//                     + B * (W*V + V*W - W*V*W )
//                     + C * ( W*W*V + V*W*W - 3*W*V*W )
//                     + D * ( W*V*W*W + W*W*V*W );

//                //Unknown Guys
//                A = 0.5;
//                B = (norm - sin(norm))/pow(norm,3);
//                C = - (1 - 0.5*pow(norm,2) - cos(norm))/(pow(norm,4));
//                D = - ( C - 3 * ( norm - sin(norm)-pow(norm,3)/6. )/( pow(norm,5) ) );
//
//                Qk = A*V
//                     + B * (W*V + V*W + W*V*W )
//                     + C * ( W*W*V + V*W*W - 3*W*V*W )
//                     + D * ( W*V*W*W );


                inv_J_left_SEk.block(3*i,0, 3,3) = -inv_jacobian_L_phi* Qk *inv_jacobian_L_phi;

            }
        }

        return inv_J_left_SEk;
    }


    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Gamma_2(const Eigen::MatrixBase<Derived> &vec)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> gamma_2;
        Eigen::Matrix<typename Derived::Scalar, 3, 3> so3;
        so3 = Hat_so3(vec);
        typename Derived::Scalar norm = vec.norm();

        if (NearZero(vec, TOLERANCE_))
        {
            gamma_2.setIdentity();
        }
        else
        {
            //Eigen::Matrix<typename Derived::Scalar, 3, 1> vec_pow3;
            //vec_pow3 << vec(0)*vec(0)*vec(0),vec(1)*vec(1)*vec(1),vec(2)*vec(2)*vec(2);
            gamma_2 = 0.5 * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()
                    + ((norm - sin(norm))/(norm*norm*norm)) * so3
                    + ((norm*norm + 2*cos(norm) - 2)/(2* norm*norm*norm*norm)) * so3*so3;
        }

        return gamma_2;
    }


    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 1> Rotation_to_EulerZYX(const Eigen::MatrixBase<Derived> &R)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 1> euler;

        if (R(2,0)<1.0)
        {

            if(R(2,0)>(-1.0))
            {
                euler<<atan2(R(2,1),R(2,2)),asin(-R(2,0)),atan2(R(1,0),R(0,0));
            }
            else
            {
                euler<<0.0,-M_PI/2.0,-atan2(-R(1,2),R(1,1));
            }

        }
        else
        {
            euler<<0.0,-M_PI/2.0,-atan2(-R(1,2),R(1,1));
        }


        return euler;
    }




    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> EulerXYZ_to_R_bw(const Eigen::MatrixBase<Derived> &eul)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> R_matrix;


        R_matrix = Rot_Z(eul(2))*Rot_Y(eul(1))*Rot_X(eul(0));

        return R_matrix;
    }


    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> EulerZYX_to_R_bw(const Eigen::MatrixBase<Derived> &eul)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> R_matrix;


        R_matrix = Rot_Z(eul(2))*Rot_Y(eul(1))*Rot_X(eul(0));

        return R_matrix;
    }


    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> EulerZYX_to_R_wb(const Eigen::MatrixBase<Derived> &eul)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> R_matrix;

        R_matrix = EulerZYX_to_R_bw(eul);
        R_matrix.transposeInPlace();

        return R_matrix;
    }


    template <typename T>
    static Eigen::Matrix<T, 3, 3> Rot_X(const T &theta)
    {
        Eigen::Matrix<T, 3, 3> R_matrix; R_matrix << 1.0,0.0,0.0, 0.0,cos(theta),-sin(theta), 0.0,sin(theta),cos(theta);

        return R_matrix;
    }

    template <typename T>
    static Eigen::Matrix<T, 3, 3> Rot_Y(const T &theta)
    {
        Eigen::Matrix<T, 3, 3> R_matrix; R_matrix << cos(theta),0.0,sin(theta), 0.0,1.0,0.0, -sin(theta),0.0,cos(theta);

        return R_matrix;
    }

    template <typename T>
    static Eigen::Matrix<T, 3, 3> Rot_Z(const T &theta)
    {
        Eigen::Matrix<T, 3, 3> R_matrix; R_matrix << cos(theta),-sin(theta),0.0, sin(theta),cos(theta),0.0, 0.0,0.0,1.0;

        return R_matrix;
    }

    template <typename T>
    static T sign(const T val)
    {
        if(val>T(0.0))
        {
            return T(1.0);
        }
        else if(val<T(0.0))
        {
            return T(-1.0);
        }
        else
        {
            return val;
        }

    }

    template <typename T>
    static T Sqrtp(const T a)
    {
        return (a>=0 ? sqrt(a) : 0);
    }




    template <typename Derived>
    static Eigen::MatrixXd blkdiag(const Eigen::MatrixBase<Derived>& a, int count)
    {
        Eigen::MatrixXd bdm = Eigen::MatrixXd::Zero(a.rows() * count, a.cols() * count);
        for (int i = 0; i < count; ++i)
        {
            bdm.block(i * a.rows(), i * a.cols(), a.rows(), a.cols()) = a;
        }

        return bdm;
    }



//    template <typename Derived>
//    static Eigen::Matrix<typename Derived::Scalar, 4, 1> Rotation_Matrix_to_Quaternion(const Eigen::MatrixBase<Derived> &R)
//    {
//        Eigen::Matrix<typename Derived::Scalar, 4, 1> quat;

//        nu = quat(0);
//        ex = quat(1);
//        ey = quat(2);
//        ez = quat(3);

//        R(0,0) = nu*nu + ex*ex -ey*ey - ez*ez;
//        R(0,1) = 2*(ex*ey - nu*ez);
//        R(0,2) = 2*(ex*ez + nu*ey);
//        R(1,0) = 2*(ex*ey +nu*ez);


//        sign(R(2,1)-R(1,2))*0.5*Sqrtp(1+R(0,0)-R(1,1)-R(2,2));
//        quat(2) = sign(R(0,2)-R(2,0))*0.5*Sqrtp(1-R(0,0)+R(1,1)-R(2,2));
//        quat(3) = sign(R(1,0)-R(0,1))*0.5*Sqrtp(1-R(0,0)-R(1,1)+R(2,2));
//        if((quat(0)*quat(0) + quat(1)*quat(1) + quat(2)*quat(2) + quat(3)*quat(3)) < TOLERANCE_)
//        {
//            quat(0) = 1.0;
//        }



//        return quat;
//    }



    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 1> Rotation_Matrix_to_Quaternion(const Eigen::MatrixBase<Derived> &R)
    {
        // R -> q(wxyz)
        Eigen::Matrix<typename Derived::Scalar, 4, 1> quat;
        quat(0) = Sqrtp(1.0+R(0,0)+R(1,1)+R(2,2));
        quat(1) = sign(R(2,1)-R(1,2))*Sqrtp(1+R(0,0)-R(1,1)-R(2,2));
        quat(2) = sign(R(0,2)-R(2,0))*Sqrtp(1-R(0,0)+R(1,1)-R(2,2));
        quat(3) = sign(R(1,0)-R(0,1))*Sqrtp(1-R(0,0)-R(1,1)+R(2,2));

        quat = 0.5*quat;
        if((quat(0)*quat(0) + quat(1)*quat(1) + quat(2)*quat(2) + quat(3)*quat(3)) < TOLERANCE_)
        {
            quat(0) = 1.0;
        }



        return quat;
    }
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> Quaternion_to_Rotation_Matrix(const Eigen::MatrixBase<Derived> &q) {
        // q(wxyz) -> R
        Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
        R(0) = q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3);
        R(1) = 2 * q(0) * q(3) + 2 * q(1) * q(2);
        R(2) = 2 * q(1) * q(3) - 2 * q(0) * q(2);

        R(3) = 2 * q(1) * q(2) - 2 * q(0) * q(3);
        R(4) = q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3);
        R(5) = 2 * q(0) * q(1) + 2 * q(2) * q(3);

        R(6) = 2 * q(0) * q(2) + 2 * q(1) * q(3);
        R(7) = 2 * q(2) * q(3) - 2 * q(0) * q(1);
        R(8) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);

        return R;
    }



};


#endif //SRC_BASICFUNCTIONS_HPP
