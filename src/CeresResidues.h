#pragma once
/** Holds all the ceres related residue terms.

        Author  : Manohar Kuse <mpkuse@connect.ust.hk>
        Created : 17th May, 2019
*/

//ceres
#include <ceres/ceres.h>
#include "utils/PoseManipUtils.h"


using namespace std;
using namespace Eigen;




class SixDOFError
{
public:
    SixDOFError( const Matrix4d& _observed__c1_T_c2, const double _weight=1.0 ) : observed__c1_T_c2( _observed__c1_T_c2 )
    {
        observed_c1_q_c2 = Quaterniond( _observed__c1_T_c2.topLeftCorner<3,3>() );
        observed_c1_t_c2 << _observed__c1_T_c2(0,3), _observed__c1_T_c2(1,3), _observed__c1_T_c2(2,3);

        weight = _weight;
    }

    // q1, t1 : w_T_c1
    // q2, t2 : w_T_c2
    template <typename T>
    bool operator() ( const T* const q1, const T* const t1,   const T* const q2, const T* const t2, T* residue_ptr ) const
    {
        // Eigen:
        // Note the order of the arguments: the real w coefficient first,
        // while internally the coefficients are stored in the following order: [x, y, z, w]

        // q1,t1 --> w_T_c1
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );

        // q2,t2 --> w_T_c2
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_2( t2 );
        Eigen::Map<const Eigen::Quaternion<T> > q_2( q2 );

        // relative transforms between the 2 frames
        Quaternion<T> q_1_inverse = q_1.conjugate();
        Quaternion<T> q_12_estimated = q_1_inverse * q_2;
        Matrix<T,3,1> p_12_estimated = q_1_inverse * (p_2 - p_1);

        // compute error between orientations estimates
        Quaternion<T> delta_q = q_12_estimated.conjugate() * observed_c1_q_c2.cast<T>();
        Matrix<T,3,1> delta_t = q_12_estimated.conjugate() * ( observed_c1_t_c2.cast<T>() - p_12_estimated );


        Eigen::Map<Matrix<T,6,1> > residuals( residue_ptr );
        residuals.block(0,0,  3,1) =  delta_t;
        residuals.block(3,0,  3,1) =  T(2.0) * delta_q.vec();


        // Dynamic Covariance Scaling
        T phi = T(5.0);
        // T s = T(2.)*phi / ( phi + residuals.squaredNorm() );
        T s = T(1.0);
        residuals *= s * T(weight);
        return true;

    }


    static ceres::CostFunction* Create( const Matrix4d& _observed__c1_T_c2, const double weight=1.0 )
    {
      return ( new ceres::AutoDiffCostFunction<SixDOFError,6,4,3,4,3>
        (
          new SixDOFError(_observed__c1_T_c2, weight )
        )
      );
    }


private:


    Matrix4d observed__c1_T_c2;
    Quaterniond observed_c1_q_c2;
    Matrix<double,3,1> observed_c1_t_c2;

    double weight;
};


// With ceres you cannot set and then unset parameter blocks as constant. So the get arround
// to that issue is to have pose regularization. The poses that you want to keep as constant
// (for example the 1st pose of the path), just add penalty for its change with a large weight value.
class NodePoseRegularization
{
public:
    NodePoseRegularization( const Matrix4d _nodepose, const double _weight ) : nodepose( _nodepose ), weight( _weight )
    {

    }

    template <typename T>
    bool operator() ( const T* const q1, const T* const t1, T* residue_ptr ) const
    {
        // q1,t1 --> w_T_c1
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );
        Matrix<T,4,4> npose = Matrix<T,4,4>::Identity();
        npose.topLeftCorner(3,3) = q_1.toRotationMatrix();
        npose.col(3).topRows(3) = p_1;

        // nodepose --> T
        Matrix<T,4,4> f = nodepose.cast<T> ();

        Matrix<T,4,4> delta = f.inverse() * npose ;
        Matrix<T,3,3> R = delta.topLeftCorner(3,3);
        Quaternion<T> delta_q( R );


        Eigen::Map<Matrix<T,6,1> > residuals( residue_ptr );
        residuals.block(0,0, 3,1) = T(weight) * delta.col(3).topRows(3);
        residuals.block(3,0, 3,1) = T(weight) * T(2.0) * delta_q.vec();

        return true;
    }

    static ceres::CostFunction* Create( const Matrix4d _observed_node_pose, const double xweight )
    {
      return ( new ceres::AutoDiffCostFunction<NodePoseRegularization,6,4,3>
        (
          new NodePoseRegularization(_observed_node_pose, xweight )
        )
      );
    }

private:
    const Matrix4d nodepose;
    const double weight;
};



class SixDOFErrorWithSwitchingConstraints
{
public:
    SixDOFErrorWithSwitchingConstraints( const Matrix4d& _observed__c1_T_c2, const double _weight=1.0 ) : observed__c1_T_c2( _observed__c1_T_c2 )
    {
        observed_c1_q_c2 = Quaterniond( _observed__c1_T_c2.topLeftCorner<3,3>() );
        observed_c1_t_c2 << _observed__c1_T_c2(0,3), _observed__c1_T_c2(1,3), _observed__c1_T_c2(2,3);

        weight = _weight;
    }

    // q1, t1 : w_T_c1
    // q2, t2 : w_T_c2
    template <typename T>
    bool operator() ( const T* const q1, const T* const t1,
                      const T* const q2, const T* const t2,
                      const T* const switching_var,
                      T* residue_ptr ) const
    {
        // Eigen:
        // Note the order of the arguments: the real w coefficient first,
        // while internally the coefficients are stored in the following order: [x, y, z, w]

        // q1,t1 --> w_T_c1
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );

        // q2,t2 --> w_T_c2
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_2( t2 );
        Eigen::Map<const Eigen::Quaternion<T> > q_2( q2 );

        // relative transforms between the 2 frames
        Quaternion<T> q_1_inverse = q_1.conjugate();
        Quaternion<T> q_12_estimated = q_1_inverse * q_2;
        Matrix<T,3,1> p_12_estimated = q_1_inverse * (p_2 - p_1);

        // compute error between orientations estimates
        Quaternion<T> delta_q = q_12_estimated.conjugate() * observed_c1_q_c2.cast<T>();
        Matrix<T,3,1> delta_t = q_12_estimated.conjugate() * ( observed_c1_t_c2.cast<T>() - p_12_estimated );


        Eigen::Map<Matrix<T,7,1> > residuals( residue_ptr );
        residuals.block(0,0,  3,1) =  delta_t;
        residuals.block(3,0,  3,1) =  T(2.0) * delta_q.vec();
        residuals(6) = T(1.0) * ( T(1.0) - switching_var[0] ); //switching constraint penalty

        // pitch and roll aka residuals(5) and residuals(4) are observable witht he IMU, so
        // increase the penalty for changing those. This will make them to be nearer to initial estimates.
        // residuals(4) *= T(5.);
        // residuals(5) *= T(5.);


        T s = switching_var[0];
        residuals *= s; //* T(weight);
        return true;

    }


    static ceres::CostFunction* Create( const Matrix4d& _observed__c1_T_c2, const double weight=1.0 )
    {
      return ( new ceres::AutoDiffCostFunction<SixDOFErrorWithSwitchingConstraints,7,4,3,4,3,1>
        (
          new SixDOFErrorWithSwitchingConstraints(_observed__c1_T_c2, weight )
        )
      );
    }


private:


    Matrix4d observed__c1_T_c2;
    Quaterniond observed_c1_q_c2;
    Matrix<double,3,1> observed_c1_t_c2;

    double weight;
};


// output is in degrees
template <typename T>
Matrix<T,3,1> R2ypr( const Matrix<T,3,3>& R)
{
  Matrix<T,3,1> n = R.col(0);
  Matrix<T,3,1> o = R.col(1);
  Matrix<T,3,1> a = R.col(2);

  Matrix<T,3,1> ypr(3);
  T y = atan2(n(1), n(0));
  T p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  T r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / T(M_PI) * T(180.0);
}





// In this residue function, internally we rely on the
// Euler angle representation and not Quaternions as before.
// We set higher penalty for changing pitch and roll. This is
// not a true 4DOF error term.
class FourDOFError
{
public:
    FourDOFError( const Matrix4d& _observed__c1_T_c2, const double _weight=1.0 ) : observed__c1_T_c2( _observed__c1_T_c2 )
    {
        // Convert c1_T_c2 to Euler Angle representation
        PoseManipUtils::eigenmat_to_rawyprt( _observed__c1_T_c2, observed_c1_ypr_c2, observed_c1_t_c2 );
        observed_c1_q_c2 = Quaterniond( _observed__c1_T_c2.topLeftCorner<3,3>() );

        weight = _weight;
    }

    // q1, t1 : w_T_c1
    // q2, t2 : w_T_c2
    template <typename T>
    bool operator() ( const T* const q1, const T* const t1,
                      const T* const q2, const T* const t2,
                      T* residue_ptr ) const
    {
        // Eigen:
        // Note the order of the arguments: the real w coefficient first,
        // while internally the coefficients are stored in the following order: [x, y, z, w]

        // q1,t1 --> w_T_c1
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );

        // q2,t2 --> w_T_c2
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_2( t2 );
        Eigen::Map<const Eigen::Quaternion<T> > q_2( q2 );

        // relative transforms between the 2 frames
        Quaternion<T> q_1_inverse = q_1.conjugate();
        Quaternion<T> q_12_estimated = q_1_inverse * q_2;
        Matrix<T,3,1> p_12_estimated = q_1_inverse * (p_2 - p_1);

        // compute error between orientations estimates
        Quaternion<T> delta_q = q_12_estimated.conjugate() * observed_c1_q_c2.cast<T>();
        Matrix<T,3,1> delta_t = q_12_estimated.conjugate() * ( observed_c1_t_c2.cast<T>() - p_12_estimated );

        // penalty - A
        Eigen::Map<Matrix<T,6,1> > residuals( residue_ptr );
        // residuals.block(0,0,  3,1) =  delta_t; // translation error
        residuals(0) = delta_t(0);
        residuals(1) = delta_t(1);
        residuals(2) = delta_t(2);


        // convert Quaternion to euler angle using rotation matrix as intermediate
        Matrix<T,3,3> delta_Rot = delta_q.toRotationMatrix();
        Matrix<T,3,1> delta_ypr = R2ypr( delta_Rot );
        residuals(3) = T(4.) * delta_ypr(0);
        residuals(4) = T(10.) * delta_ypr(1);
        residuals(5) = T(10.) * delta_ypr(2);

        residuals *= T(weight);

        return true;

    }


    static ceres::CostFunction* Create( const Matrix4d& _observed__c1_T_c2, const double weight=1.0 )
    {
      return ( new ceres::AutoDiffCostFunction<FourDOFError,6,4,3,4,3>
        (
          new FourDOFError(_observed__c1_T_c2, weight )
        )
      );
    }


private:


    Matrix4d observed__c1_T_c2;
    Quaterniond observed_c1_q_c2;
    Vector3d observed_c1_ypr_c2;
    Vector3d observed_c1_t_c2;

    double weight;
};


// In this residue function, internally we rely on the
// Euler angle representation and not Quaternions as before.
class FourDOFErrorWithSwitchingConstraints
{
public:
    FourDOFErrorWithSwitchingConstraints( const Matrix4d& _observed__c1_T_c2, const double _weight=1.0 ) : observed__c1_T_c2( _observed__c1_T_c2 )
    {
        // Convert c1_T_c2 to Euler Angle representation
        PoseManipUtils::eigenmat_to_rawyprt( _observed__c1_T_c2, observed_c1_ypr_c2, observed_c1_t_c2 );
        observed_c1_q_c2 = Quaterniond( _observed__c1_T_c2.topLeftCorner<3,3>() );

        weight = _weight;
    }

    // q1, t1 : w_T_c1
    // q2, t2 : w_T_c2
    template <typename T>
    bool operator() ( const T* const q1, const T* const t1,
                      const T* const q2, const T* const t2,
                      const T* const switching_var,
                      T* residue_ptr ) const
    {
        // Eigen:
        // Note the order of the arguments: the real w coefficient first,
        // while internally the coefficients are stored in the following order: [x, y, z, w]

        // q1,t1 --> w_T_c1
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_1( t1 );
        Eigen::Map<const Eigen::Quaternion<T> > q_1( q1 );

        // q2,t2 --> w_T_c2
        Eigen::Map<const Eigen::Matrix<T,3,1> > p_2( t2 );
        Eigen::Map<const Eigen::Quaternion<T> > q_2( q2 );

        // relative transforms between the 2 frames
        Quaternion<T> q_1_inverse = q_1.conjugate();
        Quaternion<T> q_12_estimated = q_1_inverse * q_2;
        Matrix<T,3,1> p_12_estimated = q_1_inverse * (p_2 - p_1);

        // compute error between orientations estimates
        Quaternion<T> delta_q = q_12_estimated.conjugate() * observed_c1_q_c2.cast<T>();
        Matrix<T,3,1> delta_t = q_12_estimated.conjugate() * ( observed_c1_t_c2.cast<T>() - p_12_estimated );

        // penalty - A
        Eigen::Map<Matrix<T,7,1> > residuals( residue_ptr );
        // residuals.block(0,0,  3,1) =  delta_t; // translation error
        residuals(0) = delta_t(0);
        residuals(1) = delta_t(1);
        residuals(2) = delta_t(2);

        residuals(6) = T(1.0) * ( T(1.0) - switching_var[0] ); //switching constraint penalty

        // convert Quaternion to euler angle using rotation matrix as intermediate
        Matrix<T,3,3> delta_Rot = delta_q.toRotationMatrix();
        Matrix<T,3,1> delta_ypr = R2ypr( delta_Rot );
        residuals(3) = T(4.) * delta_ypr(0);
        residuals(4) = T(10.0) * delta_ypr(1);
        residuals(5) = T(10.0) * delta_ypr(2);


        T s = switching_var[0];
        residuals *= s; //* T(weight);
        return true;

    }


    static ceres::CostFunction* Create( const Matrix4d& _observed__c1_T_c2, const double weight=1.0 )
    {
      return ( new ceres::AutoDiffCostFunction<FourDOFErrorWithSwitchingConstraints,7,4,3,4,3,1>
        (
          new FourDOFErrorWithSwitchingConstraints(_observed__c1_T_c2, weight )
        )
      );
    }


private:


    Matrix4d observed__c1_T_c2;
    Quaterniond observed_c1_q_c2;
    Vector3d observed_c1_ypr_c2;
    Vector3d observed_c1_t_c2;

    double weight;
};



//------------------
//--- Borrowed from https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/loop_fusion/src/pose_graph.h
//------------------

template <typename T>
T NormalizeAngle(const T& angle_degrees) {
  if (angle_degrees > T(180.0))
  	return angle_degrees - T(360.0);
  else if (angle_degrees < T(-180.0))
  	return angle_degrees + T(360.0);
  else
  	return angle_degrees;
};

class AngleLocalParameterization {
 public:

  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

template <typename T>
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{

	T y = yaw / T(180.0) * T(M_PI);
	T p = pitch / T(180.0) * T(M_PI);
	T r = roll / T(180.0) * T(M_PI);


	R[0] = cos(y) * cos(p);
	R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
	R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
	R[3] = sin(y) * cos(p);
	R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
	R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
	R[6] = -sin(p);
	R[7] = cos(p) * sin(r);
	R[8] = cos(p) * cos(r);
};

template <typename T>
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
	inv_R[0] = R[0];
	inv_R[1] = R[3];
	inv_R[2] = R[6];
	inv_R[3] = R[1];
	inv_R[4] = R[4];
	inv_R[5] = R[7];
	inv_R[6] = R[2];
	inv_R[7] = R[5];
	inv_R[8] = R[8];
};

template <typename T>
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
	r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
	r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
	r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};

struct QinFourDOFWeightError
{
	QinFourDOFWeightError(double t_x, double t_y, double t_z,
                    double relative_yaw, double pitch_i, double roll_i)
				  :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
				  	weight = 1;
				  }

	template <typename T>
	bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
	{
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		// euler to rotation
		T w_R_i[9];
		YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
		// rotation transpose
		T i_R_w[9];
		RotationMatrixTranspose(w_R_i, i_R_w);
		// rotation matrix rotate point
		T t_i_ij[3];
		RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

		residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
		residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
		residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
		residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double relative_yaw, const double pitch_i, const double roll_i)
	{
	  return (new ceres::AutoDiffCostFunction<
	          QinFourDOFWeightError, 4, 1, 3, 1, 3>(
	          	new QinFourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
	}

	double t_x, t_y, t_z;
	double relative_yaw, pitch_i, roll_i;
	double weight;

};
