/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>, Yuting Ye <yuting.ye@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 04/14/2013
 *
 * Georgia Tech Graphics Lab
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DYNAMICS_BODYNODE_DYNAMICS_H
#define DYNAMICS_BODYNODE_DYNAMICS_H

#include <vector>
#include <Eigen/Dense>
#include "kinematics/BodyNode.h"
#include "utils/EigenHelper.h"

namespace rtql8
{
namespace dynamics
{

/// @brief BodyNodeDynamics class represents a single node of the
/// skeleton for dynamics
class BodyNodeDynamics : public kinematics::BodyNode
{
public:
    // We need this aligned allocator because we have Matrix4d as
    // members in this class.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief Default constructor.
    /// @param[in] _name The name can be up to 128.
    BodyNodeDynamics(const char *_name = NULL);

    /// @brief Default destructor.
    virtual ~BodyNodeDynamics();

    //--------------------------------------------------------------------------
    // Following functions called automatically by the skeleton:
    //     computeInverseDynamicsLinear and computeDynamics respectively
    //--------------------------------------------------------------------------
    /// @brief Initialize data structures for linear inverse dynamics
    /// computation.
    void initInverseDynamics();

    /// @brief Initialize data structures for non-recursive dynamics
    /// computation.
    void initDynamics();

    /// @brief Set gravity mode.
    /// @param[in] _gravityMode
    /// TODO: Not implemented yet!
    void setGravityMode(bool _gravityMode)
    { mGravityMode = _gravityMode; }

    /// @brief If the gravity mode is false, this body node does not
    /// being affected by gravity.
    /// @return
    ////// TODO: Not implemented yet!
    bool getGravityMode(void) const { return mGravityMode; }

    /// @brief Computes the velocities in the first pass of the algorithm.
    /// Also computes Transform W etc using updateTransform.
    /// @param[in] _gravity
    /// @param[in] _qdot
    /// @param[in] _qdotdot
    /// @param[in] _computeJacobians Computes Jacobians Jv and Jw if the flag is
    /// true.
    ///
    /// Replaces updateFirstDerivatives of non-recursive dynamics.
    void computeInvDynVelocities(const Eigen::Vector3d& _gravity,
                                 const Eigen::VectorXd* _qdot,
                                 const Eigen::VectorXd* _qdotdot,
                                 bool _computeJacobians = true);

    /// @brief Computes the forces in the second pass of the algorithm.
    /// @param[in] _gravity
    /// @param[in] _qdot
    /// @param[in] _qdotdot
    /// @param[in] _computeJacobians Computes Jacobians Jv and Jw if the flag is
    /// true.
    /// TODO: _gravity, _qdot, _qdotdot are not using in this function.
    void computeInvDynForces(const Eigen::Vector3d& _gravity,
                             const Eigen::VectorXd* _qdot,
                             const Eigen::VectorXd* _qdotdot,
                             bool _withExternalForces);

    /// @brief Update the second derivatives of the transformations.
    void updateSecondDerivatives();

    /// @brief Update the second derivatives of the transformations.
    /// @param[in] _offset
    void updateSecondDerivatives(const Eigen::Vector3d& _offset);

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief Evaluates the velocity of the COM in the world frame.
    /// @param[in] _qDotSkel
    void evalVelocity(const Eigen::VectorXd& _qDotSkel);

    /// @brief Evaluates the Omega in the world frame.
    /// @param[in] _qDotSkel
    void evalOmega(const Eigen::VectorXd& _qDotSkel);

    /// @brief Evaluates the mass matrix mM.
    void evalMassMatrix();

    /// @brief Evaluates the Coriolis matrix mC.
    /// @param[in] _qDotSkel
    void evalCoriolisMatrix(const Eigen::VectorXd& _qDotSkel);

    /// @brief Evaluates the Coriolis vector mCvec directy: i.e. shortcut for
    /// mC*qdot.
    /// @param[in] _qDotSkel
    void evalCoriolisVector(const Eigen::VectorXd& _qDotSkel);

    /// @brief Evaluates the gravity vector mG in the generalized coordinates.
    /// @param[in] _gravity
    void evalGravityVector(const Eigen::Vector3d& _gravity);

    /// @brief Evaluates the external forces mFext in the generalized
    /// coordinates as J^TF.
    /// @param[out] _extForce
    void evalExternalForces(Eigen::VectorXd& _extForce);

    /// @brief Evaluates the external forces mFext in the generalized
    /// coordinates recursively.
    /// @param[out] _extForce External force.
    void evalExternalForcesRecursive(Eigen::VectorXd& _extForce);
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    /// @brief Convert cartesian forces in joint frame to generalized forces.
    /// @param[in] _cForce
    /// @param[out] _gForce
    /// @param[in] _isTorque
    void jointCartesianToGeneralized(const Eigen::Vector3d& _cForce,
                                     Eigen::VectorXd& _gForce,
                                     bool _isTorque = true);

    /// @brief Convert cartesian forces in body com frame to generalized forces.
    /// @param[in] _cForce
    /// @param[out] _gForce
    /// @param[in] _isTorque
    void bodyCartesianToGeneralized(const Eigen::Vector3d& _cForce,
                                    Eigen::VectorXd& _gForce,
                                    bool _isTorque = true);

    /// @brief Convert coriolis forces in cartesian space to generalized forces.
    void getGeneralized( Eigen::VectorXd& _gForce );
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // Add functions to add to the existing *full* matrices typically
    // for the entire skeleton
    //--------------------------------------------------------------------------
    /// @brief
    void aggregateMass(Eigen::MatrixXd &_M);

    /// @brief
    /// @param[out] _C
    void aggregateCoriolis(Eigen::MatrixXd &_C);

    /// @brief
    /// @param[out] _Cvec
    void aggregateCoriolisVec(Eigen::VectorXd &_Cvec);

    /// @brief
    /// @param[out] _G
    void aggregateGravity(Eigen::VectorXd &_G);

    //--------------------------------------------------------------------------
    // Add and remove contact points where forces are applied
    //--------------------------------------------------------------------------
    /// @brief Apply linear Cartesian forces to this node.
    /// @param[in] _offset
    /// @param[in] _force
    /// @param[in] _isOffsetLocal
    /// @param[in] _isForceLocal
    ///
    /// A force is defined by a point of application and a force vector. The
    /// last two parameters specify frames of the first two parameters.
    /// Coordinate transformations are applied when needed. The point of
    /// application and the force in local coordinates are stored in mContacts.
    /// When conversion is needed, make sure the transformations are avaialble.
    void addExtForce(const Eigen::Vector3d& _offset,
                     const Eigen::Vector3d& _force,
                     bool _isOffsetLocal = true,
                     bool _isForceLocal = false );

    /// @brief Apply Cartesian torque to the node. The torque in local
    /// coordinates is accumulated in mExtTorqueBody.
    /// @param[in] _torque
    /// @param[in] _isLocal
    void addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal);

    /// @brief Clean up structures that store external forces: mContacts, mFext,
    /// mExtForceBody and mExtTorqueBody.
    ///
    /// Called from @SkeletonDynamics::clearExternalForces.
    void clearExternalForces();
    //--------------------------------------------------------------------------

    /// @brief
    /// @return
    Eigen::Vector3d evalLinMomentum();

    /// @brief
    /// @param[in] _pivot
    /// @return
    Eigen::Vector3d evalAngMomentum(const Eigen::Vector3d& _pivot);

    /// @brief
    /// @param[in] _qIndex
    /// @return
    inline Eigen::MatrixXd getJvDeriv(int _qIndex) const
    { return mJvq[_qIndex]; }

    /// @brief Evaluate the first derivatives of the linear Jacobian w.r.t. to
    /// the dependent dofs.
    /// @param[in] _qi
    /// @param[in] _offset
    void evalJacDerivLin(int _qi, const Eigen::Vector3d& _offset);

    /// @brief Evaluate the first derivatives of the angular Jacobian w.r.t. to
    /// the dependent dofs.
    /// @param[in] _qi
    void evalJacDerivAng(int _qi);

    /// @brief Evaluate time derivative of the linear Jacobian of this body node
    /// (num cols == num dependent dofs).
    /// @param[in] _qDotSkel
    void evalJacDotLin(const Eigen::VectorXd& _qDotSkel);

    /// @brief Evaluate time derivative of the angular Jacobian of this body
    /// node (num cols == num dependent dofs).
    /// @param[in] _qDotSkel
    void evalJacDotAng(const Eigen::VectorXd& _qDotSkel);

    /// @brief
    /// @param[in] _q1
    /// @param[in] _q2
    /// @return
    Eigen::Matrix4d getLocalSecondDeriv(const kinematics::Dof* _q1,
                                        const kinematics::Dof* _q2) const;

    /// @brief
    /// @return
    const Eigen::Vector3d& getCurrentVelBody() const { return mVelBody; }

    /// @brief
    /// @return
    const Eigen::Vector3d& getCurrentVelDotBody() const { return mVelDotBody; }
protected:
    //--------------------------------------------------------------------------
    // Inverse Dynamics
    // TODO: These values are used only internally?
    //--------------------------------------------------------------------------
    /// @brief Jacobian matrix for the parent joint.
    /// TODO: what is the reference frame.
    Eigen::MatrixXd mJwJoint;

    /// @brief Time derivative of the Jacobian matrix for the parent joint.
    Eigen::MatrixXd mJwDotJoint;

    /// @brief Linear velocity expressed in the *local frame* of the body.
    Eigen::Vector3d mVelBody;

    /// @brief Linear acceleration expressed in the *local frame* of the body.
    Eigen::Vector3d mVelDotBody;

    /// @brief Angular velocity expressed in the *local frame* of the body.
    Eigen::Vector3d mOmegaBody;

    /// @brief Angular acceleration expressed in the *local frame* of the body.
    Eigen::Vector3d mOmegaDotBody;

    /// @brief The constraint joint force in Cartesian coordinates, expressed in
    /// the local frame of the body instead of the joint.
    Eigen::Vector3d mForceJointBody;

    /// @brief The torque in Cartesian coordinates for the joint expressed in
    /// the local frame of the body instead of the joint.
    Eigen::Vector3d mTorqueJointBody;

    /// @brief The external Cartesian force applied to the body.
    /// Usually computed from mContacts.
    Eigen::Vector3d mExtForceBody;

    /// @brief The external Cartesian torque applied to the body.
    /// Usually directly supplied from outside.
    /// Contribution of the linear force will be considered later in the
    /// computation.
    Eigen::Vector3d mExtTorqueBody;

    //--------------------------------------------------------------------------
    // non-recursive Dynamics formulation - M*qdd + C*qdot + g = 0
    //--------------------------------------------------------------------------
public: // TODO: member function should be accessed by member function
    /// @brief Linear velocity in the world frame.
    Eigen::Vector3d mVel;

    /// @brief Angular velocity in the world frame.
    Eigen::Vector3d mOmega;
protected:
    /// @brief Mass matrix of dimension numDependentDofs x numDependentDofs.
    /// To be added carefully to the skeleton mass matrix.
    Eigen::MatrixXd mM;

    /// @brief Coriolis matrix of dimension numDependentDofs x numDependentDofs.
    /// To be added carefully to the skeleton Coriolis matrix.
    Eigen::MatrixXd mC;

    /// @brief Coriolis vector of dimension numDependentDofs x 1
    /// mCvec = mC*qdot.
    Eigen::VectorXd mCvec;

    /// @brief Gravity vector or generalized gravity forces.
    /// Dimension numDependentDofs x 1.
    Eigen::VectorXd mG;

    /// @brief Generalized external forces this node contributes: J^TF.
    /// Dimension numDependentDofs x 1.
    Eigen::VectorXd mFext;
    //--------------------------------------------------------------------------

    /// @brief true if linear inverse dynamics is initialized.
    ///
    /// Init functions initialize only if false.
    bool mInitializedInvDyn;

    /// @brief True if non recursive dynamics is initialized.
    ///
    /// Init function initialize only if false.
    bool mInitializedNonRecursiveDyn;

    //--------------------------------------------------------------------------
    // Non-recursive Dynamics formulation - second derivatives
    //--------------------------------------------------------------------------
    /// @brief Partial derivative of local transformation wrt local dofs.
    ///
    /// Each element is a 4x4 matrix.
    EIGEN_VV_MAT4D mTqq;

    /// @brief Partial derivative of world transformation wrt all dependent dofs
    ///
    /// Each element is a 4x4 matrix.
    EIGEN_VV_MAT4D mWqq;

    /// @brief Linear Jacobian derivative w.r.t. to all the dependent dofs.
    std::vector<Eigen::MatrixXd> mJvq;

    /// @brief Angular Jacobian derivative w.r.t. to all the dependent dofs.
    std::vector<Eigen::MatrixXd> mJwq;

    /// @brief Time derivative of the Linear velocity Jacobian.
    Eigen::MatrixXd mJvDot;

    /// @brief Time derivative of the Angular velocity Jacobian.
    Eigen::MatrixXd mJwDot;
    //--------------------------------------------------------------------------

    /// @brief List of contact points where external forces are applied.
    ///
    /// Contact points are a pair of (local point offset, Cartesian force in
    /// local coordinates)
    std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > mContacts;

    /// @brief If the gravity mode is false, this body node does not
    /// being affected by gravity.
    /// TODO: Not implemented yet!
    bool mGravityMode;
};

} // namespace dynamics
} // namespace rtql8

#endif // #ifndef DYNAMICS_BODYNODE_DYNAMICS_H
