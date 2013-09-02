/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>, Yuting Ye <yuting.ye@gmail.com>
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

#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "kinematics/Joint.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Dof.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace rtql8::kinematics;

namespace rtql8 {
    namespace dynamics {
        SkeletonDynamics::SkeletonDynamics():
            kinematics::Skeleton(), mImmobile(false) {
        }

        SkeletonDynamics::~SkeletonDynamics() {
        }

        void SkeletonDynamics::initDynamics() {
            mM = MatrixXd::Zero(getNumDofs(), getNumDofs());
            mC = MatrixXd::Zero(getNumDofs(), getNumDofs());
            mQdot = VectorXd::Zero(getNumDofs());
            mCvec = VectorXd::Zero(getNumDofs());
            mG = VectorXd::Zero(getNumDofs());
            mCg = VectorXd::Zero(getNumDofs());
            mFint = VectorXd::Zero(getNumDofs());
            mFext = VectorXd::Zero(getNumDofs());
        }

        kinematics::BodyNode* SkeletonDynamics::createBodyNode(
                const char* const _name) {
            return new BodyNodeDynamics(_name);
        }

        // computes the C term and gravity, excluding external forces
        VectorXd SkeletonDynamics::computeInverseDynamicsLinear(
                const Vector3d &_gravity, const VectorXd *_qdot,
                const VectorXd *_qdotdot, bool _computeJacobians,
                bool _withExternalForces ) {
            // FORWARD PASS: compute the velocities recursively - from root to
            // end effectors.
            // increasing order ensures that the parent joints/nodes are
            // evaluated before the child.
            for (int i = 0; i < getNumNodes(); i++) {
                BodyNodeDynamics *nodei
                        = static_cast<BodyNodeDynamics*>(getNode(i));

                // init the node in the first pass
                nodei->initInverseDynamics();

                // compute the velocities
                nodei->computeInvDynVelocities(
                            _gravity, _qdot, _qdotdot, _computeJacobians);
            }

            VectorXd torqueGen = VectorXd::Zero(getNumDofs());
            // BACKWARD PASS: compute the forces recursively - from end
            // effectors to root.
            // decreasing order ensures that the parent joints/nodes are
            // evaluated after the child.
            for (int i = getNumNodes() - 1; i >= 0; i--) {
                BodyNodeDynamics *nodei
                        = static_cast<BodyNodeDynamics*>(getNode(i));

                // compute joint forces in cartesian space
                nodei->computeInvDynForces(_gravity, _qdot, _qdotdot,
                                           _withExternalForces);

                // convert joint forces to generalized coordinates
                nodei->getGeneralized(torqueGen);
            }

            if ( _withExternalForces )
                clearExternalForces();

            return torqueGen;
        }

        // after the computation, mM, mCg, and mFext are ready for use
        void SkeletonDynamics::computeDynamics(const Vector3d &_gravity,
                                               const VectorXd &_qdot,
                                               bool _useInvDynamics)
        {
            mM.setZero();
            //mC = MatrixXd::Zero(getNumDofs(), getNumDofs());
            mQdot.setZero();
            mCvec.setZero();
            mG.setZero();
            mCg.setZero();

            if (_useInvDynamics)
            {
                mCg = computeInverseDynamicsLinear(
                            _gravity, &_qdot, NULL, true, false);
                for (int i = 0; i < getNumNodes(); i++)
                {
                    BodyNodeDynamics *nodei
                            = static_cast<BodyNodeDynamics*>(getNode(i));
                    // mass matrix M
                    // assumes Jacobians mJv and mJw have been computed above:
                    // use flag as true in computeInverseDynamicsLinear
                    nodei->evalMassMatrix();
                    nodei->aggregateMass(mM);
                }

                mMInv = mM.inverse();
                //mCholesky.compute(mM);

                evalExternalForces( true );
                //mCg -= mFext;
                mQdot = _qdot;
            }
            else
            {
                // init the data structures for the dynamics
                for(int i=0; i<getNumNodes(); i++)
                {
                    BodyNodeDynamics *nodei
                            = static_cast<BodyNodeDynamics*>(getNode(i));
                    // initialize the data structures for storage
                    nodei->initDynamics();

                    // compute the transforms and the derivatives
                    nodei->updateTransform();
                    nodei->updateFirstDerivatives();
                    nodei->updateSecondDerivatives();
                    // compute the required data structures and add to the
                    // skel's data structures
                    // mass matrix M
                    nodei->evalMassMatrix();
                    nodei->aggregateMass(mM);
                    //// Coriolis C
                    //nodei->evalCoriolisMatrix(_qdot);
                    //nodei->addCoriolis(mC);
                    // Coriolis vector C*qd
                    nodei->evalCoriolisVector(_qdot);
                    nodei->aggregateCoriolisVec(mCvec);
                    // gravity vector G
                    nodei->evalGravityVector(_gravity);
                    nodei->aggregateGravity(mG);
                }

                mMInv = mM.inverse();
                //mCholesky.compute(mM);

                evalExternalForces( false );
                //mCg = mC*_qdot + mG;
                mQdot = _qdot;
                mCg = mCvec + mG; //- mFext;
            }

            clearExternalForces();
        }

        void SkeletonDynamics::evalExternalForces(bool _useRecursive)
        {
            mFext.setZero();
            int nNodes = getNumNodes();
            for (int i = nNodes-1; i >= 0; i--)
            { // recursive from child to parent
                BodyNodeDynamics *nodei
                        = static_cast<BodyNodeDynamics*>(getNode(i));
                if (_useRecursive)
                    nodei->evalExternalForcesRecursive( mFext );
                else
                    nodei->evalExternalForces( mFext );
            }
        }

        // assumptions made:
        // 1. the pose _q has already been set to the model
        // 2. first derivatives are up-to-date (i.e. consistent with _q )
        // XXX
        // @Warning: dof values and transformations are inconsistent after this
        // computation!
        void SkeletonDynamics::clampRotation(VectorXd& _q, VectorXd& _qdot)
        {
            for (unsigned int i = 0; i < mJoints.size(); i++) {
                Joint* jnt = mJoints.at(i);
                // only cares about euler and expmap of 3 rotations
                switch(jnt->getJointType()) {
                case Joint::J_FREEEULER:
                {
                    // clamp dof values to the range
                    for(int j=0; j<3; j++) {
                        int dofIndex = jnt->getDof(j + 3)->getSkelIndex();
                        if( _q[dofIndex]>M_PI )
                            _q[dofIndex] -= 2*M_PI;
                        else if( _q[dofIndex]<-M_PI )
                            _q[dofIndex] += 2*M_PI;
                    }
                }   break;
                case Joint::J_BALLEULER:
                {
                    // clamp dof values to the range
                    for(int j=0; j<3; j++) {
                        int dofIndex = jnt->getDof(j)->getSkelIndex();
                        if( _q[dofIndex]>M_PI ) _q[dofIndex] -= 2*M_PI;
                        else if( _q[dofIndex]<-M_PI ) _q[dofIndex] += 2*M_PI;
                    }
                }   break;
                case Joint::J_FREEEXPMAP:
                {
                    Vector3d exmap;
                    for (int j = 0; j < 3; j++) {
                        int dofIndex = jnt->getDof(j + 3)->getSkelIndex();
                        exmap[j] = _q[dofIndex];
                    }

                    double theta = exmap.norm();
                    if (theta > M_PI) {
                        exmap.normalize();
                        exmap *= theta-2*M_PI;

                        BodyNode* node = jnt->getChildNode();
                        int firstIndex = 0;
                        if (jnt->getParentNode() != NULL)
                            firstIndex = jnt->getParentNode()->getNumDependentDofs();

                        // extract the local Jw
                        Matrix3d oldJwBody;
                        for (int j = 0; j < 3; j++) {
                            // XXX do not use node->mTq here because it's not computed if the recursive algorithm is used; instead the derivative matrix is (re)computed explicitly.
                            Matrix3d omegaSkewSymmetric = jnt->getDeriv(jnt->getDof(j + 3)).topLeftCorner(3,3) * node->getLocalTransform().topLeftCorner(3,3).transpose();
                            oldJwBody.col(j) = utils::fromSkewSymmetric(omegaSkewSymmetric);
                        }

                        // the new Jw
                        Matrix3d newJwBody;
                        // set new dof values to joint for derivative evaluation
                        for (int j = 0; j < 3; j++)
                            jnt->getDof(j + 3)->setValue(exmap[j]);

                        // compute the new Jw from Rq*trans(R)
                        Matrix4d Tbody = jnt->getLocalTransform();
                        for (int j = 0; j < 3; j++) {
                            Matrix3d omegaSkewSymmetric = jnt->getDeriv(jnt->getDof(j + 3)).topLeftCorner(3,3)*Tbody.topLeftCorner(3,3).transpose();
                            newJwBody.col(j) = utils::fromSkewSymmetric(omegaSkewSymmetric);
                        }

                        // solve new_qdot s.t. newJw*new_qdot = w
                        // new_qdot = newJw.inverse()*w
                        Vector3d old_qdot; // extract old_qdot
                        for (int j = 0; j < 3; j++)
                            old_qdot[j] = _qdot[jnt->getDof(j + 3)->getSkelIndex()];
                        Vector3d new_qdot = newJwBody.inverse()*oldJwBody*old_qdot;
                        for (int j = 0; j < 3; j++) {
                            int dofIndex = jnt->getDof(j + 3)->getSkelIndex();
                            _q[dofIndex] = exmap[j];
                            _qdot[dofIndex] = new_qdot[j];
                        }
                    }
                }   break;


                case Joint::J_BALLEXPMAP:
                {
                    Vector3d exmap;
                    for (int j = 0; j < 3; j++) {
                        int dofIndex = jnt->getDof(j)->getSkelIndex();
                        exmap[j] = _q[dofIndex];
                    }

                    double theta = exmap.norm();
                    if (theta > M_PI) {
                        exmap.normalize();
                        exmap *= theta-2*M_PI;

                        BodyNode* node = jnt->getChildNode();
                        int firstIndex = 0;
                        if (jnt->getParentNode() != NULL)
                            firstIndex = jnt->getParentNode()->getNumDependentDofs();

                        // extract the local Jw
                        Matrix3d oldJwBody;
                        for (int j = 0; j < 3; j++) {
                            // XXX do not use node->mTq here because it's not computed if the recursive algorithm is used; instead the derivative matrix is (re)computed explicitly.
                            Matrix3d omegaSkewSymmetric = jnt->getDeriv(jnt->getDof(j)).topLeftCorner(3,3)
                                * node->getLocalTransform().topLeftCorner(3,3).transpose();
                            oldJwBody.col(j) = utils::fromSkewSymmetric(omegaSkewSymmetric);
                        }

                        // the new Jw
                        Matrix3d newJwBody;
                        // set new dof values to joint for derivative evaluation
                        for (int j = 0; j < 3; j++)
                            jnt->getDof(j)->setValue(exmap[j]);

                        // compute the new Jw from Rq*trans(R)
                        Matrix4d Tbody = jnt->getLocalTransform();
                        for (int j = 0; j < 3; j++) {
                            Matrix3d omegaSkewSymmetric = jnt->getDeriv(jnt->getDof(j)).topLeftCorner(3,3)*Tbody.topLeftCorner(3,3).transpose();
                            newJwBody.col(j) = utils::fromSkewSymmetric(omegaSkewSymmetric);
                        }

                        // solve new_qdot s.t. newJw*new_qdot = w
                        // new_qdot = newJw.inverse()*w
                        Vector3d old_qdot; // extract old_qdot
                        for (int j = 0; j < 3; j++)
                            old_qdot[j] = _qdot[jnt->getDof(j)->getSkelIndex()];
                        Vector3d new_qdot = newJwBody.inverse()*oldJwBody*old_qdot;
                        for (int j = 0; j < 3; j++) {
                            int dofIndex = jnt->getDof(j)->getSkelIndex();
                            _q[dofIndex] = exmap[j];
                            _qdot[dofIndex] = new_qdot[j];
                        }
                    }
                }   break;
                default:
                    break;
                }
            }
            mQdot = _qdot;
        }

        void SkeletonDynamics::clearExternalForces(){
            int nNodes = getNumNodes();
            for (int i = 0; i < nNodes; i++)
                ((BodyNodeDynamics*)mNodes.at(i))->clearExternalForces();
        }

    } // namespace dynamics
} // namespace rtql8
