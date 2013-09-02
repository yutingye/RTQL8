/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>
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

#include "ClosedLoopConstraint.h"

#include "kinematics/BodyNode.h"
#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace rtql8::utils;

namespace rtql8 {
    namespace dynamics {
        ClosedLoopConstraint::ClosedLoopConstraint(BodyNodeDynamics *_body1, BodyNodeDynamics *_body2, Vector3d _offset1, Vector3d _offset2, int _skelIndex1, int _skelIndex2) {
            mBody1 = _body1;
            mBody2 = _body2;
            mOffset1 = _offset1;
            mOffset2 = _offset2;
            mJ1 = MatrixXd::Zero(3, mBody1->getSkel()->getNumDofs());
            mJ2 = MatrixXd::Zero(3, mBody2->getSkel()->getNumDofs());
            mNumRows = 3;
            mSkelIndex1 = _skelIndex1;
            mSkelIndex2 = _skelIndex2;
        }

        ClosedLoopConstraint::~ClosedLoopConstraint() {
        }
        
        void ClosedLoopConstraint::updateDynamics(std::vector<Eigen::MatrixXd> & _J, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
            getJacobian();
            _J[mSkelIndex2].block(_rowIndex, 0, 3, mBody2->getSkel()->getNumDofs()).setZero();
            _J[mSkelIndex1].block(_rowIndex, 0, 3, mBody1->getSkel()->getNumDofs()) = mJ1;
            _J[mSkelIndex2].block(_rowIndex, 0, 3, mBody2->getSkel()->getNumDofs()) += mJ2;

            Vector3d worldP1 = xformHom(mBody1->getWorldTransform(), mOffset1);
            Vector3d worldP2 = xformHom(mBody2->getWorldTransform(), mOffset2);
            VectorXd qDot1 = ((SkeletonDynamics*)mBody1->getSkel())->getQDotVector();
            VectorXd qDot2 = ((SkeletonDynamics*)mBody2->getSkel())->getQDotVector();
            _C.segment(_rowIndex, 3) = worldP1 - worldP2;
            _CDot.segment(_rowIndex, 3) = mJ1 * qDot1 + mJ2 * qDot2;
        }

        void ClosedLoopConstraint::getJacobian() {
            for(int i = 0; i < mBody1->getNumDependentDofs(); i++) {
                int dofIndex = mBody1->getDependentDof(i);
                VectorXd Jcol = xformHom(mBody1->getDerivWorldTransform(i), mOffset1);
                mJ1.col(dofIndex) = Jcol;
            }
            for(int i = 0; i < mBody2->getNumDependentDofs(); i++) {
                int dofIndex = mBody2->getDependentDof(i);
                VectorXd Jcol = xformHom(mBody2->getDerivWorldTransform(i), mOffset2);
                mJ2.col(dofIndex) = -Jcol;
            }
        }
    } // namespace dynamics
} // namespace rtql8

