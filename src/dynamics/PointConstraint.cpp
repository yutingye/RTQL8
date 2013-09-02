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

#include "PointConstraint.h"

#include "kinematics/BodyNode.h"
#include "SkeletonDynamics.h"
#include "BodyNodeDynamics.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace rtql8::utils;

namespace rtql8 {
    namespace dynamics {
        PointConstraint::PointConstraint(BodyNodeDynamics *_body, Vector3d _offset, Vector3d _target, int _skelIndex) {
            mBody = _body;
            mOffset = _offset;
            mTarget = _target;
            mSkelIndex = _skelIndex;
            mJ = MatrixXd::Zero(3, mBody->getSkel()->getNumDofs());
            mNumRows = 3;
        }

        PointConstraint::~PointConstraint() {
        }
        
        void PointConstraint::updateDynamics(std::vector<Eigen::MatrixXd> & _J, Eigen::VectorXd & _C, Eigen::VectorXd & _CDot, int _rowIndex) {
            getJacobian();
            SkeletonDynamics *skel = (SkeletonDynamics*)mBody->getSkel();
            _J[mSkelIndex].block(_rowIndex, 0, 3, skel->getNumDofs()) = mJ;
            Vector3d worldP = xformHom(mBody->getWorldTransform(), mOffset);
            VectorXd qDot = skel->getQDotVector();
            _C.segment(_rowIndex, 3) = worldP - mTarget;
            _CDot.segment(_rowIndex, 3) = mJ * qDot;
        }

        void PointConstraint::getJacobian() {
            for(int i = 0; i < mBody->getNumDependentDofs(); i++) {
                int dofIndex = mBody->getDependentDof(i);
                VectorXd Jcol = xformHom(mBody->getDerivWorldTransform(i), mOffset);
                mJ.col(dofIndex) = Jcol;
            }
        }
    } // namespace dynamics
} // namespace rtql8

