/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#ifndef OPTIMIZER_CONSTRAINT_BOX_H
#define OPTIMIZER_CONSTRAINT_BOX_H

#include <vector>

namespace rtql8 {
    namespace optimizer {
        class Constraint;

        class ConstraintBox {
        public:
            ConstraintBox(int numDofs);
            virtual ~ConstraintBox();

            void add(Constraint *newConstraint);
            void clear();
            int remove(Constraint *target);
            int isInBox(Constraint *testConstraint);	//return index of component if true

            int getNumConstraints() const { return mConstraints.size(); }
            Constraint * getConstraint(int index) const { return mConstraints[index]; }

            void evalJac();
            void evalCon();
            void reallocateMem();

            //Must be called before using ConstraintBox
            int getNumDofs() const { return mNumDofs; }
            void setNumDofs(int numDofs);
            int getNumTotalRows() const { return mNumTotalRows; }

            int mNumDofs; //number of Model DOFs
            int mNumTotalRows;
            std::vector<Constraint *> mConstraints;
            std::vector<double> mCon;
            std::vector< std::vector<double> *> mJac; //Jacobian
            std::vector< std::vector<bool> *> mJacMap; //Show nonzero elements of Jacobian
        };
    } // namespace optimizer
} // namespace rtql8

#endif // #ifndef OPTIMIZER_CONSTRAINT_BOX_H

