/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>, Kristin Siu <kasiu@gatech.edu>,
 *            Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 03/26/2013
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

#ifndef DYNAMICS_CONTACT_DYNAMICS_H
#define DYNAMICS_CONTACT_DYNAMICS_H

#include <vector>
#include <Eigen/Dense>

namespace rtql8
{
    namespace kinematics
    {
        class BodyNode;
    } // namespace kinematics
} // namespace rtql8

namespace rtql8
{
    namespace collision_checking
    {
        class SkeletonCollision;
    } // namespace collision
} // namespace rtql8

namespace rtql8
{
    namespace lcpsolver
    {
        class LCPSolver;
    } //namespace lcpsolver
} // namespace rtql8

namespace rtql8
{
    namespace dynamics
    {
        class SkeletonDynamics;

        /// @class ContactDynamics
        /// @brief
        ///
        /// [Sample Usage]
        ///
        /// dynamics::ContactDynamics contacts(skels, dt);
        ///
        /// // Call this function after all other external and internal forces
        /// // are computed
        /// contacts.applyContactForces();
        ///
        class ContactDynamics
        {
        public:
            /// @brief
            ContactDynamics(const std::vector<SkeletonDynamics*>& _skels,
                            double _dt,
                            double _mu = 1.0,
                            int _d = 4);

            /// @brief
            virtual ~ContactDynamics();

            /// @brief
            void applyContactForces();

            /// @brief
            void reset();

            /// @brief
            void addSkeleton(SkeletonDynamics* _newSkel);

            /// @brief
            inline Eigen::VectorXd getConstraintForce(int _skelIndex) const
            { return mConstrForces[_skelIndex]; }

            /// @brief
            inline collision_checking::SkeletonCollision* getCollisionChecker() const
            {return mCollisionChecker; }

            /// @brief
            int getNumContacts() const;

        private:
            /// @brief
            void initialize();

            /// @brief
            void destroy();

            /// @brief
            void updateMassMat();

            /// @brief
            void updateTauStar();

            /// @brief
            void fillMatrices();

            /// @brief
            bool solve();

            /// @brief
            void applySolution();

            /// @brief
            inline int getNumSkels() const { return mSkels.size(); }

            /// @brief
            inline int getNumTotalDofs() const
            { return mIndices[mIndices.size() - 1]; }

            /// @brief
            inline int getNumContactDirections() const { return mNumDir; }

            /// @brief
            Eigen::MatrixXd getJacobian(kinematics::BodyNode* node,
                                        const Eigen::Vector3d& p);

            /// @brief Helper functions to compute all of the matrices.
            ///
            /// Notation is similar to that used in derivation:
            /// Mqddot + Cqdot + kq = tau + (J^T)(f_n)N + (J^T)D(f_d) -> Mqdot = tau* + N(f_n) + B(f_d)
            ///        inline Eigen::MatrixXd getMassMatrix() const { return mM; } // M matrix
            ///        inline Eigen::VectorXd getTauStarVector() const { return mTauStar; } // T* vector (not T)
            ///        void updateNormalMatrix(); // N matrix
            ///        void updateBasisMatrix() ; // B matrix
            void updateNBMatrices();

            /// @brief Gets a matrix of tangent dirs.
            Eigen::MatrixXd getTangentBasisMatrix(const Eigen::Vector3d& p,
                                                  const Eigen::Vector3d& n);

            /// @brief E matrix.
            Eigen::MatrixXd getContactMatrix() const;

            /// @brief mu matrix.
            Eigen::MatrixXd getMuMatrix() const;

            /// @brief
            std::vector<SkeletonDynamics*> mSkels;

            /// @brief
            std::vector<int> mBodyIndexToSkelIndex;

            /// @brief
            std::vector<int> mIndices;

            /// @brief
            collision_checking::SkeletonCollision* mCollisionChecker;

            /// @brief Timestep.
            double mDt;

            /// @brief Friction coeff.
            double mMu;

            /// @brief Number of basis directions.
            int mNumDir;

            //------------------------------------------------------------------
            // Cached (aggregated) mass/tau matrices
            //------------------------------------------------------------------
            /// @brief
            Eigen::MatrixXd mMInv;

            /// @brief
            Eigen::VectorXd mTauStar;

            /// @brief
            Eigen::MatrixXd mN;

            /// @brief
            Eigen::MatrixXd mB;

            //------------------------------------------------------------------
            // Matrices to pass to solver
            //------------------------------------------------------------------
            /// @brief
            Eigen::MatrixXd mA;

            /// @brief
            Eigen::VectorXd mQBar;

            /// @brief
            Eigen::VectorXd mX;

            /// @brief Solved constraint force in generalized coordinates.
            /// mConstrForces[i] is the constraint force for the ith skeleton.
            std::vector<Eigen::VectorXd> mConstrForces;
        };

    } // namespace dynamics
} // namespace rtql8

#endif // #ifndef DYNAMICS_CONTACT_DYNAMICS_H
