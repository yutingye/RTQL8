/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumitj83@gmail.com>, Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 03/28/2013
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

#ifndef KINEMATICS_BODYNODE_H
#define KINEMATICS_BODYNODE_H

#include <vector>
#include <Eigen/Dense>
#include "renderer/RenderInterface.h"
#include "utils/EigenHelper.h"

namespace rtql8
{
namespace kinematics
{

#define MAX_NODE3D_NAME 128

class Marker;
class Dof;
class Transformation;
class Shape;
class Skeleton;
class Joint;

/// @brief BodyNode class represents a single node of the skeleton.
///
/// BodyNode is a basic element of the skeleton. BodyNodes are
/// hierarchically connected and have a set of core functions for
/// calculating derivatives. Mostly automatically constructed by
/// FileInfoSkel.
/// @see FileInfoSkel.
class BodyNode
{
public:
    // we need this aligned allocator because we have Matrix4d as
    // members in this class
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @brief Default constructor.
    /// @param[in] _name The name can be up to 128.
    BodyNode(const char* _name = NULL);

    /// @brief Default destructor.
    virtual ~BodyNode();

    /// @brief Initialize the vector memebers with proper sizes.
    void init();

    /// @brief Update transformations, mT and mW, w.r.t. the current dof
    /// values in Dof*.
    void updateTransform();

    /// @brief Update the first derivatives of the transformations
    void updateFirstDerivatives();

    /// @brief Evaluate linear Jacobian of this body node (num cols ==
    /// num dependent dofs)
    void evalJacLin();

    /// @brief Evaluate angular Jacobian of this body node (num cols ==
    /// num dependent dofs)
    void evalJacAng();

    /// @brief
    /// @param[in] _W
    inline void setWorldTransform(const Eigen::Matrix4d& _W)
    { mW = _W; }

    /// @brief Transformation from the local coordinates of this body
    /// node to the world coordinates.
    /// @return
    inline Eigen::Matrix4d getWorldTransform() const { return mW; }

    /// @brief Transformation from the world coordinates to the local
    /// coordinates of this body node.
    /// @return
    inline Eigen::Matrix4d getWorldInvTransform() const { return mW.inverse(); }

    /// @brief Transformation from the local coordinates of this body
    /// node to the local coordinates of its parent.
    /// @return
    inline Eigen::Matrix4d getLocalTransform() const { return mT; }

    /// @brief Transformation from the local coordinates of the parent
    /// node to the local coordinates of this body node.
    /// @return
    inline Eigen::Matrix4d getLocalInvTransform() const { return mT.inverse(); }

    /// @brief Given a 3D vector lp in the local coordinates of this
    /// body node, return the world coordinates of this vector.
    /// @param[in] _lp
    /// @return
    Eigen::Vector3d evalWorldPos(const Eigen::Vector3d& _lp);

    /// @brief First derivative of the local transformation w.r.t. the
    /// input dof.
    /// @param[in] _q
    /// @return
    Eigen::Matrix4d getLocalDeriv(const Dof* _q) const;

    //            /// @brief Set up the dof dependence map for this node.
    //            void setDependDofMap(int _numDofs);

    //            /// @brief
    //            /// @note Not checking index range
    //            bool dependsOn(int _dofIndex) const
    //            { return mDependsOnDof[_dofIndex]; }

    /// @brief Set up the list of dependent dofs.
    void setDependDofList();

    /// @brief Test whether this dof is dependent or not.
    /// @param[in] _dofIndex
    /// @return
    /// @warning You may want to use getNumDependentDofs /
    /// getDependentDof for efficiency.
    bool dependsOn(int _dofIndex) const;

    /// @brief The number of the dofs by which this node is affected.
    /// @return
    inline int getNumDependentDofs() const { return mDependentDofs.size(); }

    /// @brief Return an dof index from the array index
    /// (< getNumDependentDofs).
    /// @param[in] _arrayIndex
    /// @return
    inline int getDependentDof(int _arrayIndex)
    { return mDependentDofs[_arrayIndex]; }

    /// @brief Render the entire subtree rooted at this body node.
    /// @param[in] _ri
    /// @param[in] _color
    /// @param[in] _useDefaultColor
    /// @param[in] _depth
    void draw(renderer::RenderInterface* _ri = NULL,
              const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
              bool _useDefaultColor = true,
              int _depth = 0) const;

    /// @brief Render the markers.
    /// @param[in] _ri
    /// @param[in] _color
    /// @param[in] _useDefaultColor
    void drawMarkers(
            renderer::RenderInterface* _ri = NULL,
            const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
            bool _useDefaultColor = true) const ;

    /// @brief
    /// @return
    inline char* getName() { return mName; }

    /// @brief
    /// @param[in] _off
    inline void setLocalCOM(const Eigen::Vector3d& _off) { mCOMLocal = _off; }

    /// @brief
    /// @return
    inline Eigen::Vector3d getLocalCOM() const { return mCOMLocal; }

    /// @brief
    /// @return
    inline Eigen::Vector3d getWorldCOM()
    { return evalWorldPos(mCOMLocal); }

    /// @brief
    /// @param[in] _skel
    inline void setSkel(Skeleton* _skel) { mSkel = _skel; }

    /// @brief
    /// @return
    inline Skeleton* getSkel() const { return mSkel; }

    /// @brief
    /// @param[in] _idx
    inline void setSkelIndex(int _idx) { mSkelIndex = _idx; }

    /// @brief
    inline int getSkelIndex() const { return mSkelIndex; }

    /// @brief
    /// @return
    inline BodyNode* getParentNode() const { return mNodeParent; }

    /// @brief
    /// @return
    inline double getMass() const { return mMass; }

    /// @brief
    /// @return
    inline Eigen::Matrix3d getInertia() const { return mIc; }

    /// @brief
    /// @param[in] _h
    inline void addMarker(Marker *_h) { mMarkers.push_back(_h); }

    /// @brief
    /// @return
    inline int getNumMarkers() const { return mMarkers.size(); }

    /// @brief
    /// @param[in] _idx
    /// @return
    inline Marker* getMarker(int _idx) const { return mMarkers[_idx]; }

    /// @brief
    /// @param[in] _newShape
    inline void setVisualShape(Shape* _newShape) { mVisualShape = _newShape; }

    /// @brief
    inline Shape* getVisualShape() const { return mVisualShape; }

    /// @brief
    /// @param[in] _newShape
    inline void setCollisionShape(Shape* _newShape)
    { mCollisionShape = _newShape; }

    /// @brief
    /// @return
    inline Shape* getCollisionShape() const { return mCollisionShape; }

    /// @brief
    /// @param[in] _c
    inline void addChildJoint(Joint *_c) { mJointsChild.push_back(_c); }

    /// @brief
    /// @return
    inline int getNumChildJoints() { return mJointsChild.size(); }

    /// @brief
    /// @param[in] _idx
    /// @return
    inline Joint* getChildJoint(int _idx) const
    { return mJointsChild[_idx]; }

    /// @brief
    /// @return
    inline Joint* getParentJoint() const { return mJointParent; }

    /// @brief
    /// @param[in] _p
    void setParentJoint(Joint* _p);

    //------------------------------------------------------------------
    // wrapper functions for joints
    //------------------------------------------------------------------
    /// @brief
    /// @param[in] _idx
    /// @return
    BodyNode* getChildNode(int _idx) const;

    /// @brief
    /// @return
    int getNumLocalDofs() const;

    /// @brief
    /// @param[in] _idx
    /// @return
    Dof* getDof(int _idx) const;

    /// @brief
    /// @param[in] _q
    /// @return
    bool isPresent(Dof *_q);
    //------------------------------------------------------------------

    /// @brief
    /// @param[in] _index
    /// @return
    Eigen::Matrix4d getDerivLocalTransform(int _index) const;

    /// @brief
    /// @param[in] _index
    /// @return
    Eigen::Matrix4d getDerivWorldTransform(int _index) const;

    /// @brief
    /// @return
    Eigen::MatrixXd getJacobianLinear() const;

    /// @brief
    /// @return
    Eigen::MatrixXd getJacobianAngular() const;

    /// @brief
    /// @return
    inline bool getCollideState() const { return mCollidable; }

    /// @brief
    /// @param[in] _c
    inline void setCollideState(bool _c) { mCollidable = _c; }

protected:
    /// @brief Name.
    char mName[MAX_NODE3D_NAME];

    /// @brief Index in the model.
    int mSkelIndex;

    /// @brief Geometry for visualization.
    Shape* mVisualShape;

    /// @brief Geometry for collision detection.
    Shape* mCollisionShape;

    /// @brief List of joints that link to child nodes.
    std::vector<Joint *> mJointsChild;

    /// @brief Joint connecting to parent node.
    Joint* mJointParent;

    /// @brief Parent node.
    BodyNode* mNodeParent;

    /// @brief List of markers associated.
    std::vector<Marker*> mMarkers;

    /// @brief A list of dependent dof indices.
    std::vector<int> mDependentDofs;

    /// @brief keep track of the root translation DOFs only if they are
    /// the first ones.
    int mNumRootTrans;

    /// @brief Mass of this node; zero if no primitive.
    double mMass;

    /// @brief COM of this body node in its local coordinate frame.
    Eigen::Vector3d mCOMLocal;

    /// @brief Pointer to the model this body node belongs to.
    Skeleton* mSkel;

    //------------------------------------------------------------------
    // transformations
    //------------------------------------------------------------------
    /// @brief Local transformation from parent to itself.
    Eigen::Matrix4d mT;

    /// @brief Global transformation.
    Eigen::Matrix4d mW;

    /// @brief Inertia matrix in the world frame = R*Ibody*RT.
    ///
    /// Updated by evalTransform.
    Eigen::Matrix3d mIc;
    //------------------------------------------------------------------

    //------------------------------------------------------------------
    // first derivatives
    //------------------------------------------------------------------
    /// @brief Partial derivative of local transformation w.r.t. local
    /// dofs.
    ///
    /// Each element is a 4x4 matrix.
    EIGEN_V_MAT4D mTq;

    /// @brief Partial derivative of world transformation w.r.t. all
    /// dependent dofs.
    ///
    /// Each element is a 4x4 matrix.
    EIGEN_V_MAT4D mWq;

    /// @brief Linear Jacobian; Cartesian_linear_velocity of the COM =
    /// mJv * generalized_velocity.
    Eigen::MatrixXd mJv;

    /// @brief Angular Jacobian; Cartesian_angular_velocity = mJw *
    /// generalized_velocity.
    Eigen::MatrixXd mJw;
    //------------------------------------------------------------------

private:
    /// @brief A unique ID of this node globally.
    int mID;

    /// @brief Counts the number of nodes globally.
    static int msBodyNodeCount;

    /// @brief
    bool mCollidable;
};

} // namespace kinematics
} // namespace rtql8

#endif // #ifndef KINEMATICS_BODYNODE_H
