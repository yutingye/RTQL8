/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumitj83@gmail.com>, Sehoon Ha <sehoon.ha@gmail.com>
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

#include "Skeleton.h"

#include <cassert>
using namespace std;
using namespace Eigen;

#include "Dof.h"
#include "Joint.h"
#include "BodyNode.h"
#include "Marker.h"
#include "Transformation.h"
#include "utils/UtilsMath.h"

#include "renderer/RenderInterface.h"

namespace rtql8 {
    namespace kinematics {

        Skeleton::Skeleton() {
            mMass = 0;
        }

        Skeleton::~Skeleton(){
            for(unsigned int i = 0; i < mJoints.size(); i++) delete mJoints[i];
            mJoints.clear();
            mDofs.clear();
            mTransforms.clear();
            for(unsigned int i=0; i<mNodes.size(); i++) delete mNodes[i];
            mNodes.clear();
            mMarkers.clear();
        }

        BodyNode* Skeleton::createBodyNode(const char* const _name) {
            return new BodyNode(_name);
        }

        void Skeleton::addMarker(Marker *_h) {
            mMarkers.push_back(_h);
            _h->setSkelIndex(mMarkers.size()-1);
            BodyNode *body = _h->getNode();
            body->addMarker(_h);
        }

        void Skeleton::addNode(BodyNode *_b, bool _addParentJoint) {
            mNodes.push_back(_b);
            mNodeNameMap[_b->getName()] = mNodes.size()-1;
            _b->setSkelIndex(mNodes.size()-1);
            if (_addParentJoint)
                addJoint(_b->getParentJoint());
        }

        void Skeleton::addJoint(Joint *_j) {
            mJoints.push_back(_j);
            _j->setSkelIndex(mJoints.size()-1);
        }

        void Skeleton::addDof(Dof *_q) {
            mDofs.push_back(_q);
            _q->setSkelIndex(mDofs.size()-1);
            _q->setVariable();
        }

        void Skeleton::addTransform(Transformation *_t) {
            mTransforms.push_back(_t);
            _t->setVariable(true);
            _t->setSkelIndex(mTransforms.size()-1);
            for(int i=0; i<_t->getNumDofs(); i++) {
                addDof(_t->getDof(i));
            }
        }

        void Skeleton::initSkel() {
            mRoot = mNodes[0];

            // calculate mass
            // init the dependsOnDof stucture for each bodylink
            for(int i=0; i<getNumNodes(); i++) {
                mNodes[i]->setSkel(this);
                // mNodes[i]->setDependDofMap(getNumDofs());
                mNodes[i]->setDependDofList();
                mNodes.at(i)->init();
                mMass += mNodes[i]->getMass();
            }

            mCurrPose = VectorXd::Zero(getNumDofs());

            for(int i=0; i<getNumDofs(); i++)
                mCurrPose[i] = mDofs.at(i)->getValue();
            for(int i=0; i<getNumNodes(); i++) {
                mNodes.at(i)->updateTransform();
            }
        }

        BodyNode* Skeleton::getNode(const char* const name) {
            const std::map<string, int>::iterator itr = 
                mNodeNameMap.find(name);
            if (itr != mNodeNameMap.end())
                return getNode(itr->second);
            else return NULL;
        }

        int Skeleton::getNodeIndex(const char* const name) {
            const std::map<string, int>::iterator itr = 
                mNodeNameMap.find(name);
            
            if (itr != mNodeNameMap.end())
                return itr->second;
            else return -1;
        }

        Vector3d Skeleton::getWorldCOM() {
            assert(mMass != 0);
            Vector3d com(0, 0, 0);
            const int nNodes = getNumNodes();
            for(int i = 0; i < nNodes; i++) {
                BodyNode* node = getNode(i);
                com += (node->getMass() * node->getWorldCOM());
            }
            return com / mMass;
        }

        void Skeleton::setPose(const VectorXd& state, bool bCalcTrans, bool bCalcDeriv) {
            mCurrPose = state;
            for (int i = 0; i < getNumDofs(); i++) {
                mDofs.at(i)->setValue(state[i]);
            }

            if (bCalcTrans) {
                for (int i = 0; i < getNumNodes(); i++) {
                    mNodes.at(i)->updateTransform();
                }
            }

            if (bCalcDeriv) {
                for (int i = 0; i < getNumNodes(); i++) {
                    mNodes.at(i)->updateFirstDerivatives();
                }
            }
        }

        void Skeleton::setPose(const vector<double>& state, bool bCalcTrans, bool bCalcDeriv) {
            VectorXd x(state.size());
            for (unsigned int i = 0; i < state.size(); i++) {
                x(i) = state[i];
            }
            setPose(x, bCalcTrans, bCalcDeriv);
        }

        //void Skeleton::setPose(const VectorXd& _pose){
        //    assert(_pose.size() == getNumDofs());
        //    for(int i=0; i<getNumDofs(); i++)
        //        mDofs[i]->setValue(_pose(i));
        //}

        //void Skeleton::setPose(const vector<double>& _pose){
        //    assert(_pose.size() == getNumDofs());
        //    for(int i=0; i<getNumDofs(); i++)
        //        mDofs[i]->setValue(_pose[i]);
        //}
        
        Eigen::VectorXd Skeleton::getPose() {
            Eigen::VectorXd pose(getNumDofs());
            for (int i = 0; i < getNumDofs(); i++) {
                pose(i) = mDofs[i]->getValue();
            }
            return pose;
        }
        /*
        void Skeleton::getPose(Eigen::VectorXd& _pose) {
            _pose.resize(getNumDofs());
            for (int i = 0; i < getNumDofs(); i++) {
                _pose(i) = mDofs[i]->getValue();
            }
        }
        void Skeleton::getPose(std::vector<double>& _pose) {
            _pose.resize(getNumDofs());
            for (int i = 0; i < getNumDofs(); i++) {
                _pose[i] = mDofs[i]->getValue();
            }
        }
        */
        MatrixXd Skeleton::getJacobian(BodyNode* _bd, Vector3d _localOffset) {
            MatrixXd J(3, mDofs.size());
            J.setZero();
            for(int i = 0; i < _bd->getNumDependentDofs(); i++) {
                int dofindex = _bd->getDependentDof(i);
                VectorXd deriv = utils::xformHom(_bd->getDerivWorldTransform(i), _localOffset);
                J.col(dofindex) = deriv;
            }
            return J;
        }

        void Skeleton::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
            mRoot->draw(_ri, _color, _useDefaultColor);
        }
        void Skeleton::drawMarkers(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
            mRoot->drawMarkers(_ri, _color, _useDefaultColor);
        }


    } // namespace kinematics
} // namespace rtql8
