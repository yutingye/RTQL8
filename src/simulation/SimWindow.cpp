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

#include <cstdio>

#include "SimWindow.h"
#include "simulation/World.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "collision/CollisionSkeleton.h"
#include "yui/GLFuncs.h"

using namespace rtql8::utils;
using namespace Eigen;
using namespace std;

namespace rtql8
{
    namespace simulation
    {
        void SimWindow::drawSkels() 
        {
            for (int i = 0; i < mWorld->getNumSkels(); i++)
                mWorld->getSkel(i)->draw(mRI);
        }

        void SimWindow::displayTimer(int _val)
        {
            int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
            if (mPlay) {
                mPlayFrame += 16;
                if (mPlayFrame >= mBakedStates.size())
                    mPlayFrame = 0;
            }else if (mWorld->isSimulating()) {
                for (int i = 0; i < numIter; i++) {
                    timeStepping();
                    bake();
                }
            }
            glutPostRedisplay();
            glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
        }

        void SimWindow::draw()
        {
            glDisable(GL_LIGHTING);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            if (!mWorld->isSimulating()) {
                if (mPlayFrame < mBakedStates.size()) {
                    int nSkels = mWorld->getNumSkels();
                    for (unsigned int i = 0; i < nSkels; i++) {
                        int start = mWorld->getIndex(i);
                        int size = mWorld->getDofs(i).size();
                        mWorld->getSkel(i)->setPose(mBakedStates[mPlayFrame].segment(start, size), false, false);
                    }
                    if (mShowMarkers) {
                        int sumDofs = mWorld->getIndex(nSkels);
                        int nContact = (mBakedStates[mPlayFrame].size() - sumDofs) / 6;
                        for (int i = 0; i < nContact; i++) {
                            Vector3d v = mBakedStates[mPlayFrame].segment(sumDofs + i * 6, 3);
                            Vector3d f = mBakedStates[mPlayFrame].segment(sumDofs + i * 6 + 3, 3) / 10.0;
                            glBegin(GL_LINES);
                            glVertex3f(v[0], v[1], v[2]);
                            glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
                            glEnd();
                            mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
                            mRI->pushMatrix();
                            glTranslated(v[0], v[1], v[2]);
                            mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
                            mRI->popMatrix();
                        }
                    }
                }
            }else{
                if (mShowMarkers) {
                    for (int k = 0; k < mWorld->getCollisionHandle()->getCollisionChecker()->getNumContact(); k++) {
                        Vector3d  v = mWorld->getCollisionHandle()->getCollisionChecker()->getContact(k).point;
                        Vector3d f = mWorld->getCollisionHandle()->getCollisionChecker()->getContact(k).force / 10.0;
                        glBegin(GL_LINES);
                        glVertex3f(v[0], v[1], v[2]);
                        glVertex3f(v[0] + f[0], v[1] + f[1], v[2] + f[2]);
                        glEnd();
                        mRI->setPenColor(Vector3d(0.2, 0.2, 0.8));
                        mRI->pushMatrix();
                        glTranslated(v[0], v[1], v[2]);
                        mRI->drawEllipsoid(Vector3d(0.02, 0.02, 0.02));
                        mRI->popMatrix();
                    }
                }
            }
            drawSkels();
            
            // display the frame count in 2D text
            char buff[64];
            if (!mWorld->isSimulating()) 
                sprintf(buff, "%d", mPlayFrame);
            else
                sprintf(buff, "%d", mWorld->getSimFrames());
            string frame(buff);
            glColor3f(0.0, 0.0, 0.0);
            rtql8::yui::drawStringOnScreen(0.02f, 0.02f, frame);
            glEnable(GL_LIGHTING);
        }

        void SimWindow::keyboard(unsigned char key, int x, int y)
        {
            switch(key){
            case ' ': // use space key to play or stop the motion
                mWorld->setSimulatingFlag(!mWorld->isSimulating());
                if(mWorld->isSimulating()) {
                    mPlay = false;
                    glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
                }
                break;
            case 'p': // playBack
                mPlay = !mPlay;
                if (mPlay) {
                    mWorld->setSimulatingFlag(false);
                    glutTimerFunc( mDisplayTimeout, refreshTimer, 0);
                }
                break;
            case '[': // step backward
                if (!mWorld->isSimulating()) {
                    mPlayFrame--;
                    if(mPlayFrame < 0)
                        mPlayFrame = 0;
                    glutPostRedisplay();
                }
                break;
            case ']': // step forwardward
                if (!mWorld->isSimulating()) {
                    mPlayFrame++;
                    if(mPlayFrame >= mBakedStates.size())
                        mPlayFrame = 0;
                    glutPostRedisplay();
                }
                break;
            case 'v': // show or hide markers
                mShowMarkers = !mShowMarkers;
                break;
            default:
                Win3D::keyboard(key,x,y);
            }
            glutPostRedisplay();
        }

        void SimWindow::bake()
        {
            int nContact = mWorld->getCollisionHandle()->getCollisionChecker()->getNumContact();
            VectorXd state(mWorld->getIndex(mWorld->getNumSkels()) + 6 * nContact);
            for (unsigned int i = 0; i < mWorld->getNumSkels(); i++)
                state.segment(mWorld->getIndex(i), mWorld->getDofs(i).size()) = mWorld->getDofs(i);
            for (int i = 0; i < nContact; i++) {
                int begin = mWorld->getIndex(mWorld->getNumSkels()) + i * 6;
                state.segment(begin, 3) = mWorld->getCollisionHandle()->getCollisionChecker()->getContact(i).point;
                state.segment(begin + 3, 3) = mWorld->getCollisionHandle()->getCollisionChecker()->getContact(i).force;
            }
            mBakedStates.push_back(state);
        }

    } // namespace simulation
} // namespace rtql8
