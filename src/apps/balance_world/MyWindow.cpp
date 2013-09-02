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

#include "MyWindow.h"
#include "simulation/World.h"
#include "dynamics/BodyNodeDynamics.h"
#include "yui/GLFuncs.h"
#include "utils/UtilsMath.h"

using namespace Eigen;
using namespace std;
using namespace rtql8::dynamics;
using namespace rtql8::yui;
using namespace rtql8::utils;


void MyWindow::timeStepping()
{
    static_cast<BodyNodeDynamics*>(mWorld->getSkel(1)->getNode("fullbody1_h_spine"))->addExtForce(Vector3d(0.0, 0.0, 0.0), mForce);
    mController->computeTorques(mWorld->getDofs(1), mWorld->getDofVels(1));
    mWorld->getSkel(1)->setInternalForces(mController->getTorques());

    // for perturbation test
    mImpulseDuration--;
    if (mImpulseDuration <= 0) {
        mImpulseDuration = 0;
        mForce.setZero();
    }    
}

void MyWindow::drawSkels()
{
    for (unsigned int i = 0; i < mWorld->getNumSkels(); i++)
        mWorld->getSkel(i)->draw(mRI);

    // draw arrow
    if (mImpulseDuration > 0) {
        Vector3d poa = xformHom(mWorld->getSkel(1)->getNode("fullbody1_h_spine")->getWorldTransform(), Vector3d(0.0, 0.0, 0.0));
        Vector3d start = poa - mForce / 10.0;
        double len = mForce.norm() / 10.0;
        drawArrow3D(start, mForce, len, 0.05, 0.1);
    }
}

void MyWindow::keyboard(unsigned char key, int x, int y)
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
    case '1':
        mForce[0] = 20;
        mImpulseDuration = 10.0;
        cout << "push forward" << endl;
        break;
    case '2':
        mForce[0] = -10;
        mImpulseDuration = 10.0;
        cout << "push backward" << endl;
        break;
    case '3':
        mForce[2] = 50;
        mImpulseDuration = 10.0;
        cout << "push right" << endl;
        break;
    case '4':
        mForce[2] = -50;
        mImpulseDuration = 10.0;
        cout << "push left" << endl;
        break;
    default:
        Win3D::keyboard(key,x,y);

    }
    glutPostRedisplay();
}
