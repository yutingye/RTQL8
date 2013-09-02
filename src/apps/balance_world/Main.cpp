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
 *		(Copyright (c) 2001-2004, Russell L. Smith. All rights 
 *		reserved.) and portions of FCL (Copyright (c) 2011, Willow 
 *		Garage, Inc. All rights reserved.), which were released under
 *		the same BSD license as below
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

#include "simulation/World.h"
#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"

#include "MyWindow.h"

using namespace Eigen;
using namespace std;
using namespace rtql8::kinematics;
using namespace rtql8::dynamics;
using namespace rtql8::simulation;

int main(int argc, char* argv[])
{
    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model, model2;
    model.loadFile(RTQL8_DATA_PATH"/skel/ground1.skel", SKEL);
    model2.loadFile(RTQL8_DATA_PATH"/skel/fullbody1.skel", SKEL);

    // create and initialize the world
    World *myWorld = new World();
    myWorld->addSkeleton((SkeletonDynamics*)model.getSkel());
    myWorld->addSkeleton((SkeletonDynamics*)model2.getSkel());

    VectorXd initPose = myWorld->getSkel(0)->getPose();
    initPose[1] = -0.92;
    myWorld->getSkel(0)->setPose(initPose);
    myWorld->getSkel(0)->setImmobileState(true);

    initPose = myWorld->getSkel(1)->getPose();
    initPose[1] = -0.1;
    initPose[6] = 0.2; // left hip
    initPose[9] = -0.5; // left knee
    initPose[10] = 0.3; // left ankle
    initPose[13] = 0.2; // right hip
    initPose[16] = -0.5; // right knee
    initPose[17] = 0.3; // right ankle
    initPose[21] = -0.1; // lower back
    myWorld->getSkel(1)->setPose(initPose);
    
    myWorld->init();

    // create controller
    Controller *myController = new Controller(myWorld->getSkel(1), myWorld->getCollisionHandle(), myWorld->getTimeStep());

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
    window.setController(myController);


    cout << "space bar: simulation on/off" << endl;
    cout << "'p': playback/stop" << endl;
    cout << "'[' and ']': play one frame backward and forward" << endl;
    cout << "'v': visualization on/off" << endl;
    cout << "'1'--'4': programmed interaction" << endl;
       
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Balance");
    glutMainLoop();

    return 0;
}
