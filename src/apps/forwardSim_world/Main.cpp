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

#include "dynamics/SkeletonDynamics.h"
#include "kinematics/FileInfoSkel.hpp"
#include "utils/Paths.h"
#include "utils/UtilsMath.h"
#include "simulation/World.h"

#include "MyWindow.h"

using namespace rtql8::kinematics;
using namespace rtql8::dynamics;
using namespace rtql8::simulation;
using namespace rtql8::utils;

int main(int argc, char* argv[])
{
    // load a skeleton file
    FileInfoSkel<SkeletonDynamics> model;
    model.loadFile(RTQL8_DATA_PATH"/skel/Chain.skel", SKEL);
    
    // create and initialize the world
    World *myWorld = new World();  
    myWorld->addSkeleton((SkeletonDynamics*)model.getSkel());
    int nDof =  myWorld->getSkel(0)->getNumDofs();
    VectorXd initPose(nDof);
    for (int i = 0; i < nDof; i++)
        initPose[i] = random(-0.5, 0.5);
    myWorld->getSkel(0)->setPose(initPose);
    myWorld->setTimeStep(1.0/2000);
    myWorld->init();

    // create a window and link it to the world
    MyWindow window;
    window.setWorld(myWorld);
  
    glutInit(&argc, argv);
    window.initWindow(640, 480, "Forward Simulation");
    glutMainLoop();

    return 0;
}