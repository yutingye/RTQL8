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

#ifndef _SIMWINDOW_
#define _SIMWINDOW_

#include "yui/Win3D.h"
#include "World.h"

namespace rtql8
{
  namespace simulation
  {
    class World;
  } // namespace simulation
} // namespace rtql8

namespace rtql8
{
    namespace simulation
    {
        class SimWindow : public rtql8::yui::Win3D
            {
            public:
            SimWindow(): Win3D()
                    {
                        mBackground[0] = 1.0;
                        mBackground[1] = 1.0;
                        mBackground[2] = 1.0;
                        mBackground[3] = 1.0;
		
                        mPlay = false;
                        mPlayFrame = 0;
                        mShowMarkers = true;
                        mPersp = 45.f;
                        mTrans[1] = 300.f;
                    }
                virtual void timeStepping() 
                {
                    mWorld->updatePhysics();
                }
                virtual void drawSkels();
                virtual void displayTimer(int _val);
                virtual void draw();
                virtual void keyboard(unsigned char key, int x, int y);
                void setWorld(rtql8::simulation::World *_world)
                {
                    mWorld = _world;
                }

            protected:    
                rtql8::simulation::World* mWorld;
                int mPlayFrame;
                bool mPlay;
                bool mShowMarkers;
                std::vector<Eigen::VectorXd> mBakedStates;

                void bake();
            };
    } // namespace simulation
} // namespace rtql8
#endif // #ifndef SIMULATION_SIMWINDOW
