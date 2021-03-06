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

#include "FileInfoC3D.h"
#include "C3D.h"

#include <cassert>
using namespace std;

namespace rtql8 {
    namespace kinematics {

        FileInfoC3D::FileInfoC3D()
            : mNumMarkers(0), mNumFrames(0), mFPS(0){
        }

        bool FileInfoC3D::loadFile(const char* _fName)
        {
            if( loadC3DFile( _fName, mData, &mNumFrames, &mNumMarkers, &mFPS) ){
                string text = _fName;
                int lastSlash = text.find_last_of("/");
                text = text.substr(lastSlash+1);
                strcpy(mFileName, text.c_str());
                return true;
            }else{
                return false;
            }
        }

        bool FileInfoC3D::saveFile(const char* _fName, int _start, int _end, double _sampleRate)
        {
            EIGEN_VV_VEC3D tmpData = mData;

            int first = _start<mNumFrames?_start:mNumFrames-1;
            int last = _end<mNumFrames?_end:mNumFrames-1;

            tmpData.erase( tmpData.begin()+last+1, tmpData.end()+1);
            tmpData.erase( tmpData.begin(), tmpData.begin()+first);

            if( saveC3DFile( _fName, tmpData, last-first+1, mData[0].size(), mFPS )){
                string text = _fName;
                int lastSlash = text.find_last_of("/");
                text = text.substr(lastSlash+1);
                strcpy(mFileName, text.c_str());
                return true;
            }else{
                return false;
            }
        }
    }
} // namespace rtql8
