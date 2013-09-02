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

#ifndef KINEMATICS_PARSER_SKEL_H
#define KINEMATICS_PARSER_SKEL_H

namespace rtql8 {
    namespace kinematics {
        class Transformation;
        class Shape;
        class Dof;
    }
}

typedef rtql8::kinematics::Dof* dofVec3[3];
typedef rtql8::kinematics::Dof* dofVec4[4];
typedef double doubleVec3[3];

typedef union {
    double	dValue;
    int iValue;
    char* sValue;
    dofVec3 v3DValue;
    dofVec4 v4DValue;
    doubleVec3 v3VValue;
    rtql8::kinematics::Transformation* tValue;
    rtql8::kinematics::Shape* pValue;
    rtql8::kinematics::Dof* dofValue;
} yystype;

# define YYSTYPE yystype
# define YYSTYPE_IS_TRIVIAL 1
# define YYSTYPE_IS_DECLARED

#define YY_FLOAT 258
#define YY_INTEGER 259
#define YY_STRING 260
#define YY_PRIMITIVE 261
#define YY_CHAIN 262
#define YY_TRANSLATE 263
#define YY_TELESCOPE 264
#define YY_SCALE 265
#define YY_ROTATE_QUAT 266
#define YY_ROTATE_EXPMAP 267
#define YY_ROTATE_EULER 268
#define YY_ROTATE_CONS 269
#define YY_MARKER 270
#define YY_NODE 271
#define YY_CONST 272
#define YY_DOFS 273
#define YY_MARKERS 274
#define YY_MASS 275

extern YYSTYPE yylval;

#endif // #ifndef KINEMATICS_PARSER_SKEL_H
