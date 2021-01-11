/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/
#include "PathHolder.h"

CPathHolder::CPathHolder()
{
}

CPathHolder::~CPathHolder()
{
}

void CPathHolder::SetTarget(float inX , float inY)
{
    arPoint.clear();

    // 离散出中间连续点
    stPathPoint pntStart,inPnt;
    pntStart.x = 0;
    pntStart.y = 0;
    inPnt.x = inX;
    inPnt.y = inY;
    float fDist = sqrt((inPnt.x - pntStart.x) * (inPnt.x - pntStart.x) + (inPnt.y - pntStart.y) * (inPnt.y - pntStart.y));
    int nStep = fDist/0.05;
    stPathPoint pntMid;
    float fStep_x = (inPnt.x - pntStart.x)/nStep;
    float fStep_y = (inPnt.y - pntStart.y)/nStep;
    for(int i=1;i<=nStep;i++)
    {
        pntMid.x = pntStart.x + fStep_x*i;
        pntMid.y = pntStart.y + fStep_y*i;
        arPoint.push_back(pntMid);
    }
    arPoint.push_back(inPnt);
}
