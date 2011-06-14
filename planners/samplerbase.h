/*
    Copyright (c) 2010-2011, Billy Okal sudo@makokal.com
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the author nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY the author ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL the author BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file samplerbase.h
 * \brief Set of Samplers
 */

#ifndef SAMPLERBASE_H
#define SAMPLERBASE_H

#include "../utils/oputils.h"

namespace openprm
{

/**
 * \class SamplerBase
 */
class SamplerBase
{
public:
    SamplerBase ( RobotBasePtr probot ) : robot ( probot )
    {
        idimension = robot->GetActiveDOF();
    }

    virtual ~SamplerBase() {}

    virtual RobotBasePtr GetRobot () const
    {
        return robot;
    }

    virtual int GetDim () const
    {
        return idimension;
    }


    virtual bool GenSingleSample ( config& sq )
    {
        sq.resize ( idimension );

        return true;
    }

	//TODO - more required
    virtual bool GenMultiSamples ( configSet& sqs )
    {
        return true;
    }

protected:
	
    RobotBasePtr robot;
    int idimension;
};

/**
 * \class RandomSampler
 */
class RandomSampler : public SamplerBase
{
public:
    RandomSampler ( RobotBasePtr probot ) : SamplerBase ( probot )
    {
        robot->GetActiveDOFLimits ( lower, upper );
        range.resize ( lower.size() );
        for ( int i = 0; i < ( int ) range.size(); ++i )
        {
            range[i] = upper[i] - lower[i];
        }
    }

    virtual ~RandomSampler() {}

    bool GenSingleSample ( config& vs )
    {
        vs.resize ( lower.size() );
        for ( size_t i = 0; i < lower.size(); i++ )
        {
            vs[i] = lower[i] + RaveRandomFloat() *range[i];
        }
        return true;
    }

    bool GenMultiSamples ( configSet& vvs )
    {
        FOREACH ( it, vvs )
        {
            GenSingleSample ( *it );
        }
        return true;
    }

    bool GenNeighSample ( config& vns, const config vcs, dReal fRadius )
    {
        RAVELOG_INFO ( "Not implemnted yet\n" );
        return true;
    }

protected:
    std::vector<dReal> range, lower, upper;
};


class HaltonSampler : public SamplerBase
{
public:
    HaltonSampler ( RobotBasePtr probot ) : SamplerBase(probot)
	{
		
	}
	
    virtual ~HaltonSampler();
};


class GaussianSampler : public SamplerBase
{
public:
    GaussianSampler ( RobotBasePtr probot ) : SamplerBase(probot)
	{
		
	}
	
    virtual ~GaussianSampler();
};

}

#endif // SAMPLERBASE_H
