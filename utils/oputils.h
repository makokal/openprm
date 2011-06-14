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
 * \file oputils.h
 * \author Billy Okal
 * Contains useful utilities for openprm planners such as;
 * 	- Collision Checkers
 * 	- Distance Metrics
 */

#ifndef OPUTILS_H
#define OPUTILS_H

#include "../plugindefs.h"

namespace openprm
{

typedef std::vector<dReal> v_config;
typedef std::vector<std::vector<dReal> > vv_config_set;

enum IntervalType
{
    OPEN=0,
    OPEN_START,
    OPEN_END,
    CLOSED
};


/** ======================================================================================= */

/**
 * \class DistMetric
 * \brief RoadMaps Distance Metric in arbitrary dimensions,
 * It reduces to basic Euclidean distance
 */
class DistMetric
{
public:
    static dReal Eval ( v_config q1, v_config q2 )
    {
        if ( q1.size() != q2.size() )
        {
            RAVELOG_ERROR ( "Invalid Configuration dimensions [%d] and [%d], Hint: must be equal\n",q1.size(), q2.size() );
        }

        dReal d = 0;
        int i = 0;

        while ( i < ( int ) q1.size() )
        {
            dReal t = abs ( q1[i] - q2[i] );
            d += t * t;
            i++;
        }

        return sqrt ( d );
    }
};


/** ======================================================================================= */

/**
 * \class ICollision
 * \brief Collision cheking in incremental fashion along a line
 */
class ICollision
{
public:
    static bool CheckCollision ( PlannerBase::PlannerParametersConstPtr params, RobotBasePtr robot, const vector<dReal>& pQ0, const vector<dReal>& pQ1, IntervalType interval, vector< vector<dReal> >* pvCheckedConfigurations = NULL )
    {
        // set the bounds based on the interval type
        int start=0;
        bool bCheckEnd=false;
        switch ( interval )
        {
        case IT_Open:
            start = 1;
            bCheckEnd = false;
            break;
        case IT_OpenStart:
            start = 1;
            bCheckEnd = true;
            break;
        case IT_OpenEnd:
            start = 0;
            bCheckEnd = false;
            break;
        case IT_Closed:
            start = 0;
            bCheckEnd = true;
            break;
        default:
            BOOST_ASSERT ( 0 );
        }

        // first make sure the end is free
        vector<dReal> vlastconfig ( params->GetDOF() ), vtempconfig ( params->GetDOF() );
        if ( bCheckEnd )
        {
            params->_setstatefn ( pQ1 );
            if ( robot->GetEnv()->CheckCollision ( KinBodyConstPtr ( robot ) ) || ( robot->CheckSelfCollision() ) )
            {
                if ( pvCheckedConfigurations != NULL )
                {
                    pvCheckedConfigurations->push_back ( pQ1 );
                }
                return true;
            }
        }

        // compute  the discretization
        vector<dReal> dQ = pQ1;
        params->_diffstatefn ( dQ,pQ0 );
        int i, numSteps = 1;
        vector<dReal>::const_iterator itres = params->_vConfigResolution.begin();
        for ( i = 0; i < params->GetDOF(); i++,itres++ )
        {
            int steps;
            if ( *itres != 0 )
                steps = ( int ) ( fabs ( dQ[i] ) / *itres );
            else
                steps = ( int ) ( fabs ( dQ[i] ) * 100 );
            if ( steps > numSteps )
                numSteps = steps;
        }
        dReal fisteps = dReal ( 1.0f ) /numSteps;
        FOREACH ( it,dQ )
        *it *= fisteps;

        if ( !!params->_constraintfn )
            vlastconfig = pQ0;
        // check for collision along the straight-line path
        // NOTE: this does not check the end config, and may or may
        // not check the start based on the value of 'start'
        for ( int f = start; f < numSteps; f++ )
        {
            for ( i = 0; i < params->GetDOF(); i++ )
            {
                vtempconfig[i] = pQ0[i] + ( dQ[i] * f );
            }
            params->_setstatefn ( vtempconfig );
            if ( !!params->_constraintfn )
            {
                if ( !params->_constraintfn ( vlastconfig,vtempconfig,0 ) )
                {
                    return true;
                }
                vlastconfig = pQ0;
            }
            if ( pvCheckedConfigurations != NULL )
            {
                params->_getstatefn ( vtempconfig ); // query again in order to get normalizations/joint limits
                pvCheckedConfigurations->push_back ( vtempconfig );
            }
            if ( robot->GetEnv()->CheckCollision ( KinBodyConstPtr ( robot ) ) || ( robot->CheckSelfCollision() ) )
            {
                return true;
            }
        }

        if ( bCheckEnd && pvCheckedConfigurations != NULL )
        {
            pvCheckedConfigurations->push_back ( pQ1 );
        }
        return false;
    }

    /// check collision between body and environment
    static bool CheckCollision ( PlannerBase::PlannerParametersConstPtr params, RobotBasePtr robot, const vector<dReal>& pConfig, CollisionReportPtr report=CollisionReportPtr() )
    {
        params->_setstatefn ( pConfig );
        bool bCol = robot->GetEnv()->CheckCollision ( KinBodyConstPtr ( robot ), report ) || ( robot->CheckSelfCollision ( report ) );
        if ( bCol && !!report )
        {
            RAVELOG_WARN ( str ( boost::format ( "fcollision %s\n" ) %report->__str__() ) );
        }
        return bCol;
    }

};


/** ======================================================================================= */

/**
 * \class BCollision
 * \brief Checks for collisions starting at the center and uses divide and conquer approach
 * \TODO - Implement the methods
 */
class BCollision
{
public:
    static bool CheckCollision ( PlannerBase::PlannerParametersConstPtr params, RobotBasePtr robot, const vector<dReal>& pQ0, const vector<dReal>& pQ1, IntervalType interval, vector< vector<dReal> >* pvCheckedConfigurations = NULL )
    {
        RAVELOG_INFO ( "Not implemented yet\n" );
        return false;
    }

    static bool CheckCollision ( PlannerBase::PlannerParametersConstPtr params, RobotBasePtr robot, const vector<dReal>& pConfig, CollisionReportPtr report=CollisionReportPtr() )
    {
        RAVELOG_INFO ( "Not implemented yet\n" );
        return false;
    }
};

}

#endif