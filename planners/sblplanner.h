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

#ifndef SBLPLANNER_H
#define SBLPLANNER_H

#include "../utils/spatialrep.h"
#include "prmparams.h"
#include "samplerbase.h"

namespace openprm
{

class SBLPlanner : public PlannerBase
{
public:
    SBLPlanner ( EnvironmentBasePtr penv );
    virtual ~SBLPlanner();

    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams);
    bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream);
    virtual PlannerParametersConstPtr GetParameters() const;
    virtual RobotBasePtr GetRobot() const;

protected:
    RobotBasePtr p_robot;
    boost::shared_ptr<PRMParams> p_parameters;
    boost::shared_ptr<SpatialTree<SBLPlanner, tree_node> > t_start;	//! tree from start config
    boost::shared_ptr<SpatialTree<SBLPlanner, tree_node> > t_goal;	//! tree from goal config
    boost::shared_ptr<RandomSampler> p_sampler;
    std::list<spatial_node> l_pathnodes;
    v_config v_random_config;
};


/** ======================================================================================= */

SBLPlanner::SBLPlanner(EnvironmentBasePtr penv): PlannerBase(penv)
{
    v_random_config.clear();
    l_pathnodes.clear();
}

SBLPlanner::~SBLPlanner() {}

PlannerBase::PlannerParametersConstPtr SBLPlanner::GetParameters() const
{
    return p_parameters;
}

RobotBasePtr SBLPlanner::GetRobot() const
{
    return p_robot;
}


/** ======================================================================================= */

bool SBLPlanner::InitPlan(RobotBasePtr pbase, PlannerBase::PlannerParametersConstPtr pparams)
{
    RAVELOG_INFO("SBL::Initializing Planner\n");

    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    p_parameters.reset<PRMParams>(new PRMParams());
    p_parameters->copy(pparams);

    // set up robot and the tsrchains
    p_robot = pbase;

    v_random_config.resize(p_robot->GetActiveDOF());

    p_sampler.reset<RandomSampler>(new RandomSampler(p_robot));
    // 	_sampler = new RandomSampler(_pRobot);

    t_goal.reset<SpatialTree<SBLPlanner, t_node> >(new SpatialTree<SBLPlanner, tree_node>);
    t_start.reset<SpatialTree<SBLPlanner, t_node> >(new SpatialTree<SBLPlanner, tree_node>);

    RAVELOG_INFO("SBLPlanner Initialized\n");
    return true;
}


/** ======================================================================================= */

bool SBLPlanner::PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr< ostream > pOutStream)
{
    if (!p_parameters)
    {
        RAVELOG_ERROR("ClassicPRM::PlanPath - Error, planner not initialized\n");
        return false;
    }

    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    uint32_t basetime = timeGetTime();

    RobotBase::RobotStateSaver savestate(p_robot);
    CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

    //! \todo build tree and get a path





    // create Trajectory from path found
    OpenRAVE::Trajectory::TPOINT pt;
    pt.q.resize(p_parameters->GetDOF());
    
    FOREACH ( itnode, l_pathnodes ) {
        for ( int i = 0; i < p_parameters->GetDOF(); ++i ) {
            pt.q[i] = (*itnode).nconfig[i];
        }
        ptraj->AddPoint(pt);
    }

    RAVELOG_DEBUGA(str(boost::format("plan success, path=%d points in %fs\n")%ptraj->GetPoints().size()%((0.001f*(float)(timeGetTime()-basetime)))));

    return true;
}


}

#endif
