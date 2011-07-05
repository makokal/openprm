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
    v_config v_random_config;
    bool b_connected;
    int i_start_index, i_goal_index;


    inline void buildTrees(int start_id, int goal_id)
    {
        // verify just in case
        b_connected = false;
        //! \todo add a stopping heuristic
        i_start_index = 0;
        i_goal_index = 0;

        RAVELOG_DEBUGA("buiding start and goal trees..\n");

        while(!b_connected)
        {
            RAVELOG_DEBUGA("iteration...\n");
            if (!(p_sampler->GenSingleSample(v_random_config)) )
            {
                RAVELOG_DEBUGA("Error in sampling");
                continue;
            }
//            if (!p_parameters->_samplefn(v_random_config))
//            {
//                RAVELOG_DEBUGA("Error in sampling");
//                continue;
//            }
            else
            {
                /* DEBUG */ RAVELOG_DEBUGA("Sampled, extending trees...\n");

                ExtendType ets, etg;

                t_start->Extend (t_start->GetConfig (start_id), i_start_index);
                ets = t_start->Extend (v_random_config, i_start_index);

                if (ets == ET_Failed)
                {
                    RAVELOG_DEBUGA("Failed to extend tree\n");
                    continue;
                }

//                else
//                {
                    t_goal->Extend (t_goal->GetConfig (goal_id), i_goal_index);
                    etg = t_goal->Extend (t_start->GetConfig (i_start_index), i_goal_index);

                    // check if the trees are conected already
                    if (etg == ET_Connected)
                    {
                        b_connected = true;
                        RAVELOG_INFOA("Trees connected");
                        break;
                    }
//                }
            }

            RAVELOG_DEBUGA("swapping trees\n");

            // swap the trees
            swap(t_start, t_goal);
        }
    }
};


/** ======================================================================================= */

SBLPlanner::SBLPlanner(EnvironmentBasePtr penv): PlannerBase(penv)
{
    v_random_config.clear();
    b_connected = false;
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

    t_goal.reset<SpatialTree<SBLPlanner, tree_node> >(new SpatialTree<SBLPlanner, tree_node>);
    t_start.reset<SpatialTree<SBLPlanner, tree_node> >(new SpatialTree<SBLPlanner, tree_node>);

    v_random_config.resize(p_parameters->GetDOF());
    t_goal->_distmetricfn = p_parameters->_distmetricfn;
    t_goal->_fStepLength = p_parameters->_fStepLength;
    t_start->_distmetricfn = p_parameters->_distmetricfn;
    t_start->_fStepLength = p_parameters->_fStepLength;

    RAVELOG_INFO("SBLPlanner Initialized\n");
    return true;
}


/** ======================================================================================= */

bool SBLPlanner::PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr< ostream > pOutStream)
{
    if (!p_parameters)
    {
        RAVELOG_DEBUGA("Error, planner not initialized\n");
        return false;
    }

    RAVELOG_DEBUGA("locking env\n");
    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    uint32_t basetime = timeGetTime();

    RobotBase::RobotStateSaver savestate(p_robot);
    CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

    RAVELOG_DEBUGA("initializing start and goal trees\n");
    //! build tree and get a path
    int s_id = t_start->AddNode (0, p_parameters->vinitialconfig);
    int g_id = t_goal->AddNode (-1000, p_parameters->vgoalconfig);
    RAVELOG_DEBUGA("initialized start and goal trees\n");

    // build the trees
    buildTrees (s_id, g_id);
//    buildTrees (0,0);

    if ( !b_connected )
    {
        RAVELOG_DEBUGA("plan failed(cound not connect trees), %fs\n",0.001f*(float)(timeGetTime()-basetime));
        return false;
    }

    // ---------------------------------------------
    std::list<tree_node*> l_pathnodes;

    RAVELOG_DEBUGA("concatenating path\n");

    // add nodes in the path, from start to connect point
    tree_node* st_node = t_start->_nodes.at (i_start_index);
    while (1)
    {
        l_pathnodes.push_front (st_node);
        if (st_node->parent < 0)
        {
            break;
        }
        else
        {
            st_node = t_start->_nodes.at (st_node->parent);
        }
    }

    // add nodes in the path, from goal to connect point
    tree_node* gt_node = t_goal->_nodes.at (i_goal_index);
    while (1)
    {
        l_pathnodes.push_back (gt_node);
        if (gt_node->parent < 0)
        {
            break;
        }
        else
        {
            gt_node = t_goal->_nodes.at (gt_node->parent);
        }
    }

    RAVELOG_DEBUGA("Path has [%d] nodes\n", l_pathnodes.size());

    //--------------------------------------
    //! create Trajectory from path found
    OpenRAVE::Trajectory::TPOINT pt;
    pt.q.resize(p_parameters->GetDOF());
    
    FOREACH ( itnode, l_pathnodes ) {
        for ( int i = 0; i < p_parameters->GetDOF(); ++i ) {
            pt.q[i] = (*itnode)->q[i]; /*(*itnode).nconfig[i];*/
        }
        ptraj->AddPoint(pt);
    }

    RAVELOG_DEBUGA(str(boost::format("plan success, path=%d points in %fs\n")%ptraj->GetPoints().size()%((0.001f*(float)(timeGetTime()-basetime)))));

    return true;
}


}

#endif
