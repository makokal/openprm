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

#ifndef VISPRM_H
#define VISPRM_H

#include "../utils/spatialrep.h"
#include "prmparams.h"
#include "samplerbase.h"

//TODO - change to visibility graph

namespace openprm
{

class VISPRM : public PlannerBase
{
public:
    VISPRM ( EnvironmentBasePtr penv );
    virtual ~VISPRM();

    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams);
    bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream);
    virtual PlannerParametersConstPtr GetParameters() const;
    virtual RobotBasePtr GetRobot() const;

protected:
    RobotBasePtr p_robot;
    boost::shared_ptr<PRMParams> p_parameters;
    boost::shared_ptr<SpatialGraph> g_roadmap;
    boost::shared_ptr<RandomSampler> p_sampler;
    std::list<spatial_node> l_pathnodes;
    v_config v_random_config;
    vv_config_set vv_samples;
    spatial_node n_start, n_goal;

    int buildRoadMap();
    bool findPath();
    bool addStartConfig();
    bool addGoalConfig();


    inline virtual boost::shared_ptr<VISPRM> shared_planner()
    {
        return boost::static_pointer_cast<VISPRM>(shared_from_this());
    }

    inline virtual boost::shared_ptr<VISPRM const> shared_planner_const() const
    {
        return boost::static_pointer_cast<VISPRM const>(shared_from_this());
    }
};


///Implems
VISPRM::VISPRM ( EnvironmentBasePtr penv ) : PlannerBase ( penv )
{
    v_random_config.clear();
    vv_samples.clear();
    l_pathnodes.clear();
}

VISPRM::~VISPRM() {}

PlannerBase::PlannerParametersConstPtr VISPRM::GetParameters() const
{
    return p_parameters;
}

RobotBasePtr VISPRM::GetRobot() const
{
    return p_robot;
}

bool VISPRM::InitPlan ( RobotBasePtr pbase, PlannerParametersConstPtr pparams )
{
    RAVELOG_INFO("Initializing Planner\n");

    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    p_parameters.reset<PRMParams>(new PRMParams());
    p_parameters->copy(pparams);

    // set up robot and the tsrchains
    p_robot = pbase;

    v_random_config.resize(p_robot->GetActiveDOF());

    p_sampler.reset<RandomSampler>(new RandomSampler(p_robot));

    //TODO - add use of medges
    g_roadmap.reset<SpatialGraph>(new SpatialGraph);

    RAVELOG_INFO("VISPRM::building roadmap\n");
    int nodes = buildRoadMap();

    RAVELOG_INFO("VISPRM Initialized with Roadmap of [%d] Nodes\n", nodes);
    return true;
}

bool VISPRM::PlanPath ( TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream )
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

    if ( !addStartConfig() )
    {
        RAVELOG_ERROR("Start configuration not added to roadmap, planning abort\n");
        return false;
    }

    if ( !addGoalConfig() )
    {
        RAVELOG_ERROR("Goal configuration not added to roadmap, planning abort\n");
        return false;
    }

    if ( !findPath() )
    {
        RAVELOG_ERROR("No path found\n");
        return false;
    }

    /// create Trajectory from path found
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

/**
 * \def _buildRoadMap
 * Main distinguishing feature
 * Based on Visibility Graph
 */
int VISPRM::buildRoadMap()
{
    // Generate samples from the CSpace
    int i = 0;
    while (i < p_parameters->iMnodes)
    {
        if (p_sampler->GenSingleSample(v_random_config) )
        {
            vv_samples.push_back(v_random_config);
        }
        else
        {
            RAVELOG_INFO("Failed to get a sample\n");
            continue;
        }
        i++;
    }

    RAVELOG_DEBUG("connecting samples\n");
    for (vv_config_set::iterator it = vv_samples.begin(); it != vv_samples.end(); it++)
    {
        std::list<spatial_node> neighbors;
        spatial_vertex vs = g_roadmap->addNode(*it);

        RAVELOG_DEBUGA("added node\n");
        int nns = g_roadmap->findNN(vs, neighbors);
        if (nns == 0)
        {
            continue;
        }

        for (std::list<spatial_node>::iterator itt = neighbors.begin(); itt != neighbors.end(); itt++)
        {
            if (!ICollision::CheckCollision(p_parameters, p_robot, (*it), (*itt).nconfig, OPEN) )
            {
                if (!g_roadmap->addEdge(vs, (*itt).vertex ))
                {
                    RAVELOG_WARN("Failure in adding an edge\n");
                }
            }
            else
            {
                continue;
            }
        }
    }

    // print the roadmap topographical sketch
    g_roadmap->printGraph("visprm_roadmap.dot");

    return g_roadmap->getNodes();
}

bool VISPRM::findPath(spatial_node _start, spatial_node _goal)
{	
    if ( g_roadmap->findPathAS(_start, _goal, l_pathnodes) )
    {
        RAVELOG_VERBOSE("Found Goal with A* \n");
        return true;
    }
    else if ( g_roadmap->findPathDK(_start, _goal, l_pathnodes) )
    {
        RAVELOG_VERBOSE("Found Goal with Dijkstra \n");
        return true;
    }

    return false;
}

bool VISPRM::addStartConfig()
{
    if ((int)p_parameters->vinitialconfig.size() != p_robot->GetActiveDOF())
    {
        RAVELOG_ERROR("Specified Start Configuration is valid\n");
        return false;
    }


    spatial_vertex st = g_roadmap->addNode(p_parameters->vinitialconfig);
    std::list<spatial_node> nearsamples;

    RAVELOG_INFO("Getting near samples for start [%d]\n", st);

    int neis = g_roadmap->findNN(st, nearsamples);
    RAVELOG_INFO("neighbors [%d]\n", neis);

    if ( neis == 0)
    {
        RAVELOG_WARN("Warning! Start node too far from the roadmap\n");
        return false;
    }

    RAVELOG_INFO("Adding near samples\n");

    for (std::list<spatial_node>::iterator it = nearsamples.begin(); it != nearsamples.end(); it++)
    {
        spatial_node nn = g_roadmap->getNode(st);
        if (!ICollision::CheckCollision(p_parameters, p_robot, (*it).nconfig, nn.nconfig, OPEN) )
        {
            if (g_roadmap->addEdge((*it).vertex, st))
            {
                RAVELOG_INFO("Added the start configuration\n");
                n_start = nn;
                break;
            }
        }
        else
        {
            continue;
        }
    }

    if ( n_start.vertex > (2*g_roadmap->getNodes()) )
    {
        RAVELOG_ERROR("Added invalid start configuration\n");
        return false;
    }

    return true;
}

bool VISPRM::addGoalConfig()
{
    if ((int)p_parameters->vgoalconfig.size() != p_robot->GetActiveDOF())
    {
        RAVELOG_ERROR("Specified Goal Configuration is valid\n");
        return false;
    }


    spatial_vertex st = g_roadmap->addNode(p_parameters->vgoalconfig);
    std::list<spatial_node> nearsamples;

    RAVELOG_INFO("Getting near samples for goal [%d]\n", st);

    int neis = g_roadmap->findNN(st, nearsamples);
    RAVELOG_INFO("neighbors [%d]\n", neis);

    if ( neis == 0)
    {
        RAVELOG_WARN("Warning! Goal node too far from the roadmap\n");
        return false;
    }

    RAVELOG_INFO("Adding near samples\n");

    for (std::list<spatial_node>::iterator it = nearsamples.begin(); it != nearsamples.end(); it++)
    {
        spatial_node nn = g_roadmap->getNode(st);
        if (!ICollision::CheckCollision(p_parameters, p_robot, (*it).nconfig, nn.nconfig, OPEN) )
        {
            if (g_roadmap->addEdge((*it).vertex, st))
            {
                RAVELOG_INFO("Added the Goal configuration\n");
                n_goal = nn;
                break;
            }
        }
        else
        {
            continue;
        }
    }

    if ( n_goal.vertex > (2*g_roadmap->getNodes()) )
    {
        RAVELOG_ERROR("Added invalid goal configuration\n");
        return false;
    }

    return true;
}

}

#endif
