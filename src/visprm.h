/**
 * Copyright (c) 2010-2011, Billy Okal
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 * must display the following acknowledgement:
 * This product includes software developed by the author.
 * 4. Neither the name of the author nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VISPRM_H
#define VISPRM_H

#include "spatialrep.h"
#include "prmparams.h"
#include "samplerbase.h"

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
	RobotBasePtr _pRobot;
	boost::shared_ptr<PRMParams> _pParameters;
	boost::shared_ptr<SpatialGraph> _gRoadmap;
	boost::shared_ptr<RandomSampler> _sampler;
// 	RandomSampler* _sampler;
	std::list<s_node> _lPathNodes;
	config _vRandomConfig;
	configSet _vvSamples;
	s_node _nStart, _nGoal;
	
	int _buildRoadMap();
	bool _findPath();
	bool _addStart();
	bool _addGoal();
	
	
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
	__description = " Visibility PRM Planner ";
    _vRandomConfig.clear();
    _vvSamples.clear();
    _lPathNodes.clear();
}

VISPRM::~VISPRM() {}

PlannerBase::PlannerParametersConstPtr VISPRM::GetParameters() const
{
    return _pParameters;
}

RobotBasePtr VISPRM::GetRobot() const
{
    return _pRobot;
}

bool VISPRM::InitPlan ( RobotBasePtr pbase, PlannerParametersConstPtr pparams )
{
	RAVELOG_INFO("Initializing Planner\n");

	EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
	_pParameters.reset<PRMParams>(new PRMParams());
	_pParameters->copy(pparams);
	
    // set up robot and the tsrchains
    _pRobot = pbase;
	
	_vRandomConfig.resize(_pRobot->GetActiveDOF());
	
	_sampler.reset<RandomSampler>(new RandomSampler(_pRobot));
// 	_sampler = new RandomSampler(_pRobot);
	
	//TODO - add use of medges
	_gRoadmap.reset<SpatialGraph>(new SpatialGraph);
// 	SpatialGraph g(_pParameters->iMnodes, _pParameters->fNeighthresh);
// 	_gRoadmap = &g;
	
	RAVELOG_INFO("VISPRM::building roadmap\n");
	int nodes = _buildRoadMap();

	RAVELOG_INFO("VISPRM Initialized with Roadmap of [%d] Nodes\n", nodes);
	return true;
}

bool VISPRM::PlanPath ( TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream )
{
	if(!_pParameters) {
		RAVELOG_ERROR("ClassicPRM::PlanPath - Error, planner not initialized\n");
		return false;
	}

	EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
	uint32_t basetime = timeGetTime();
	
	RobotBase::RobotStateSaver savestate(_pRobot);
	CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);
	
	if ( !_addStart() )
	{
		RAVELOG_ERROR("Start configuration not added to roadmap, planning abort\n");
		return false;
	}
	
	if ( !_addGoal() )
	{
		RAVELOG_ERROR("Goal configuration not added to roadmap, planning abort\n");
		return false;
	}
	
	if ( !_findPath() )
	{
		RAVELOG_ERROR("No path found\n");
		return false;
	}
	
	/// create Trajectory from path found
	OpenRAVE::Trajectory::TPOINT pt;
    pt.q.resize(_pParameters->GetDOF());
    
    FOREACH ( itnode, _lPathNodes ) {
        for ( int i = 0; i < _pParameters->GetDOF(); ++i ) {
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
int VISPRM::_buildRoadMap()
{
	// Generate samples from the CSpace
	int i = 0;
	while (i < _pParameters->iMnodes) 
	{
		if (_sampler->GenSingleSample(_vRandomConfig) ) 
		{
			_vvSamples.push_back(_vRandomConfig);
		} 
		else 
		{
			RAVELOG_INFO("Failed to get a sample\n");
			continue;
		}
		i++;
	}
	
	RAVELOG_DEBUG("connecting samples\n");
	for (configSet::iterator it = _vvSamples.begin(); it != _vvSamples.end(); it++) 
	{
		std::list<s_node> neighbors;
		s_vertex vs = _gRoadmap->addNode(*it);

		RAVELOG_DEBUGA("added node\n");
		int nns = _gRoadmap->findNN(vs, neighbors);
		if (nns == 0)
		{
			continue;
		}
		
		for (std::list<s_node>::iterator itt = neighbors.begin(); itt != neighbors.end(); itt++) 
		{	
			if (!ICollision::CheckCollision(_pParameters, _pRobot, (*it), (*itt).nconfig, OPEN) )
			{
				if (!_gRoadmap->addEdge(vs, (*itt).vertex ))
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
	_gRoadmap->printGraph("visprm_roadmap.dot");

	return _gRoadmap->getNodes();
}

bool VISPRM::_findPath(s_node _start, s_node _goal)
{	
	if ( _gRoadmap->findPathAS(_start, _goal, _lPathNodes) )
	{
		RAVELOG_VERBOSE("Found Goal with A* \n");
		return true;
	}
	else if ( _gRoadmap->findPathDK(_start, _goal, _lPathNodes) )
	{
		RAVELOG_VERBOSE("Found Goal with Dijkstra \n");
		return true;
	}
	
	return false;
}

bool VISPRM::_addStart()
{
	if ((int)_pParameters->vinitialconfig.size() != _pRobot->GetActiveDOF())
	{
		RAVELOG_ERROR("Specified Start Configuration is valid\n");
		return false;
	}
	
	
	s_vertex st = _gRoadmap->addNode(_pParameters->vinitialconfig);
	std::list<s_node> nearsamples;
	
	RAVELOG_INFO("Getting near samples for start [%d]\n", st);
	
	int neis = _gRoadmap->findNN(st, nearsamples);
	RAVELOG_INFO("neighbors [%d]\n", neis);
	
	if ( neis == 0)
	{
		RAVELOG_WARN("Warning! Start node too far from the roadmap\n");
		return false;
	}
		
	RAVELOG_INFO("Adding near samples\n");
	
	for (std::list<s_node>::iterator it = nearsamples.begin(); it != nearsamples.end(); it++)
	{
		s_node nn = _gRoadmap->getNode(st);
		if (!ICollision::CheckCollision(_pParameters, _pRobot, (*it).nconfig, nn.nconfig, OPEN) )
		{
			if (_gRoadmap->addEdge((*it).vertex, st))
			{
				RAVELOG_INFO("Added the start configuration\n");
				_nStart = nn;
				break;
			}
		}
		else
		{
			continue;
		}
	}

	if ( _nStart.vertex > (2*_gRoadmap->getNodes()) )
	{
		RAVELOG_ERROR("Added invalid start configuration\n");
		return false;
	}

	return true;
}

bool VISPRM::_addGoal()
{
	if ((int)_pParameters->vgoalconfig.size() != _pRobot->GetActiveDOF())
	{
		RAVELOG_ERROR("Specified Goal Configuration is valid\n");
		return false;
	}
	
	
	s_vertex st = _gRoadmap->addNode(_pParameters->vgoalconfig);
	std::list<s_node> nearsamples;
	
	RAVELOG_INFO("Getting near samples for goal [%d]\n", st);
	
	int neis = _gRoadmap->findNN(st, nearsamples);
	RAVELOG_INFO("neighbors [%d]\n", neis);
	
	if ( neis == 0)
	{
		RAVELOG_WARN("Warning! Goal node too far from the roadmap\n");
		return false;
	}
		
	RAVELOG_INFO("Adding near samples\n");
	
	for (std::list<s_node>::iterator it = nearsamples.begin(); it != nearsamples.end(); it++)
	{
		s_node nn = _gRoadmap->getNode(st);
		if (!ICollision::CheckCollision(_pParameters, _pRobot, (*it).nconfig, nn.nconfig, OPEN) )
		{
			if (_gRoadmap->addEdge((*it).vertex, st))
			{
				RAVELOG_INFO("Added the Goal configuration\n");
				_nGoal = nn;
				break;
			}
		}
		else
		{
			continue;
		}
	}

	if ( _nGoal.vertex > (2*_gRoadmap->getNodes()) )
	{
		RAVELOG_ERROR("Added invalid goal configuration\n");
		return false;
	}
	
	return true;
}

}

#endif
