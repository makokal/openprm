

#ifndef SBLPLANNER_H
#define SBLPLANNER_H

#include "prmparams.h"
#include "../utils/spatialrep.h"
#include "samplerbase.h"

namespace openprm
{

class SBLPlanner : public PlannerBase {
public:
	SBLPlanner( EnvironmentBasePtr penv );
	
	virtual ~SBLPlanner();
	
	bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams);
	
	bool PlanPath(TrajectoryBasePtr ptraj, boost::shared_ptr<std::ostream> pOutStream);
	
	virtual PlannerParametersConstPtr GetParameters() const;
	
	virtual RobotBasePtr GetRobot() const; 
	
protected:
	
	SpatialTree<SBLPlanner, SimpleNode > _goalMap;
	
    boost::shared_ptr<PRMParams> _parameters;

    boost::shared_ptr<RandomSampler> p_sampler;
	
	std::vector< std::vector<dReal> > _vecGoals;
	
	SpatialTreeBase* _treeS;
	
	SpatialTreeBase* _treeG;
	
	float _fGoalBiasProb;
	
	bool _bOneStep, bConnected;
	
	int iConnectedS, iConnectedG;
	
	RobotBasePtr _robot;

	std::vector<dReal> _randomConfig;
		
	CollisionReportPtr _report;

	SpatialTree< SBLPlanner, SimpleNode > _roadMap;

	
	
	inline boost::shared_ptr<SBLPlanner> shared_planner();
		
	inline boost::shared_ptr<SBLPlanner const> shared_planner_const() const;
	
	void buildFrom(int _lastIndex, int _goalIndex);
};


SBLPlanner::SBLPlanner ( EnvironmentBasePtr penv ) : PlannerBase ( penv ) {
    _fGoalBiasProb = 0.10f;
    _bOneStep = true;
    bConnected = false;
}

SBLPlanner::~SBLPlanner() {
}

bool SBLPlanner::InitPlan ( RobotBasePtr pbase, PlannerParametersConstPtr pparams ) {
    RAVELOG_DEBUGA("SBL::Initializing...\n");

    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    _parameters.reset();
    boost::shared_ptr<PRMParams> parameters(new PRMParams());
    parameters->copy(pparams);

    _robot = pbase;
    p_sampler.reset<RandomSampler>(new RandomSampler(_robot));

    // set up the roadmap and the sample config
    _randomConfig.resize(pparams->GetDOF());
    _roadMap.Reset(shared_planner(), pparams->GetDOF());
    _roadMap._fStepLength = pparams->_fStepLength;
    _roadMap._distmetricfn = pparams->_distmetricfn;
    _roadMap.AddNode(0, pparams->vinitialconfig);

    // set up the corresponding map from the goal
    _goalMap.Reset(shared_planner(), pparams->GetDOF());
    _goalMap._fStepLength = pparams->_fStepLength;
    _goalMap._distmetricfn = pparams->_distmetricfn;
    _goalMap.AddNode(-10000, pparams->vgoalconfig);


    // RAVELOG_DEBUGA("reading goals\n");
    //     //read goals
    //     int goal_index = 0;
    //     int num_goals = 0;
    //     vector<dReal> vgoal(parameters->GetDOF());
    //
    //     while ( goal_index < (int)parameters->vgoalconfig.size() ) {
    // 		for ( int i = 0 ; i < parameters->GetDOF(); i++ ) {
    // 			if ( goal_index < (int)parameters->vgoalconfig.size() ) {
    // 				vgoal[i] = parameters->vgoalconfig[goal_index];
    // 			} else {
    // 				RAVELOG_ERRORA("SBLPlanner::InitPlan - Error: goals are improperly specified:\n");
    // 				parameters.reset();
    // 				return false;
    // 			}
    // 			goal_index++;
    // 		}
    //
    // 		RAVELOG_DEBUGA("checking collisions\n");
    // 		if ( !CollisionFunctions::CheckCollision(parameters,_robot,vgoal)) {
    // 			bool bSuccess = true;
    // 			if ( !!parameters->_constraintfn ) {
    // 				// filter
    // 				if ( !parameters->_constraintfn(vgoal, vgoal, 0) ) {
    // 					// failed
    // 					RAVELOG_WARNA("goal state rejected by constraint fn\n");
    // 					bSuccess = false;
    // 				}
    // 			}
    //
    // 			if ( bSuccess ) {
    // 				// set up the goal states
    // 				//_goalMap.AddNode(num_goals-1, vgoal);
    // 				num_goals++;
    // 			}
    // 		} else {
    // 			RAVELOG_WARNA("goal in collision %s\n", _robot->CheckSelfCollision()?"(self)":NULL);
    // 			if ( GetEnv()->CheckCollision(KinBodyConstPtr(_robot), _report) || _robot->CheckSelfCollision(_report) ) {
    // 				RAVELOG_WARN(str(boost::format("SBLPlanner: robot initially in collision %s!\n")%_report->__str__()));
    // 			}
    // 		}
    // 	}
    //
    // check if some goals were specified
    // if ( num_goals == 0 && !parameters->_samplegoalfn ) {
    // 		RAVELOG_WARNA("no goals specified\n");
    // 		parameters.reset();
    // 		return false;
    //     }

    RAVELOG_DEBUGA("setting up trees\n");
    // set up the trees
    _treeS = &_roadMap;
    _treeG = &_goalMap;

    _parameters=parameters;
    RAVELOG_DEBUGA("SBLPlanner::InitPlan - SBL Planner Initialized\n");
    return true;
}

bool SBLPlanner::PlanPath ( TrajectoryBasePtr ptraj, boost::shared_ptr< ostream > pOutStream ) {
    if ( !_parameters ) {
        RAVELOG_WARNA("SBLPlanner::PlanPath - Error, planner not initialized\n");
        return false;
    }

    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    uint32_t basetime = timeGetTime();

    // the main planning loop
    // grow two trees from start and goal
    // delay collision checking up to only when needed

    // build the expansive trees from start and goal
    RAVELOG_DEBUGA("starting to build roadmap \n");

    buildFrom(0, 0);

    RAVELOG_DEBUGA("SBLPlanner::RoadMap built \n");

    if ( !bConnected ) {
        RAVELOG_DEBUGA("plan failed(cound not connect roadmaps), %fs\n",0.001f*(float)(timeGetTime()-basetime));
        return false;
    }

    list<SimpleNode*> _pathNodes;

    RAVELOG_DEBUGA("SBLPlanner::Connecting Trees... \n");

    // add from start tree
    SimpleNode* _stNode = _roadMap._nodes.at(_treeS == &_roadMap ? iConnectedS : iConnectedG);
    while ( 1 ) {
        _pathNodes.push_front(_stNode);

        if (_stNode->parent < 0) { break; }

        _stNode = _roadMap._nodes.at(_stNode->parent);
    }

    // add from goal tree
    int goalindex = -1;
    SimpleNode* _gtNode = _goalMap._nodes.at(_treeS == &_goalMap ? iConnectedS : iConnectedG);
    while ( 1 ) {
        if ( ICollision::CheckCollision(GetParameters(), _robot, _gtNode->q) ) {
            buildFrom(_stNode->parent, _gtNode->parent);
            continue;
        }

        _pathNodes.push_back(_gtNode);
        if (_gtNode->parent < 0) {
            goalindex = _gtNode->parent - 1;
            break;
        }
        _gtNode = _goalMap._nodes.at(_gtNode->parent);
    }

    // _SimpleOptimizePath(vecnodes,10);
    RAVELOG_DEBUGA("SBLPlanner::Checking goal index \n");

    // 	BOOST_ASSERT( goalindex >= 0 );
    if( pOutStream != NULL ) {
        *pOutStream << goalindex;
    }

    RAVELOG_DEBUGA("SBLPlanner::Setting up Trajectory \n");

    Trajectory::TPOINT pt; pt.q.resize(_parameters->GetDOF());
    FOREACH(itnode, _pathNodes) {
        for(int i = 0; i < _parameters->GetDOF(); ++i) {
            pt.q[i] = (*itnode)->q[i];
        }

        ptraj->AddPoint(pt);
    }

    //            _OptimizePath(_robot,ptraj);
    RAVELOG_DEBUGA(str(boost::format("plan success, path=%d points in %fs\n")%ptraj->GetPoints().size()%((0.001f*(float)(timeGetTime()-basetime)))));

    return true;
}

PlannerBase::PlannerParametersConstPtr SBLPlanner::GetParameters() const {
    return _parameters;
}

RobotBasePtr SBLPlanner::GetRobot() const {
    return _robot;
}

boost::shared_ptr<SBLPlanner> SBLPlanner::shared_planner() {
    return boost::static_pointer_cast<SBLPlanner>(shared_from_this());
}

boost::shared_ptr<SBLPlanner const> SBLPlanner::shared_planner_const() const {
    return boost::static_pointer_cast<SBLPlanner const>(shared_from_this());
}

void SBLPlanner::buildFrom ( int _lastIndex, int _goalIndex ) {
    //RobotBase::RobotStateSaver savestate(_robot);
    int iter = 0;

    while( !bConnected ) {
        RAVELOG_DEBUGA("Building RoadMap iter: %d\n", iter);
        iter++;

        // if ( RaveRandomFloat() < _fGoalBiasProb ) { //&& _vecGoals.size() > 0 ) {
        // 			_randomConfig = _vecGoals[RaveRandomInt()%_vecGoals.size()];
        // 		} else if ( !_parameters->_samplefn(_randomConfig) ) {
        // 			continue;
        // 		}

//        if ( !_parameters->_samplefn(_randomConfig) ) {
//            continue;
//        }

        if (!(p_sampler->GenSingleSample(_randomConfig)) )
        {
            RAVELOG_DEBUGA("Error in sampling");
            continue;
        }

        RAVELOG_DEBUGA("extending start tree\n");
        _treeS->Extend(_treeS->GetConfig(_lastIndex), iConnectedS);
        ExtendType et = _treeS->Extend(_randomConfig, iConnectedS);

        if ( et == ET_Failed ) { continue; }

        RAVELOG_DEBUGA("extending goal tree\n");
        _treeG->Extend(_treeG->GetConfig(_goalIndex), iConnectedG);
        et = _treeG->Extend(_treeS->GetConfig(iConnectedS), iConnectedG);

        if ( et == ET_Connected ) {
            bConnected = true;
            break;	// we are done, proceed to check collisions
        }

        swap(_treeS, _treeG);
        iter +=3;
    }
}

}

#endif // SBLPLANNER_H
