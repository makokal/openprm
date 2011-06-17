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
  * \file prmplanning.h
  * \author Billy Okal
  * \brief PRM Planning interface: defines commands to access prm plannners
  */

#ifndef PRMPLANNING_H
#define PRMPLANNING_H

#include "prmparams.h"

namespace openprm
{

class PRMPlanning : public ProblemInstance
{
public:

    PRMPlanning ( EnvironmentBasePtr penv );
    virtual ~PRMPlanning();
    virtual void Destroy();
    virtual int main ( const std::string& args );
    virtual void SetActiveRobots ( const std::vector<RobotBasePtr>& robots );
    virtual bool SendCommand ( std::ostream& sout, std::istream& sinput );

protected:

    RobotBasePtr _robot;
    std::string _plannerName;
    std::string _robotName;
    bool bReusePlanner;
    bool bExecute;
    string _trajFilename;
    boost::shared_ptr<ostream> _outputTrajStream;



    bool GrabBody ( ostream& sout, istream& sinput );
    bool ReleaseAll ( ostream& sout, istream& sinput );
    bool Traj ( ostream& sout, istream& sinput );
    bool SetActiveTrajectory ( RobotBasePtr robot, TrajectoryBasePtr pActiveTraj, bool bExecute, const string& strsavetraj, boost::shared_ptr<ostream> pout);
    bool RunPRM ( ostream& sout, istream& sinput );
    bool BuildRoadMap ( ostream& sout, istream& sinput );
    bool RunQuery ( ostream& sout, istream& sinput );

    inline std::string getfilename_withseparator(istream& sinput, char separator)
    {
        string filename;
        if ( !getline(sinput, filename, separator) ) {
            // just input directly
            RAVELOG_ERRORA("filename not terminated with ';'\n");
            sinput >> filename;
        }

        // trim leading spaces
        size_t startpos = filename.find_first_not_of(" \t");
        size_t endpos = filename.find_last_not_of(" \t");

        // if all spaces or empty return an empty string
        if (( string::npos == startpos ) || ( string::npos == endpos))
            return "";

        filename = filename.substr( startpos, endpos-startpos+1 );
        return filename;
    }

    inline boost::shared_ptr<PRMPlanning> shared_problem()
    {
        return boost::static_pointer_cast< PRMPlanning >(shared_from_this());
    }

    inline boost::shared_ptr<PRMPlanning const> shared_problem_const() const
    {
        return boost::static_pointer_cast< PRMPlanning const >(shared_from_this());
    }

};



/** ======================================================================================= */

PRMPlanning::PRMPlanning( EnvironmentBasePtr penv ) : ProblemInstance ( penv )
{
    __description = "PRM Planning";

    //Register new commands
    RegisterCommand("RunPRM", boost::bind(&PRMPlanning::RunPRM, this, _1, _2),
                    "Build the RoadMap and querry it, combines two steps (for single short querries)");
    RegisterCommand("BuildRoadMap", boost::bind(&PRMPlanning::BuildRoadMap, this, _1, _2),
                    "Build the RoadMap based on the current state of the Configuration Space");
    RegisterCommand("RunQuery", boost::bind(&PRMPlanning::RunQuery, this, _1, _2),
                    "Run a query on an already built roadmap");

    RegisterCommand("Traj",boost::bind(&PRMPlanning::Traj,this,_1,_2),
                    "Execute a trajectory from a file on the local filesystem");

    RegisterCommand("GrabBody",boost::bind(&PRMPlanning::GrabBody,this,_1,_2),
                    "Robot calls ::Grab on a body with its current manipulator");

    RegisterCommand("ReleaseAll",boost::bind(&PRMPlanning::ReleaseAll,this,_1,_2),
                    "Releases all grabbed bodies (RobotBase::ReleaseAllGrabbed).");
}

PRMPlanning::~PRMPlanning()
{
    Destroy();
}


/** ======================================================================================= */

int PRMPlanning::main ( const std::string& args )
{
    stringstream ss(args);
    ss >> _robotName;

    string cmd;
    while (!ss.eof())
    {
        ss >> cmd;

        if ( !ss )
        {
            break;
        }

        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        if ( cmd == "planner" )
        {
            ss >> _plannerName;
        }

        if ( ss.fail() || !ss )
        {
            break;
        }
    }

    std::vector<RobotBasePtr> robots;
    GetEnv()->GetRobots(robots);
    SetActiveRobots(robots);

    PlannerBasePtr planner;
    if ( _plannerName.size() > 0 )
    {
        planner = RaveCreatePlanner(GetEnv(), _plannerName);
    }

    if ( !planner )
    {
        _plannerName = "classicprm";
        planner = RaveCreatePlanner(GetEnv(), _plannerName);
        if ( !planner )
        {
            _plannerName = "";
        }
    }

    RAVELOG_DEBUGA(str(boost::format("PRM Manipulation Planning: using %s planner\n")%_plannerName));
    return 0;
}


/** ======================================================================================= */

void PRMPlanning::Destroy()
{
    _robot.reset();
    ProblemInstance::Destroy();
}

bool PRMPlanning::SendCommand ( ostream& sout, istream& sinput )
{
    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    return ProblemInstance::SendCommand(sout,sinput);
}

/** ======================================================================================= */

void PRMPlanning::SetActiveRobots ( const std::vector< RobotBasePtr >& robots ) {
    if ( robots.size() == 0 ) {
        RAVELOG_WARNA("No robots to plan for\n");
        return;
    }

    vector<RobotBasePtr >::const_iterator itrobot;
    FORIT (itrobot, robots) {
        if ( strcmp((*itrobot)->GetName().c_str(), _robotName.c_str() ) == 0  ) {
            _robot = *itrobot;
            break;
        }
    }

    if ( _robot == NULL ) {
        RAVELOG_ERRORA("Failed to find %S\n", _robotName.c_str());
        return;
    }
}


/** ======================================================================================= */
/** Commands
/-- ======================================================================================= */

bool PRMPlanning::GrabBody ( ostream& sout, istream& sinput ) {
    RAVELOG_DEBUG("Starting GrabBody...\n");

    KinBodyPtr ptarget;
    string cmd;

    while (!sinput.eof()) {
        sinput >> cmd;
        if ( !sinput )
            break;

        if ( cmd == "name") {
            string name;
            sinput >> name;
            ptarget = GetEnv()->GetKinBody(name.c_str());
        }
        else break;

        if ( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }

    if ( ptarget == NULL ) {
        RAVELOG_INFO("ERROR PRMPlanning::GrabBody - Invalid body name.\n");
        return false;
    }

    _robot->Grab(ptarget);
    return true;
}


/** ======================================================================================= */

bool PRMPlanning::Traj ( ostream& sout, istream& sinput ) {
    string filename;
    sinput >> filename;
    if ( !sinput )
        return false;

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),_robot->GetDOF());

    char sep = ' ';
    if ( filename == "sep" ) {
        sinput >> sep;
        filename = getfilename_withseparator(sinput,sep);
    }

    if ( filename == "stream" ) {
        // the trajectory is embedded in the stream
        RAVELOG_VERBOSE("PRMPlanning: reading trajectory from stream\n");

        if ( !ptraj->Read(sinput, _robot) ) {
            RAVELOG_ERROR("PRMPlanning: failed to get trajectory\n");
            return false;
        }
    }
    else {
        RAVELOG_VERBOSE(str(boost::format("PRMPlanning: reading trajectory: %s\n")%filename));
        ifstream f(filename.c_str());
        if ( !ptraj->Read(f, _robot) ) {
            RAVELOG_ERROR(str(boost::format("PRMPlanning: failed to read trajectory %s\n")%filename));
            return false;
        }
    }

    bool bResetTrans = false;
    sinput >> bResetTrans;

    if ( bResetTrans ) {
        RAVELOG_VERBOSE("resetting transformations of trajectory\n");
        Transform tcur = _robot->GetTransform();
        // set the transformation of every point to the current robot transformation
        FOREACH(itpoint, ptraj->GetPoints()) {
            itpoint->trans = tcur;
        }
    }

    RAVELOG_VERBOSE(str(boost::format("executing traj with %d points\n")%ptraj->GetPoints().size()));
    _robot->SetMotion(ptraj);
    sout << "1";
    return true;
}


/** ======================================================================================= */

bool PRMPlanning::ReleaseAll ( ostream& sout, istream& sinput )
{
    if ( !!_robot )
    {
        RAVELOG_DEBUGA("Releasing all bodies\n");
        _robot->ReleaseAllGrabbed();
    }
    return true;
}


/** ======================================================================================= */

bool PRMPlanning::BuildRoadMap ( ostream& sout, istream& sinput )
{
    RAVELOG_WARN("Not implemented yet\n");
    return false;
}


/** ======================================================================================= */

bool PRMPlanning::RunQuery ( ostream& sout, istream& sinput )
{
    RAVELOG_WARN("Not implemented yet\n");
    return false;
}

/** ======================================================================================= */

bool PRMPlanning::RunPRM ( ostream& sout, istream& sinput )
{
    boost::shared_ptr<PRMParams> params;
    params.reset(new PRMParams());
    std::vector<dReal> goals;
    std::vector<dReal> starts;
    int nMaxTries=3;
    RobotBase::ManipulatorPtr pmanip = _robot->GetActiveManipulator();

    string cmd;
    while ( !sinput.eof() )
    {
        sinput >> cmd;
        RAVELOG_DEBUGA(str(boost::format(" [cmd=%s] ")%cmd));
        if ( !sinput )
        {
            break;
        }
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if ( cmd == "outputtraj" )
        {
            _outputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
        }
        else if ( cmd == "execute" )
        {
            sinput >> bExecute;
        }
        else if ( cmd == "writetraj" )
        {
            sinput >> _trajFilename;
        }
        else if ( cmd == "jointgoals" || cmd == "armvals" || cmd == "goal" )
        {
            goals.resize(pmanip->GetArmIndices().size());
            FOREACH(it, goals)
            {
                sinput >> *it;
            }
        }
        else if ( cmd == "jointstarts")
        {
            int temp;
            sinput >> temp;
            int oldsize = starts.size();
            starts.resize(oldsize+temp);
            for (int i = oldsize; i < oldsize+temp; i++)
            {
                sinput >> starts[i];
            }
        }
        else if ( cmd == "pruneroadmap" )
        {
            sinput >> params->b_prune_roadmap;
        }
        else if ( cmd == "smoothpath" )
        {
            sinput >> params->b_smooth_path;
        }
        else if ( cmd == "ntries" )
        {
            sinput >> params->i_ntries;
        }
        else if ( cmd == "mnodes" )
        {
            sinput >> params->i_nnodes;
        }
        else if ( cmd == "medges" )
        {
            sinput >> params->i_nedges;
        }
        else if ( cmd == "neighthresh" )
        {
            sinput >> params->f_neigh_thresh;
        }


        if ( !sinput ) {
            RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
            return false;
        }
    }

    if ( starts.size() == 0 )
    {
        RAVELOG_INFO("Setting initial configuration to the current configuration\n");
        //default case: when not sampling starts and no starts specified, use current config as start
        params->vinitialconfig.resize(_robot->GetActiveDOF());
        _robot->GetActiveDOFValues(params->vinitialconfig);
    }
    else
    {
        //add any starts if they were specified
        for (int i = 0; i < (int)starts.size(); i++)
        {
            params->vinitialconfig.push_back(starts[i]);
        }
    }

    if ( goals.size() != pmanip->GetArmIndices().size() ) {
        RAVELOG_WARN("Invalid params [manip indices=%d] and [vgoal size=%d]\n", pmanip->GetArmIndices().size(), goals.size());
        return false;
    }

    //add any goals if they were specified
    if ( goals.size() > 0 )
    {
        RAVELOG_DEBUGA("adding speficied goals\n");
        for (int i = 0; i < (int)goals.size(); i++) {
            params->vgoalconfig.push_back(goals[i]);
        }
    }

    RAVELOG_DEBUGA("setting robot to initial config\n");
    // need to check constraints here
    _robot->SetActiveDOFValues(params->vinitialconfig);

    boost::shared_ptr<Trajectory> ptraj(RaveCreateTrajectory(GetEnv(),_robot->GetActiveDOF()));

    RAVELOG_DEBUGA("preparing trajectory\n");
    Trajectory::TPOINT pt;
    pt.q = params->vinitialconfig;
    ptraj->AddPoint(pt);

    RAVELOG_DEBUGA("creating planner\n");
    boost::shared_ptr<PlannerBase> _planner = RaveCreatePlanner(GetEnv(),_plannerName);
    if ( !_planner ) {
        RAVELOG_ERROR(str(boost::format("Failed to create %s PRM planner\n")%_plannerName));
        return false;
    }

    RAVELOG_DEBUGA("starting planning\n");
    // now plan and write trajectory
    bool bSuccess = false;
    for (int iter = 0; iter < nMaxTries; ++iter)
    {
        RAVELOG_DEBUGA("interation %d \n",iter);
        if ( !_planner->InitPlan(_robot, params) )
        {
            RAVELOG_ERROR("InitPlan failed\n");
            return false;
        }
        RAVELOG_DEBUGA("init plan finished\n");
        if ( _planner->PlanPath(ptraj) )
        {
            bSuccess = true;
            RAVELOG_INFO("finished planning\n");
            break;
        }
        else
            RAVELOG_WARN("PlanPath failed\n");
    }

    _planner.reset(); // have to destroy before environment

    if ( !bSuccess ) {
        return false;
    }

    // save the traj
    TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),_robot->GetDOF());
    _robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
    pfulltraj->CalcTrajTiming(_robot, pfulltraj->GetInterpMethod(), true, false);
    ofstream outfile(_trajFilename.c_str(), ios::out);
    pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    outfile.close();
    //chmod(filename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777

    // execute
    SetActiveTrajectory(_robot, ptraj, bExecute, _trajFilename, _outputTrajStream);
    sout << "1";

    return true;
}


/** ======================================================================================= */

bool PRMPlanning::SetActiveTrajectory ( RobotBasePtr robot, TrajectoryBasePtr pActiveTraj, bool bExecute, const std::string& strsavetraj, boost::shared_ptr< ostream > pout ) {
    if	( pActiveTraj->GetPoints().size() == 0 ) {
        return false;
    }

    pActiveTraj->CalcTrajTiming(robot, pActiveTraj->GetInterpMethod(), true, true);

    bool bExecuted = false;
    if ( bExecute ) {
        if ( pActiveTraj->GetPoints().size() > 1 ) {
            robot->SetActiveMotion(pActiveTraj);
            bExecute = true;
        }
        // have to set anyway since calling script will orEnvWait!
        else if ( !!robot->GetController() ) {
            TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(), robot->GetDOF());
            robot->GetFullTrajectoryFromActive(pfulltraj, pActiveTraj);

            if ( robot->GetController()->SetDesired(pfulltraj->GetPoints()[0].q) )
                bExecuted = true;
        }
    }

    if ( strsavetraj.size() || !!pout ) {
        TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(), robot->GetDOF());
        robot->GetFullTrajectoryFromActive(pfulltraj, pActiveTraj);

        if ( strsavetraj.size() > 0 ) {
            ofstream f(strsavetraj.c_str());
            pfulltraj->Write(f, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        }

        if ( !!pout ) {
            pfulltraj->Write(*pout, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation|Trajectory::TO_OneLine);
        }
    }

    return bExecuted;
}

}

#endif // PRMPLANNING_H
