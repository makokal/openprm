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
#include "roadmap_inspector.h"

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

    RobotBasePtr p_robot;
    std::string s_planner_name;
    std::string s_robot_name;
    bool b_reuseplanner;
    bool b_execute;
    string s_traj_filename;
    boost::shared_ptr<ostream> p_output_traj_stream;



    bool GrabBody ( ostream& sout, istream& sinput );
    bool ReleaseAll ( ostream& sout, istream& sinput );
    bool Traj ( ostream& sout, istream& sinput );
    bool SetActiveTrajectory ( RobotBasePtr robot, TrajectoryBasePtr pActiveTraj, bool bExecute, const string& strsavetraj, boost::shared_ptr<ostream> pout);
    bool RunPRM ( ostream& sout, istream& sinput );
    bool BuildRoadMap ( ostream& sout, istream& sinput );
    bool RunQuery ( ostream& sout, istream& sinput );
    bool TestPrmGraph ( ostream& sout, istream& sinput );


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

    RegisterCommand("TestPrmGraph",boost::bind(&PRMPlanning::TestPrmGraph,this,_1,_2),
                    "Test the prm graph by sampling configs and displaying map (PRMPlanning::TestPrmGraph).");
}

PRMPlanning::~PRMPlanning()
{
    Destroy();
}


/** ======================================================================================= */

int PRMPlanning::main ( const std::string& args )
{
    stringstream ss(args);
    ss >> s_robot_name;

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
            ss >> s_planner_name;
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
    if ( s_planner_name.size() > 0 )
    {
        planner = RaveCreatePlanner(GetEnv(), s_planner_name);
    }

    if ( !planner )
    {
        s_planner_name = "classicprm";
        planner = RaveCreatePlanner(GetEnv(), s_planner_name);
        if ( !planner )
        {
            s_planner_name = "";
        }
    }

    RAVELOG_DEBUGA(str(boost::format("PRM Manipulation Planning: using %s planner\n")%s_planner_name));
    return 0;
}


/** ======================================================================================= */

void PRMPlanning::Destroy()
{
    p_robot.reset();
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
        if ( strcmp((*itrobot)->GetName().c_str(), s_robot_name.c_str() ) == 0  ) {
            p_robot = *itrobot;
            break;
        }
    }

    if ( p_robot == NULL ) {
        RAVELOG_ERRORA("Failed to find %S\n", s_robot_name.c_str());
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

    p_robot->Grab(ptarget);
    return true;
}


/** ======================================================================================= */

bool PRMPlanning::Traj ( ostream& sout, istream& sinput ) {
    string filename;
    sinput >> filename;
    if ( !sinput )
        return false;

    TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),p_robot->GetDOF());

    char sep = ' ';
    if ( filename == "sep" ) {
        sinput >> sep;
        filename = getfilename_withseparator(sinput,sep);
    }

    if ( filename == "stream" ) {
        // the trajectory is embedded in the stream
        RAVELOG_VERBOSE("PRMPlanning: reading trajectory from stream\n");

        if ( !ptraj->Read(sinput, p_robot) ) {
            RAVELOG_ERROR("PRMPlanning: failed to get trajectory\n");
            return false;
        }
    }
    else {
        RAVELOG_VERBOSE(str(boost::format("PRMPlanning: reading trajectory: %s\n")%filename));
        ifstream f(filename.c_str());
        if ( !ptraj->Read(f, p_robot) ) {
            RAVELOG_ERROR(str(boost::format("PRMPlanning: failed to read trajectory %s\n")%filename));
            return false;
        }
    }

    bool bResetTrans = false;
    sinput >> bResetTrans;

    if ( bResetTrans ) {
        RAVELOG_VERBOSE("resetting transformations of trajectory\n");
        Transform tcur = p_robot->GetTransform();
        // set the transformation of every point to the current robot transformation
        FOREACH(itpoint, ptraj->GetPoints()) {
            itpoint->trans = tcur;
        }
    }

    RAVELOG_VERBOSE(str(boost::format("executing traj with %d points\n")%ptraj->GetPoints().size()));
    p_robot->SetMotion(ptraj);
    sout << "1";
    return true;
}


/** ======================================================================================= */

bool PRMPlanning::ReleaseAll ( ostream& sout, istream& sinput )
{
    if ( !!p_robot )
    {
        RAVELOG_DEBUGA("Releasing all bodies\n");
        p_robot->ReleaseAllGrabbed();
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
    RobotBase::ManipulatorPtr pmanip = p_robot->GetActiveManipulator();

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
            p_output_traj_stream = boost::shared_ptr<ostream>(&sout,null_deleter());
        }
        else if ( cmd == "execute" )
        {
            sinput >> b_execute;
        }
        else if ( cmd == "writetraj" )
        {
            sinput >> s_traj_filename;
        }
        else if ( cmd == "jointgoals" || cmd == "armvals" || cmd == "goal" )
        {
//            goals.resize(pmanip->GetArmIndices().size());
            goals.resize(p_robot->GetActiveDOF());
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
        params->vinitialconfig.resize(p_robot->GetActiveDOF());
        p_robot->GetActiveDOFValues(params->vinitialconfig);
    }
    else
    {
        //add any starts if they were specified
        for (int i = 0; i < (int)starts.size(); i++)
        {
            params->vinitialconfig.push_back(starts[i]);
        }
    }

//    if ( goals.size() != pmanip->GetArmIndices().size() ) {
//        RAVELOG_WARN("Invalid params [manip indices=%d] and [vgoal size=%d]\n", pmanip->GetArmIndices().size(), goals.size());
//        return false;
//    }

    //add any goals if they were specified
    if ( goals.size() > 0 )
    {
        RAVELOG_DEBUGA("adding speficied goals of size %d\n", (int)goals.size());
        for (int i = 0; i < (int)goals.size(); i++) {
            params->vgoalconfig.push_back(goals[i]);
        }
    }

    RAVELOG_DEBUGA("setting robot to initial config\n");
    // need to check constraints here
    p_robot->SetActiveDOFValues(params->vinitialconfig);

    boost::shared_ptr<Trajectory> ptraj(RaveCreateTrajectory(GetEnv(),p_robot->GetActiveDOF()));

    RAVELOG_DEBUGA("preparing trajectory\n");
    Trajectory::TPOINT pt;
    pt.q = params->vinitialconfig;
    ptraj->AddPoint(pt);

    RAVELOG_DEBUGA("creating planner\n");
    boost::shared_ptr<PlannerBase> p_planner = RaveCreatePlanner(GetEnv(),s_planner_name);
    if ( !p_planner ) {
        RAVELOG_ERROR(str(boost::format("Failed to create %s PRM planner\n")%s_planner_name));
        return false;
    }

    RAVELOG_DEBUGA("starting planning\n");
    // now plan and write trajectory
    bool b_success = false;
    for (int iter = 0; iter < nMaxTries; ++iter)
    {
        RAVELOG_DEBUGA("interation %d \n",iter);
        if ( !p_planner->InitPlan(p_robot, params) )
        {
            RAVELOG_ERROR("InitPlan failed\n");
            return false;
        }
        RAVELOG_DEBUGA("init plan finished\n");
        if ( p_planner->PlanPath(ptraj) )
        {
            b_success = true;
            RAVELOG_DEBUGA("finished planning\n");
            break;
        }
        else
            RAVELOG_WARNA("PlanPath failed, trying again\n");
    }

    p_planner.reset(); // have to destroy before environment

    if ( !b_success ) {
        return false;
    }

    // save the traj
    TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),p_robot->GetDOF());
    p_robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
    pfulltraj->CalcTrajTiming(p_robot, pfulltraj->GetInterpMethod(), true, false);
    ofstream outfile(s_traj_filename.c_str(), ios::out);
    pfulltraj->Write(outfile, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    outfile.close();
    //chmod(filename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU); //chmod 777

    // execute
    SetActiveTrajectory(p_robot, ptraj, b_execute, s_traj_filename, p_output_traj_stream);
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





bool PRMPlanning::TestPrmGraph( ostream& sout, istream& sinput )
{
    //! create a simple graph and render it on the viewer
//    RAVELOG_WARN("Not implemented yet\n");

    boost::shared_ptr<RMapInspector> p_graph_test;

    p_graph_test.reset(new RMapInspector(50));
    p_graph_test->generateSamples();
    p_graph_test->createGraph();
    p_graph_test->renderGraph();

    return false;
}


}
#endif // PRMPLANNING_H
