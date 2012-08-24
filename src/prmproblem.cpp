/// Copyright (c) 2010-2012, Billy Okal sudo@makokal.com
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
/// * Redistributions of source code must retain the above copyright
///   notice, this list of conditions and the following disclaimer.
/// * Redistributions in binary form must reproduce the above copyright
///   notice, this list of conditions and the following disclaimer in the
///   documentation and/or other materials provided with the distribution.
/// * Neither the name of the author nor the
///   names of its contributors may be used to endorse or promote products
///   derived from this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY the author ''AS IS'' AND ANY
/// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL the author BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
/// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#include <prmproblem.h>

using namespace openprm;


PRMProblem::PRMProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
{
    __description = "Planning using PRM based planners (Billy Okal)";

    /// Register new commands
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

    reuseplanner_ = false;
}


PRMProblem::~PRMProblem()
{
    Destroy();
}




void PRMProblem::Destroy()
{
    robot_ptr_.reset();
    ProblemInstance::Destroy();
}



int PRMProblem::main(const string &args)
{
    stringstream ss(args);
    ss >> robot_name_;

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
            ss >> planner_name_;
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
    if ( planner_name_.size() > 0 )
    {
        planner = RaveCreatePlanner(GetEnv(), planner_name_);
    }

    if ( !planner )
    {
        planner_name_ = "basicprm";
        planner = RaveCreatePlanner(GetEnv(), planner_name_);
        if ( !planner )
        {
            planner_name_ = "";
        }
    }

    RAVELOG_DEBUG(str(boost::format("PRM Planning: using %s planner\n")%planner_name_));

    return 0;
}




void PRMProblem::SetActiveRobots(const vector &robots)
{
    if ( robots.size() == 0 )
    {
        RAVELOG_WARNA("No robots to plan for\n");
        return;
    }

    vector<RobotBasePtr >::const_iterator itrobot;
    FORIT (itrobot, robots)
    {
        if ( strcmp((*itrobot)->GetName().c_str(), robot_name_.c_str() ) == 0 )
        {
            robot_ptr_ = *itrobot;
            break;
        }
    }

    if ( robot_ptr_ == NULL )
    {
        RAVELOG_ERRORA("Failed to find %S\n", robot_name_.c_str());
        return;
    }
}


bool PRMProblem::SendCommand(ostream &sout, istream &sinput)
{
    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    return ProblemInstance::SendCommand(sout,sinput);
}









/// ============================ Protected methods ============================
/// Represent the commands available on tha planner interface when used from
/// Python or C++ or Matlab
/// ===========================================================================



bool PRMProblem::GrabBody(ostream &sout, istream &sinput)
{
    RAVELOG_DEBUG("Starting GrabBody...\n");

    KinBodyPtr ptarget;
    string cmd;

    while (!sinput.eof())
    {
        sinput >> cmd;
        if ( !sinput )
            break;

        if ( cmd == "name")
        {
            string name;
            sinput >> name;
            ptarget = GetEnv()->GetKinBody(name.c_str());
        }
        else
            break;

        if ( !sinput )
        {
            RAVELOG_DEBUG("failed\n");
            return false;
        }
    }

    if ( ptarget == NULL )
    {
        RAVELOG_INFO("ERROR PRMPlanning::GrabBody - Invalid body name.\n");
        return false;
    }

    robot_ptr_->Grab(ptarget);

    return true;
}




bool PRMProblem::ReleaseAll(ostream &sout, istream &sinput)
{
    if ( !!robot_ptr_ )
    {
        RAVELOG_DEBUGA("Releasing all bodies\n");
        robot_ptr_->ReleaseAllGrabbed();
    }
    return true;
}




bool PRMProblem::Traj(ostream &sout, istream &sinput)
{
    string filename;
    sinput >> filename;
    if ( !sinput )
        return false;

    TrajectoryBasePtr trajectory = RaveCreateTrajectory(GetEnv(), robot_ptr_->GetDOF());

    char sep = ' ';
    if ( filename == "sep" )
    {
        sinput >> sep;
        filename = getfilename_withseparator(sinput,sep);
    }

    if ( filename == "stream" )
    {
        // the trajectory is embedded in the stream
        RAVELOG_VERBOSE("PRMPlanning: reading trajectory from stream\n");

        if ( !trajectory->Read(sinput, robot_ptr_) )
        {
            RAVELOG_ERROR("PRMPlanning: failed to get trajectory\n");
            return false;
        }
    }
    else
    {
        RAVELOG_VERBOSE(str(boost::format("PRMPlanning: reading trajectory: %s\n")%filename));
        ifstream f(filename.c_str());
        if ( !trajectory->Read(f, robot_ptr_) )
        {
            RAVELOG_ERROR(str(boost::format("PRMPlanning: failed to read trajectory %s\n")%filename));
            return false;
        }
    }

    bool reset_transformations = false;
    sinput >> reset_transformations;

    if ( reset_transformations ) {
        RAVELOG_VERBOSE("resetting transformations of trajectory\n");
        Transform current_transform = robot_ptr_->GetTransform();
        // set the transformation of every point to the current robot transformation
        FOREACH(itpoint, trajectory->GetPoints()) {
            itpoint->trans = current_transform;
        }
    }

    RAVELOG_VERBOSE(str(boost::format("executing traj with %d points\n")%trajectory->GetPoints().size()));
    robot_ptr_->SetMotion(trajectory);
    sout << "1";

    return true;
}




bool PRMProblem::SetActiveTrajectory(RobotBasePtr robot, TrajectoryBasePtr active_traj, bool execute, const string &strsavetraj, boost::shared_ptr pout)
{
    if	( active_traj->GetPoints().size() == 0 )
    {
        return false;
    }

    active_traj->CalcTrajTiming(robot, pActiveTraj->GetInterpMethod(), true, true);

    bool execution_done = false;
    if ( execute ) {
        if ( active_traj->GetPoints().size() > 1 )
        {
            robot->SetActiveMotion(active_traj);
            execute = true;
        }
        // have to set anyway since calling script will orEnvWait!
        else if ( !!robot->GetController() )
        {
            TrajectoryBasePtr full_traj = RaveCreateTrajectory(GetEnv(), robot->GetDOF());
            robot->GetFullTrajectoryFromActive(full_traj, active_traj);

            if ( robot->GetController()->SetDesired(full_traj->GetPoints()[0].q) )
                execution_done = true;
        }
    }

    if ( strsavetraj.size() || !!pout ) {
        TrajectoryBasePtr full_traj = RaveCreateTrajectory(GetEnv(), robot->GetDOF());
        robot->GetFullTrajectoryFromActive(full_traj, active_traj);

        if ( strsavetraj.size() > 0 ) {
            ofstream f(strsavetraj.c_str());
            full_traj->Write(f, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        }

        if ( !!pout ) {
            full_traj->Write(*pout, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation|Trajectory::TO_OneLine);
        }
    }

    return execution_done;
}




bool PRMProblem::RunPRM(ostream &sout, istream &sinput)
{
    RAVELOG_WARN("Not implemented yet\n");
    return false;
}




bool PRMProblem::BuildRoadMap(ostream &sout, istream &sinput)
{
    RAVELOG_WARN("Not implemented yet\n");
    return false;
}





bool PRMProblem::RunQuery(ostream &sout, istream &sinput)
{
    RAVELOG_WARN("Not implemented yet\n");
    return false;
}




bool PRMProblem::TestPrmGraph(ostream &sout, istream &sinput)
{
    RAVELOG_WARN("Not implemented yet\n");
    return false;
}
