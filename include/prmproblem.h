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

#ifndef PRMPROBLEM_H
#define PRMPROBLEM_H

#include <prmparams.h>

namespace openprm
{

class PRMProblem : public ProblemInstance
{
public:

    PRMProblem ( EnvironmentBasePtr penv );
    virtual ~PRMProblem();
    virtual void Destroy();
    virtual int main ( const std::string& args );
    virtual void SetActiveRobots ( const std::vector<RobotBasePtr>& robots );
    virtual bool SendCommand ( std::ostream& sout, std::istream& sinput );

protected:

    RobotBasePtr robot_ptr_;
    std::string planner_name_;
    std::string robot_name_;
    bool reuseplanner_;
    bool execute_;
    string traj_filename_;
    boost::shared_ptr<ostream> output_traj_stream_;



    bool GrabBody ( ostream& sout, istream& sinput );
    bool ReleaseAll ( ostream& sout, istream& sinput );
    bool SetActiveTrajectory ( RobotBasePtr robot, TrajectoryBasePtr active_traj, bool execute, const string& strsavetraj, boost::shared_ptr<ostream> pout);
    bool RunPRM ( ostream& sout, istream& sinput );
    bool BuildRoadMap ( ostream& sout, istream& sinput );
    bool RunQuery ( ostream& sout, istream& sinput );
    bool TestPrmGraph ( ostream& sout, istream& sinput );


    inline std::string getfilename_withseparator(istream& sinput, char separator)
    {
        string filename;
        if ( !getline(sinput, filename, separator) )
        {
            // just input directly
            RAVELOG_ERROR("filename not terminated with ';'\n");
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

    inline boost::shared_ptr<PRMProblem> shared_problem()
    {
        return boost::static_pointer_cast< PRMProblem >(shared_from_this());
    }

    inline boost::shared_ptr<PRMProblem const> shared_problem_const() const
    {
        return boost::static_pointer_cast< PRMProblem const >(shared_from_this());
    }

};

}


#endif // PRMPROBLEM_H
