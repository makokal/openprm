#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;

class PRMPlanner : public PlannerBase
{
public:
    PRMPlanner(EnvironmentBasePtr penv, std::istream& ss) : PlannerBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&PRMPlanner::MyCommand,this,_1,_2),
                        "This is an example command");
    }
    virtual ~PRMPlanner() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Planner && interfacename == "prmplanner" ) {
        return InterfaceBasePtr(new PRMPlanner(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Planner].push_back("PRMPlanner");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

