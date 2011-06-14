#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace OpenRAVE;

class openprm : public PlannerBase
{
public:
    openprm(EnvironmentBasePtr penv, std::istream& ss) : PlannerBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&openprm::MyCommand,this,_1,_2),
                        "This is an example command");
    }
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
    if( type == PT_Planner && interfacename == "openprm" ) {
        return InterfaceBasePtr(new openprm(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Planner].push_back("openprm");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

