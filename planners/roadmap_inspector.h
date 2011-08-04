#ifndef ROADMAP_INSPECTOR_H
#define ROADMAP_INSPECTOR_H

#include "../utils/prmgraph.h"
#include "samplerbase.h"
#include <boost/shared_ptr.hpp>

namespace openprm
{

class RMapInspector
{
public:
    RMapInspector(RobotBasePtr probot, int m_samp);
    virtual ~RMapInspector();

    int generateSamples();
    bool createGraph();
    void renderGraph();

private:

    RobotBasePtr p_robot;
    int max_samples;
    std::vector<std::vector<double> > vv_samples;
    boost::shared_ptr<PRMGraph> g_roadmap;
//    SpaceSamplerBasePtr p_sampler;
    boost::shared_ptr<RandomSampler> p_sampler;

    GraphHandlePtr p_graph;
};

//! ============================================================================================
RMapInspector::RMapInspector(RobotBasePtr probot, int m_samp) : p_robot(probot), max_samples(m_samp)
{
    vv_samples.clear();

    //initialize roadmap
    g_roadmap.reset(new PRMGraph(p_robot->GetActiveDOF(),20,50,5.0));

//    p_sampler = RaveCreateSpaceSampler("halton");
//    p_sampler->SetSpaceDOF(7);
    p_sampler.reset(new RandomSampler(p_robot));
}

RMapInspector::~RMapInspector() {}

//! ============================================================================================
int RMapInspector::generateSamples()
{
    std::vector<double> v_samp;
    int k = 0;

    while (k < max_samples)
    {
//        p_sampler->SampleSequence(v_samp, 1);
        if (!p_sampler->GenSingleSample(v_samp))
        {
            RAVELOG_WARN("Error in sampling\n");
            continue;
        }

        vv_samples.push_back(v_samp);
        v_samp.clear();
        k++;
    }

    return (int)vv_samples.size();
}

//! ============================================================================================
bool RMapInspector::createGraph()
{
    RAVELOG_INFO("Creating the Roadmap graph\n");

    for (std::vector<std::vector<double> >::iterator it = vv_samples.begin(); it != vv_samples.end(); it++)
    {
        node_t nd = g_roadmap->addNode((*it));
        std::list<node_t> neighbors;

        if (g_roadmap->findNN(nd, neighbors) == 0)
        {
            continue;
        }

        // add the neighbor edges;
        for (std::list<node_t>::iterator lt = neighbors.begin(); lt != neighbors.end(); lt++)
        {
            g_roadmap->addEdge(nd, (*lt));
        }
    }


    RAVELOG_INFO("Roadmap created\n");
    return true;
}

//! ============================================================================================
void RMapInspector::renderGraph()
{
}


}

#endif // ROADMAP_INSPECTOR_H
