#ifndef ROADMAP_INSPECTOR_H
#define ROADMAP_INSPECTOR_H

#include "../utils/prmgraph.h"
#include <boost/shared_ptr.hpp>

namespace openprm
{

class RMapInspector
{
public:
    RMapInspector(int m_samp);
    virtual ~RMapInspector();

    int generateSamples();
    bool createGraph();
    void renderGraph();

private:

    int max_samples;
    std::vector<std::vector<double> > vv_samples;
    boost::shared_ptr<PRMGraph> g_roadmap;
    SpaceSamplerBasePtr p_sampler;
};

//! ============================================================================================
RMapInspector::RMapInspector(int m_samp) : max_samples(m_samp)
{
    vv_samples.clear();

    //initialize roadmap
    g_roadmap.reset(new PRMGraph(7,20,50,5.0));

    p_sampler = RaveCreateSpaceSampler("halton");
    p_sampler->SetSpaceDOF(7);
}

RMapInspector::~RMapInspector() {}

//! ============================================================================================
int RMapInspector::generateSamples()
{
    std::vector<double> v_samp;
    int k = 0;

    while (k < max_samples)
    {
        p_sampler->SampleSequence(v_samp, 1);

        vv_samples.push_back(v_samp);
        v_samp.clear();
        k++;
    }

    return (int)vv_samples.size();
}

//! ============================================================================================
bool RMapInspector::createGraph()
{

}

//! ============================================================================================
void RMapInspector::renderGraph()
{

}


}

#endif // ROADMAP_INSPECTOR_H
