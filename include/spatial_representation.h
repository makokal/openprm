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


#ifndef SPATIAL_REPRESENTATION_H
#define SPATIAL_REPRESENTATION_H

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/lexical_cast.hpp>

#include <openrave/planningutils.h>

#include <prm_utils.h>


namespace openprm
{

/// A node in the spatial structure
struct Vertex
{
    std::vector<dReal> config;
};

/// An edge in the spatial structure
struct Edge
{
    dReal length;
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, Vertex, Edge> SpatialGraph;
typedef boost::graph_traits<SpatialGraph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<SpatialGraph>::edge_descriptor edge_t;





class SpatialStructure
{
public:
    SpatialStructure() :
        max_nodes_(100), no_nodes_(0), max_edges_(1000), no_edges_(0), dimension_(7)
    {
        graph_.clear();
    }

    SpatialStructure(int mnodes, int medges, int dim) :
        max_nodes_(mnodes), no_nodes_(0), max_edges_(medges), no_edges_(0), dimension_(dim)
    {
        graph_.clear();
    }


    /// add a vertex to the graph
    vertex_t addVertex(std::vector<dReal> config)
    {
        /// check that we dont exceed max nodes
        if ( no_nodes_ == max_nodes_ )
        {
            RAVELOG_WARN("SpatialStructure::addVertex - Max nodes reached, ignoring further nodes \n");
            return;
        }

        /// check that the dimension of the configuration matches graph dimension
        if ( config.size() != dimension_ )
        {
            RAVELOG_WARN("SpatialStructure::addVertex - dimension of configuration does not match graph dimension \n");
            return;
        }

        vertex_t v = boost::add_vertex(graph_);
        graph_[v].config = config;

        no_nodes_++;

        return v;
    }


    /// add an edge to the graph
    bool addEdge(vertex_t u, vertex_t v, dReal length)
    {
        /// check if edge already exists
        if ( boost::edge(u,v, graph_).second )
        {
            RAVELOG_WARN("SpatialStructure::addEdge - edge already exists \n");
            return false;
        }

        /// check that we dont exceed max edges
        if ( no_edges_ == max_edges_ )
        {
            RAVELOG_WARN("SpatialStructure::addEdge - Max edges reached, ignoring further edges \n");
            return false;
        }

        edge_t e, bool added;
        boost::tie(e, added) = boost::add_edge(u,v);

        if (added)
        {
            graph_[e].length = length;
            no_edges_++;

            return true;
        }

        return false;
    }

protected:
    int max_nodes_, no_nodes_;
    int max_edges_, no_edges_;
    int dimension_;

    SpatialGraph graph_;
};




}

#endif // SPATIAL_REPRESENTATION_H
