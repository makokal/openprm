/*
  Copyright (c) 2010 - 2011, Billy Okal (makokal@nyumbanilabs.com)
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  * Neither the name of the <organization> nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY <copyright holder> ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file spatialrep.h
 * \author Billy Okal
 * \brief Spatial Representation System
 */

#ifndef SPATIALREP_H
#define SPATIALREP_H

#include "oputils.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

#define DMAXNODES 100	/** default maximum nodes in the graph */
#define DNTHRESH 5		/** default neighborhood threshold */

namespace openprm
{

typedef dReal s_cost;

typedef boost::adjacency_list<
			boost::listS,
			boost::vecS,
			boost::undirectedS,
			boost::no_property,
			boost::property<boost::edge_weight_t, s_cost> >
		s_graph;

typedef s_graph::vertex_descriptor s_vertex;

typedef  std::pair<s_vertex, s_vertex> s_edge;

typedef boost::property_map<s_graph, boost::edge_weight_t>::type weight_map;

typedef s_graph::edge_descriptor e_descriptor;

typedef boost::graph_traits<s_graph>::out_edge_iterator out_edge_iterator;

typedef boost::adjacency_iterator_generator<s_graph, s_vertex, out_edge_iterator>::type adjacency_iterator;

struct found_goal {};

/* spatial node */
struct s_node
{
    s_node() {}
    s_node ( const config& conf, s_vertex v ) : nconfig ( conf ), vertex ( v ) {}
    config nconfig;
    s_vertex vertex;
};



/* A Star Goal visitor*/
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor ( Vertex goal )
    {
        m_goal = goal;
    }

    template <class Graph>
    void examine_vertex ( Vertex u, Graph& g )
    {
        if ( u == m_goal )
        {
            throw found_goal();
        }
    }

private:
    Vertex m_goal;
};

/* A Star Heuritic */
template <class Graph, class CostType, class NodeMap>
class distance_heuristic : public boost::astar_heuristic<Graph, CostType>
{
public:
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;

    distance_heuristic ( NodeMap n, Vertex goal )
    {
        m_node = n;
        m_goal = goal;
    }

    CostType operator() ( Vertex u )
    {
        return DistMetric::Eval ( m_node[m_goal].nconfig , m_node[u].nconfig );
    }

private:
    NodeMap m_node;
    Vertex m_goal;
};


/**
 * \class SpatialGraph
 * Spatial Representation mechanism mainly geared at Sample based planners
 * Implemented using the BOOST Graph Library
 */
class SpatialGraph
{
public:

    SpatialGraph()
    {
        max_nodes = DMAXNODES;
        neigh_thresh = DNTHRESH;
        node_list.clear();
        s_graph g;
        p_graph = g;
        w_map = boost::get ( boost::edge_weight, p_graph );
    }

    SpatialGraph ( int m_nodes, dReal n_thresh )
    {
        max_nodes = m_nodes;
        neigh_thresh = n_thresh;
        node_list.clear();
        s_graph g ( max_nodes );
        p_graph = g;
        w_map = boost::get ( boost::edge_weight, p_graph );
    }

    virtual ~SpatialGraph() {}


    s_vertex addNode ( const std::vector<dReal>& conf )
    {
        const s_vertex v = boost::add_vertex ( p_graph );

        s_node nn ( conf,v );
        node_list.push_back ( nn );

        return v;
    }

    bool addEdge ( s_vertex u, s_vertex v )
    {
        //TODO - control the no of edges allowed per node to ensure good coverage

        // ensure the edge does not exist or the vertices are not the same
        if ( edgeExists ( u, v ) || ( u == v ) )
        {
            RAVELOG_WARN ( "edge already exists\n" );
            return false;
        }

        s_node nu = getNode ( u );
        s_node nv = getNode ( v );

        dReal dist = DistMetric::Eval ( nu.nconfig, nv.nconfig );
		if (dist > neigh_thresh)
		{
			return false;
		}

        s_graph::edge_descriptor ed;
        bool inserted;

        boost::tie ( ed, inserted ) = boost::add_edge ( u, v, p_graph );
        if ( !inserted )
        {
            return false;
        }

        w_map[ed] = dist;

        return true;
    }

    bool findPathDK ( s_node f, s_node t, std::list<s_node>& s_path )
    {
        std::vector<s_vertex> p ( boost::num_vertices ( p_graph ) );
        std::vector<int> d ( boost::num_vertices ( p_graph ) );

        s_vertex start = f.vertex;

        boost::dijkstra_shortest_paths ( p_graph, start, boost::predecessor_map ( &p[0] ).distance_map ( &d[0] ) );

        s_vertex child = t.vertex;

        do
        {
            s_path.push_front ( getNode ( child ) );
            child = p[child];
        }
        while ( child != p[child] );

        /** then there is no path, same connected component */
        if ( child == f.vertex )
        {
            return false;
        }

        s_path.push_front ( f );

        return true;
    }

    bool findPathAS ( s_node f, s_node t, std::list<s_node>& s_path )
    {
        s_vertex start = f.vertex;
        s_vertex goal = t.vertex;

        // node map
        s_node node_map[node_list.size() ];

        int i = 0;
        FOREACH ( node, node_list )
        {
            node_map[i] = *node;
            i++;
        }

        std::vector<s_vertex> p ( boost::num_vertices ( p_graph ) );
        std::vector<int> d ( boost::num_vertices ( p_graph ) );


        RAVELOG_INFO ( "Running Astar\n" );
        try
        {
            boost::astar_search (
                p_graph,
                start,
                distance_heuristic<s_graph, s_cost, s_node*> ( node_map, goal ),
                boost::predecessor_map ( &p[0] ).distance_map ( &d[0] ).visitor ( astar_goal_visitor<s_vertex> ( goal ) )
            );
        }
        catch ( found_goal fg )
        {
            for ( s_vertex v = goal;; v = p[v] )
            {
                s_path.push_front ( getNode ( v ) );
                if ( p[v] == v )
                {
                    break;
                }
            }
        }

        return true;
    }

    /**
     * \def findNN
     * \brief Find the nearest neighbors of a node based on the neighborhood threshold
     */
    int findNN ( s_vertex n, std::list<s_node>& s_neighbors )
    {
        if ( node_list.size() <=1 )
        {
            return 0;
        }

        if ( n > node_list.size() )
        {
            RAVELOG_ERROR ( "vertex non existent vertexid = [%d]  num_vert[%d]\n", n, boost::num_vertices ( p_graph ) );
            return 0;
        }

        s_node nn = getNode ( n );
        s_neighbors.clear();
        for ( std::vector<s_node>::iterator it = node_list.begin(); it != node_list.end(); it++ )
        {
            //avoid vertices already connected with
            if ( edgeExists ( n, ( *it ).vertex ) )
            {
                continue;
            }

            if ( DistMetric::Eval ( nn.nconfig, ( *it ).nconfig ) <= neigh_thresh )
            {
                s_neighbors.push_back ( ( *it ) );
            }
        }
        //s_neighbors.sort(); NOTE - proving unnecesarily too expensive

        return ( int ) s_neighbors.size();
    }

    void printGraph ( std::string filename )
    {
        boost::dynamic_properties dp;
        dp.property<> ( "edge_weight", w_map );
        dp.property<> ( "node_id", boost::get ( boost::vertex_index, p_graph ) );

        std::ofstream ofile;
        ofile.open ( filename.c_str(), std::ios::out );
        boost::write_graphviz<> ( ofile, p_graph, dp );
        ofile.close();
    }

    inline int getNodes()
    {
        return node_list.size();
    }


    inline s_node getNode ( s_vertex v )
    {
        s_node n;
        for ( std::vector<s_node>::iterator it = node_list.begin(); it != node_list.end(); it++ )
        {
            if ( ( *it ).vertex == v )
            {
                n = ( *it );
            }
        }
        return n;
    }

    inline bool edgeExists ( s_vertex u, s_vertex v )
    {
        if ( u == v )
        {
            return true;
        }

        adjacency_iterator it, it_end;
        boost::tie ( it, it_end ) = boost::adjacent_vertices ( v, p_graph );

        while ( it != it_end )
        {
            if ( u == ( *it ) )
            {
                return true;
            }
            ++it;
        }
        return false;
    }

protected:

    unsigned int max_nodes;
    dReal neigh_thresh;
    std::vector<s_node> node_list;
    s_graph p_graph;
    weight_map w_map;


};

}

#endif