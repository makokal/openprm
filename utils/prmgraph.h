#ifndef PRMGRAPH_H
#define PRMGRAPH_H

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/lexical_cast.hpp>

#include "../plugindefs.h"

namespace openprm
{

//! ===============================================================================================================

//! Robot configuration in CSpace, and node id
typedef std::vector<double> config_t;

//! graph concepts

// Vertex/Node properties
typedef boost::property<boost::vertex_name_t, std::string> node_property;

// Edge properties
//  - Edge is defined as {A,B,l} where A and B are the vertices/nodes, l = label(distance btw nodes)
typedef boost::property<boost::edge_weight_t, double> edge_property;

// The graph structure
typedef boost::adjacency_list<
	boost::listS, 
	boost::vecS, 
	boost::undirectedS,
	node_property, 
	edge_property> 
	graph_t;


// A spatial node
typedef struct spatial_node
{
	spatial_node(graph_t::vertex_descriptor id, config_t conf) : node_id(id), config(conf) {}
    graph_t::vertex_descriptor node_id;
    config_t config;
} node_t;

// A spatial Edge
typedef std::pair<node_t, node_t> edge_t;

//! ===============================================================================================================
//! Metrics and Utils
class EMetric
{
public:
    static double computeLength ( config_t q1, config_t q2 )
    {
        if ( q1.size() != q2.size() )
        {
            RAVELOG_ERROR ( "Invalid Configuration dimensions [%d] and [%d], Hint: must be equal\n",q1.size(), q2.size() );
        }

        double d = 0;
        int i = 0;

        while ( i < ( int ) q1.size() )
        {
            double t = abs ( q1[i] - q2[i] );
            d += t * t;
            i++;
        }

        return sqrt ( d );
    }
};

//! ===============================================================================================================
//! \class PRMGraph
//! \brief Represents a spatial graph in specified dimension
class PRMGraph
{
public:
    PRMGraph ( unsigned int dim, unsigned max_edges, unsigned int max_node, double max_elen );
    virtual ~PRMGraph();

	node_t addNode( const config_t conf );
	bool addEdge( node_t e_start, node_t e_end );
	int findNN( node_t n, std::list<node_t>& l_neighbors );
	
protected:
	
	typedef std::pair<node_t, double> dnode_t; 
	
	inline bool edgeExists( node_t e_start, node_t e_end );
	inline bool compareNodes( dnode_t x, dnode_t y);
	
	
	graph_t g_roadmap;
	std::list<node_t> l_nodes;
	std::vector<edge_t> v_edges;
	
    unsigned int i_dimension;
    unsigned int i_max_edge_branch;
    unsigned int i_max_nodes;
    double f_max_edge_length;
	
	boost::property_map<graph_t, boost::edge_weight_t>::type edge_lengths;
	boost::property_map<graph_t, boost::vertex_name_t>::type node_names;
};

//! ====================================================================================================

PRMGraph::PRMGraph ( unsigned int dim, unsigned int max_edges, unsigned int max_node, double max_elen )
	: i_dimension(dim), i_max_edge_branch(max_edges), i_max_nodes(max_node), f_max_edge_length(max_elen)
{
	//! \TODO add checks for non-zero vals
	g_roadmap.clear();
	l_nodes.clear();
	v_edges.clear();
}
PRMGraph::~PRMGraph() {}

//! ====================================================================================================

node_t PRMGraph::addNode ( const openprm::config_t conf )
{
	if (conf.empty())
	{
		RAVELOG_ERRORA("Trying to add a node with empty configuration");
		return NULL;
	}
	else
	{
		graph_t::vertex_descriptor n_id = boost::add_vertex ( g_roadmap );
		
		// add node property
		node_names[n_id] = boost::lexical_cast<std::string>(n_id);
		
        node_t node ( n_id, conf );
        l_nodes.push_back ( node );

        return node;
	}
}

//! ====================================================================================================

bool PRMGraph::addEdge ( node_t e_start, node_t e_end )
{
    // ensure the edge does not exist or the vertices are not the same
    if ( edgeExists ( e_start, e_end ) || ( e_start == e_end ) )
    {
        RAVELOG_WARN ( "edge already exists\n" );
        return false;
    }
    
    // check for max edges per node(to control how dense the roadmap is)
	if (/* \TODO check for max edges*/ )
	{
	}
	
	double elen = EMetric::computeLength(e_start.config, e_end.config);
	// compute edge length
	if ( elen > f_max_edge_length)
	{
		RAVELOG_DEBUGA("Nodes are too far apart, edge cannot be added\n");
		return false;
	}
	
	graph_t::edge_descriptor edged;
	bool b_success;
	
	boost::tie(edged, b_success) = boost::add_edge(e_start, e_end, g_roadmap);
	if (!b_success)
	{
		RAVELOG_DEBUGA("Boost error in adding edge\n");
		return false;
	}
	
	edge_lengths[edged] = elen;
	v_edges.push_back(std::make_pair<node_t,node_t>(e_start,e_end));
	
	return true;
}


//! ====================================================================================================
int PRMGraph::findNN ( node_t n, list< node_t >& l_neighbors )
{
	if (l_nodes.size() <= 1)
	{
		return 0;
	}
	
	std::list<dnode_t> l_dnodes;
	l_dnodes.clear();
	l_neighbors.clear();
	for ( std::vector<spatial_node>::iterator it = l_nodes.begin(); it != l_nodes.end(); it++ )
    {
		// skip all nodes not connected with
		if (!edgeExists(n, (*it)) )
		{
			continue;
		}
		double dist = EMetric::computeLength(n.config, (*it).config);
		l_dnodes.push_back(std::make_pair<node_t,double>((*it), dist));
	}
	
	// sort the neighbors by distance to current node
	l_dnodes.sort(compareNodes);
	for ( std::list<dnode_t>::iterator k = l_dnodes.begin(); k != l_dnodes.end(); k++)
	{
		l_neighbors.push_front((*k).first);
	}
	l_dnodes.clear();
	
	return (int)l_neighbors.size();
}

//! ====================================================================================================

bool PRMGraph::compareNodes(dnode_t x, dnode_t y)
{
	if (x.second > y.second)
	{
		return true;
	}
	return false;
}

//! ====================================================================================================

bool PRMGraph::edgeExists ( node_t e_start, node_t e_end )
{
	if (e_start == e_end)
	{
		return true;
	}
	
// 	boost::adjacency_iterator<graph_t, graph_t::vertex_descrip>
}



}

#endif // PRMGRAPH_H
