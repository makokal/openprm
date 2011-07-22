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

typedef boost::graph_traits<graph_t>::out_edge_iterator out_edge_iter_t;

typedef boost::adjacency_iterator_generator<graph_t, graph_t::vertex_descriptor, out_edge_iter_t>::type adjacency_iter_t;

typedef graph_t::vertex_descriptor vertex_t;

// A spatial node
typedef struct spatial_node
{
    spatial_node();
	spatial_node(vertex_t id, config_t conf) : node_id(id), config(conf) {}
    vertex_t node_id;
    config_t config;
} node_t;

// A spatial Edge
typedef std::pair<node_t, node_t> edge_t;

struct found_goal {};

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

template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor ( Vertex goal ) : m_goal(goal) {}

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

//! ===============================================================================================================
template <class MutableGraph, class CostType, class LocMap>
class distance_heuristic : public boost::astar_heuristic<MutableGraph, CostType> {
public:
	typedef typename boost::graph_traits<MutableGraph>::vertex_descriptor Vertex;
	
	distance_heuristic(LocMap l, Vertex goal) : m_location(l), m_goal(goal) {}
	
	CostType operator()(Vertex u) 
	{
		return (EMetric::computeLength(m_location[m_goal].config, m_location[u].config));
	}
private:
	LocMap m_location;
	Vertex m_goal;
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
	bool findPathAstar( node_t n_start, node_t n_goal, std::list<node_t>& l_path );
	
protected:
	
	typedef std::pair<node_t, double> dnode_t; 
	
	inline bool edgeExists( node_t e_start, node_t e_end );
	inline bool compareNodes( dnode_t x, dnode_t y);
//	inline node_t nodeAtVertex(vertex_t v);
	
	
	graph_t g_roadmap;
	std::list<node_t> l_nodes;
	std::vector<edge_t> v_edges;
	spatial_node *node_map;
	
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
	g_roadmap.clear();
	l_nodes.clear();
	v_edges.clear();
	
	spatial_node nmap[max_node];
	node_map = nmap;
	
    edge_lengths = boost::get(boost::edge_weight, g_roadmap);
	node_names = boost::get(boost::vertex_name, g_roadmap);
}
PRMGraph::~PRMGraph() {}

//! ====================================================================================================

node_t PRMGraph::addNode ( const openprm::config_t conf )
{
	if (conf.empty())
	{
		RAVELOG_ERRORA("Trying to add a node with empty configuration");
//		return NULL;
        return spatial_node();
	}
	else
	{
		vertex_t n_id = boost::add_vertex ( g_roadmap );
		
		// add node property
		node_names[n_id] = boost::lexical_cast<std::string>(n_id);
		
        node_t node ( n_id, conf );
        l_nodes.push_back ( node );
		
		//!\todo cleanup
		node_map[n_id] = node;

        return node;
	}
}

//! ====================================================================================================

bool PRMGraph::addEdge ( node_t e_start, node_t e_end )
{
    // ensure the edge does not exist or the vertices are not the same
    if ( edgeExists ( e_start, e_end ) || ( e_start.node_id == e_end.node_id ) )
    {
        RAVELOG_WARN ( "edge already exists\n" );
        return false;
    }
    
    // check for max edges per node(to control how dense the roadmap is)
//	if (/* \TODO check for max edges*/ )
//	{
//	}
	
	double elen = EMetric::computeLength(e_start.config, e_end.config);
	// compute edge length
	if ( elen > f_max_edge_length)
	{
		RAVELOG_DEBUGA("Nodes are too far apart, edge cannot be added\n");
		return false;
	}
	
	graph_t::edge_descriptor edged;
	bool b_success;
	
    boost::tie(edged, b_success) = boost::add_edge(e_start.node_id, e_end.node_id, g_roadmap);
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
    for ( std::list<spatial_node>::iterator it = l_nodes.begin(); it != l_nodes.end(); it++ )
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
    if (e_start.node_id == e_end.node_id)
	{
		return true;
	}
	
	adjacency_iter_t it_beg, it_end;
	
    boost::tie ( it_beg, it_end ) = boost::adjacent_vertices ( e_end.node_id, g_roadmap );
	
	while ( it_beg != it_end )
    {
		//! \todo add a heuristic to allow arbitrary closeness
        if ( e_start.node_id == ( *it_beg ) )
        {
            return true;
        }
        ++it_beg;
    }
    return false;
}

//! ====================================================================================================

bool PRMGraph::findPathAstar(node_t n_start, node_t n_goal, list< node_t >& l_path)
{
	std::vector<vertex_t> p(boost::num_vertices(g_roadmap));
	std::vector<double> d(boost::num_vertices(g_roadmap));
	
	try
	{
		boost::astar_search(g_roadmap,
							n_start,
							distance_heuristic<graph_t, double, spatial_node*>(node_map, g_roadmap),
							boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(astar_goal_visitor<vertex_t>(n_goal))
							);
	}
	catch ( found_goal )
	{
		RAVELOG_INFO("Found a path\n");
		for ( vertex_t v = n_goal;; v = p[v] )
        {
            l_path.push_front ( node_map[v] );
            if ( p[v] == v )
            {
                return true;
// 				break;
            }
        }
    }
	
	return false;
}


}

#endif // PRMGRAPH_H
