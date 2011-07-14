/*
    Copyright (c) 2010-2011, Billy Okal sudo@makokal.com
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the author nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY the author ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL the author BE LIABLE FOR ANY
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
 * \brief Spatial Representation System used in openprm
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

//! NOTE testing
#define DMAXNODES 100	/** default maximum nodes in the graph */
#define DNTHRESH 5		/** default neighborhood threshold */

namespace openprm
{

typedef dReal graph_cost;
typedef boost::adjacency_list<
boost::listS,
boost::vecS,
boost::undirectedS,
boost::no_property,
boost::property<boost::edge_weight_t, graph_cost> >
spatial_graph;
typedef spatial_graph::vertex_descriptor spatial_vertex;
typedef std::pair<spatial_vertex, spatial_vertex> spatial_edge;
typedef boost::property_map<spatial_graph, boost::edge_weight_t>::type weight_map;
typedef spatial_graph::edge_descriptor e_descriptor;
typedef boost::graph_traits<spatial_graph>::out_edge_iterator out_edge_iterator;
typedef boost::adjacency_iterator_generator<spatial_graph, spatial_vertex, out_edge_iterator>::type adjacency_iterator;

//! for spatial tree used in SBL planner
enum ExtendType 
{
    ET_Failed=0,
    ET_Sucess=1,
    ET_Connected=2
};

struct found_goal {};

//! Spatial Graph Node 
struct spatial_node
{
    spatial_node() {}
    spatial_node ( const v_config& conf, spatial_vertex v ) : nconfig ( conf ), vertex ( v ) {}
    v_config nconfig;
    spatial_vertex vertex;
};

//! Spatial Tree Node
struct tree_node
{
    tree_node(int parent, const vector<dReal>& q) : parent(parent), q(q) {}
    int parent;
    v_config q;
};

/** ======================================================================================= */

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

/** ======================================================================================= */

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
        return EDMetric::Eval ( m_node[m_goal].nconfig , m_node[u].nconfig );
    }

private:
    NodeMap m_node;
    Vertex m_goal;
};


/** ======================================================================================= */

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
        spatial_graph g;
        p_graph = g;
        w_map = boost::get ( boost::edge_weight, p_graph );
    }

    SpatialGraph ( int m_nodes, dReal n_thresh )
    {
        max_nodes = m_nodes;
        neigh_thresh = n_thresh;
        node_list.clear();
        spatial_graph g ( max_nodes );
        p_graph = g;
        w_map = boost::get ( boost::edge_weight, p_graph );
    }

    virtual ~SpatialGraph() {}


    spatial_vertex addNode ( const std::vector<dReal>& conf )
    {
        const spatial_vertex v = boost::add_vertex ( p_graph );

        spatial_node nn ( conf,v );
        node_list.push_back ( nn );

        return v;
    }

    bool addEdge ( spatial_vertex u, spatial_vertex v )
    {
        //TODO - control the no of edges allowed per node to ensure good coverage

        // ensure the edge does not exist or the vertices are not the same
        if ( edgeExists ( u, v ) || ( u == v ) )
        {
            RAVELOG_WARN ( "edge already exists\n" );
            return false;
        }

        spatial_node nu = getNode ( u );
        spatial_node nv = getNode ( v );

        dReal dist = EDMetric::Eval ( nu.nconfig, nv.nconfig );
        if (dist > neigh_thresh)
        {
            return false;
        }

        spatial_graph::edge_descriptor ed;
        bool inserted;

        boost::tie ( ed, inserted ) = boost::add_edge ( u, v, p_graph );
        if ( !inserted )
        {
            return false;
        }

        w_map[ed] = dist;

        return true;
    }

    bool findPathDK ( spatial_node f, spatial_node t, std::list<spatial_node>& s_path )
    {
        std::vector<spatial_vertex> p ( boost::num_vertices ( p_graph ) );
        std::vector<int> d ( boost::num_vertices ( p_graph ) );

        spatial_vertex start = f.vertex;

        RAVELOG_INFO ( "Running Dijikstra\n" );

        boost::dijkstra_shortest_paths ( p_graph, start, boost::predecessor_map ( &p[0] ).distance_map ( &d[0] ) );

        spatial_vertex child = t.vertex;

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

    bool findPathAS ( spatial_node f, spatial_node t, std::list<spatial_node>& s_path )
    {
        spatial_vertex start = f.vertex;
        spatial_vertex goal = t.vertex;

        // node map
        spatial_node node_map[node_list.size() ];

        int i = 0;
        FOREACH ( node, node_list )
        {
            node_map[i] = *node;
            i++;
        }

        std::vector<spatial_vertex> p ( boost::num_vertices ( p_graph ) );
        std::vector<int> d ( boost::num_vertices ( p_graph ) );


        RAVELOG_INFO ( "Running Astar\n" );
        try
        {
            boost::astar_search (
                    p_graph,
                    start,
                    distance_heuristic<spatial_graph, graph_cost, spatial_node*> ( node_map, goal ),
                    boost::predecessor_map ( &p[0] ).distance_map ( &d[0] ).visitor ( astar_goal_visitor<spatial_vertex> ( goal ) )
                    );
        }
        catch ( found_goal fg )
        {
            RAVELOG_INFO("Found goal\n");
            for ( spatial_vertex v = goal;; v = p[v] )
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
    int findNN ( spatial_vertex n, std::list<spatial_node>& s_neighbors )
    {
        if ( node_list.size() <=1 )
        {
            return 0;
        }

//        if ( n > (node_list.size() + 1) )
//        {
//            RAVELOG_ERROR ( "vertex non existent vertexid = [%d]  num_vert[%d]\n", n, boost::num_vertices ( p_graph ) );
//            return 0;
//        }

        spatial_node nn = getNode ( n );
        s_neighbors.clear();
        for ( std::vector<spatial_node>::iterator it = node_list.begin(); it != node_list.end(); it++ )
        {
            //avoid vertices already connected with
            if ( !edgeExists ( n, ( *it ).vertex ) )
            {
                continue;
            }

            if ( EDMetric::Eval ( nn.nconfig, ( *it ).nconfig ) <= neigh_thresh )
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


    inline spatial_node getNode ( spatial_vertex v )
    {
        spatial_node n;
        for ( std::vector<spatial_node>::iterator it = node_list.begin(); it != node_list.end(); it++ )
        {
            if ( ( *it ).vertex == v )
            {
                n = ( *it );
            }
        }
        return n;
    }

    inline bool edgeExists ( spatial_vertex u, spatial_vertex v )
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
    std::vector<spatial_node> node_list;
    spatial_graph p_graph;
    weight_map w_map;
};


/** ======================================================================================= */
struct SimpleNode
{
SimpleNode(int parent, const vector<dReal>& q) : parent(parent), q(q) {}
    int parent;
    vector<dReal> q; // the configuration immediately follows the struct
};

class SpatialTreeBase
{
 public:
    virtual int AddNode(int parent, const vector<dReal>& config) = 0;
    virtual int GetNN(const vector<dReal>& q) = 0;
    virtual const vector<dReal>& GetConfig(int inode) = 0;
    virtual ExtendType Extend(const vector<dReal>& pTargetConfig, int& lastindex, bool bOneStep=false) = 0;
    virtual int GetDOF() = 0;
};


template <typename Planner, typename Node>
class SpatialTree : public SpatialTreeBase
{
 public:
    SpatialTree() {
        _fStepLength = 0.04f;
        _dof = 0;
        _cols = 0;
        _fBestDist = 0;
        _nodes.reserve(5000);
    }

    ~SpatialTree(){}

    virtual void Reset(boost::weak_ptr<Planner> planner, int dof=0)
    {
        _planner = planner;

        typename vector<Node*>::iterator it;
        FORIT(it, _nodes)
            delete *it;
        _nodes.resize(0);

        if( dof > 0 ) {
            _vNewConfig.resize(dof);
            _dof = dof;
        }
    }

    virtual int AddNode(int parent, const vector<dReal>& config)
    {
        _nodes.push_back(new Node(parent,config));
        return (int)_nodes.size()-1;
    }

    ///< return the nearest neighbor
    virtual int GetNN(const vector<dReal>& q)
    {
        if( _nodes.size() == 0 ) {
            return -1;
        }
        int ibest = -1;
        dReal fbest = 0;
        FOREACH(itnode, _nodes) {
            dReal f = _distmetricfn(q, (*itnode)->q);
            if( ibest < 0 || f < fbest ) {
                ibest = (int)(itnode-_nodes.begin());
                fbest = f;
            }
        }

        _fBestDist = fbest;
        return ibest;
    }

    /// extends toward pNewConfig
    /// \return true if extension reached pNewConfig
    virtual ExtendType Extend(const vector<dReal>& pTargetConfig, int& lastindex, bool bOneStep=false)
    {
        // get the nearest neighbor
        lastindex = GetNN(pTargetConfig);
        if( lastindex < 0 ) {
            return ET_Failed;
        }
        Node* pnode = _nodes.at(lastindex);
        bool bHasAdded = false;
        boost::shared_ptr<Planner> planner(_planner);
        PlannerBase::PlannerParametersConstPtr params = planner->GetParameters();
        // extend
        while(1) {
            dReal fdist = _distmetricfn(pTargetConfig,pnode->q);
            if( fdist > _fStepLength ) {
                fdist = _fStepLength / fdist;
            }
            else if( fdist <= dReal(0.1) * _fStepLength ) { // return connect if the distance is very close
                return ET_Connected;
            }

            _vNewConfig = pTargetConfig;
            params->_diffstatefn(_vNewConfig,pnode->q);
            for(int i = 0; i < _dof; ++i)
                _vNewConfig[i] = pnode->q[i] + _vNewConfig[i]*fdist;

            // project to constraints
//            if( !!params->_constraintfn ) {
                params->_setstatefn(_vNewConfig);
//                if( !params->_constraintfn(pnode->q, _vNewConfig, 0) ) {
                    if(bHasAdded) {
                        return ET_Sucess;
                    }
//                    return ET_Failed;
//                }

                // it could be the case that the node didn't move anywhere, in which case we would go into an infinite loop
                if( _distmetricfn(pnode->q, _vNewConfig) <= dReal(0.01)*_fStepLength ) {
                    if(bHasAdded) {
                        return ET_Sucess;
                    }
                    return ET_Failed;
                }
//            }

            if( ICollision::CheckCollision(params,planner->GetRobot(),pnode->q, _vNewConfig, OPEN_START) ) {
                if(bHasAdded) {
                    return ET_Sucess;
                }
                return ET_Failed;
            }

            lastindex = AddNode(lastindex, _vNewConfig);
            pnode = _nodes[lastindex];
            bHasAdded = true;
            if( bOneStep ) {
                return ET_Connected;
            }
        }

        return ET_Failed;
    }

    virtual const vector<dReal>& GetConfig(int inode) { return _nodes.at(inode)->q; }
    virtual int GetDOF() { return _dof; }

    vector<Node*> _nodes;
    boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;

    dReal _fBestDist; ///< valid after a call to GetNN
    dReal _fStepLength;

 private:
    int _cols;	// number of collision checks
    vector<dReal> _vNewConfig;
    boost::weak_ptr<Planner> _planner;
    int _dof;
};


//template <typename Planner, typename Node>
//class SpatialTree
//{
//public:
//    SpatialTree()
//    {
//        _fStepLength = 0.04f;
//        _dof = 0;
//        _fBestDist = 0;
//        _nodes.reserve(5000);
//    }
//
//    ~SpatialTree(){}
//
//    //! differentiate from reset provided by boost shared ptr
//    void cReset(boost::weak_ptr<Planner> planner, int dof=0)
//    {
//        _planner = planner;
//
//        typename vector<Node*>::iterator it;
//        FORIT(it, _nodes)
//                delete *it;
//        _nodes.resize(0);
//
//        if( dof > 0 ) {
//            _vNewConfig.resize(dof);
//            _dof = dof;
//        }
//    }
//
//    virtual int AddNode(int parent, const vector<dReal>& config)
//    {
//        _nodes.push_back(new Node(parent,config));
//        return (int)_nodes.size()-1;
//    }
//
//    ///< return the nearest neighbor
//    virtual int GetNN(const vector<dReal>& q)
//    {
//        if( _nodes.size() == 0 ) {
//            return -1;
//        }
//        int ibest = -1;
//        dReal fbest = 0;
//        FOREACH(itnode, _nodes) {
//            dReal f = _distmetricfn(q, (*itnode)->q);
//            if( ibest < 0 || f < fbest ) {
//                ibest = (int)(itnode-_nodes.begin());
//                fbest = f;
//            }
//        }
//
//        _fBestDist = fbest;
//        return ibest;
//    }
//
//    virtual ExtendType Extend(const vector<dReal>& pTargetConfig, int& lastindex, bool bOneStep=false)
//    {
//        // get the nearest neighbor
//        lastindex = GetNN(pTargetConfig);
//        if( lastindex < 0 ) {
//            return ET_Failed;
//        }
//        Node* pnode = _nodes.at(lastindex);
//        bool bHasAdded = false;
//        boost::shared_ptr<Planner> planner(_planner);
//        PlannerBase::PlannerParametersConstPtr params = planner->GetParameters();
//        // extend
//        while(1) {
//            dReal fdist = _distmetricfn(pTargetConfig,pnode->q);
//            if( fdist > _fStepLength ) {
//                fdist = _fStepLength / fdist;
//            }
//            else if( fdist <= dReal(0.1) * _fStepLength ) { // return connect if the distance is very close
//                return ET_Connected;
//            }
//
//            _vNewConfig = pTargetConfig;
//            params->_diffstatefn(_vNewConfig,pnode->q);
//            for(int i = 0; i < _dof; ++i)
//                _vNewConfig[i] = pnode->q[i] + _vNewConfig[i]*fdist;
//
//            // project to constraints
////            if( !!params->_constraintfn ) {
//                params->_setstatefn(_vNewConfig);
////                if( !params->_constraintfn(pnode->q, _vNewConfig, 0) ) {
////                    if(bHasAdded) {
////                        return ET_Sucess;
////                    }
////                    return ET_Failed;
////                }
//
//                // it could be the case that the node didn't move anywhere, in which case we would go into an infinite loop
//                if( _distmetricfn(pnode->q, _vNewConfig) <= dReal(0.01)*_fStepLength ) {
//                    if(bHasAdded) {
//                        return ET_Sucess;
//                    }
//                    return ET_Failed;
//                }
////            }
//
//            if( ICollision::CheckCollision(params,planner->GetRobot(),pnode->q, _vNewConfig, OPEN_START) ) {
//                if(bHasAdded) {
//                    return ET_Sucess;
//                }
//                return ET_Failed;
//            }
//
//            lastindex = AddNode(lastindex, _vNewConfig);
//            pnode = _nodes[lastindex];
//            bHasAdded = true;
//            if( bOneStep ) {
//                return ET_Connected;
//            }
//        }
//
//        return ET_Failed;
//    }
//
//    virtual const vector<dReal>& GetConfig(int inode)
//    {
//        return _nodes.at(inode)->q;
//
//    }
//
//    virtual int GetDOF()
//    {
//        return _dof;
//    }
//
//    vector<Node*> _nodes;
//    boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;
//    dReal _fBestDist; ///< valid after a call to GetNN
//    dReal _fStepLength;
//
//private:
//    vector<dReal> _vNewConfig;
//    boost::weak_ptr<Planner> _planner;
//    int _dof;
//};

}

#endif
