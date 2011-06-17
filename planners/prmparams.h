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
 * \file prmparams.h
 * \author Billy Okal
 * PRM Parameters
 */

#ifndef PRMPARAMS_H
#define PRMPARAMS_H

#include "../plugindefs.h"

namespace openprm
{

class PRMParams : public PlannerBase::PlannerParameters
{
public:
    PRMParams();

    bool b_prune_roadmap;
    bool b_smooth_path;
    unsigned int i_ntries;
    unsigned int i_nnodes;
    unsigned int i_nedges;
    dReal f_neigh_thresh;

protected:

    bool b_processing;
    virtual bool serialize ( std::ostream& O ) const;
    BaseXMLReader::ProcessElement startElement ( const std::string& name, const std::list<std::pair<std::string,std::string> >& atts );
    virtual bool endElement ( const std::string& name );
};


/** ======================================================================================= */

PRMParams::PRMParams () :
    b_prune_roadmap ( false ),
    b_smooth_path ( false ),
    i_ntries ( 10 ),
    i_nnodes ( 100 ),
    i_nedges ( 10 ),
    f_neigh_thresh ( 4.5 ),
    b_processing ( false )
{

    _vXMLParameters.push_back ( "bpruneroadmap" );
    _vXMLParameters.push_back ( "bsmoothpath" );
    _vXMLParameters.push_back ( "ntries" );
    _vXMLParameters.push_back ( "mnodes" );
    _vXMLParameters.push_back ( "medges" );
    _vXMLParameters.push_back ( "neighthresh" );
}


/** ======================================================================================= */

bool PRMParams::serialize ( std::ostream& O ) const
{

    if ( !PlannerParameters::serialize ( O ) )
    {
        return false;
    }

    O << "<pruneroadmap>" << b_prune_roadmap << "</pruneroadmap>" << endl;

    O << "<smoothpath>" << b_smooth_path << "</smoothpath>" << endl;

    O << "<ntries>" << i_ntries << "</ntries>" << endl;

    O << "<mnodes>" << i_nnodes << "</mnodes>" << endl;

    O << "<medges>" << i_nedges << "</medges>" << endl;

    O << "<neighthresh>" << f_neigh_thresh << "</neighthresh>" << endl;

    return !!O;
}


/** ======================================================================================= */

BaseXMLReader::ProcessElement PRMParams::startElement ( const std::string& name, const std::list< pair< string, string > >& atts )
{
    if ( b_processing )
    {
        return PE_Ignore;
    }

    switch ( PlannerBase::PlannerParameters::startElement ( name,atts ) )
    {
    case PE_Pass:
        break;
    case PE_Support:
        return PE_Support;
    case PE_Ignore:
        return PE_Ignore;
    }

    b_processing = ( name == "bpruneroadmap" ||
                   name == "bsmoothpath" ||
                   name == "ntries" ||
                   name == "mnodes" ||
                   name == "medges" ||
                   name == "neighthresh" );

    return b_processing ? PE_Support : PE_Pass;
}


/** ======================================================================================= */

bool PRMParams::endElement ( const std::string& name )
{

    if ( b_processing )
    {
        if ( name == "pruneroadmap" )
        {
            _ss >> b_prune_roadmap;
        }
        else if ( name == "smoothpath" )
        {
            _ss >> b_smooth_path;
        }
        else if ( name == "ntries" )
        {
            _ss >> i_ntries;
        }
        else if ( name == "mnodes" )
        {
            _ss >> i_nnodes;
        }
        else if ( name == "medges" )
        {
            _ss >> i_nedges;
        }
        else if ( name == "neighthresh" )
        {
            _ss >> f_neigh_thresh;
        }
        else
        {
            RAVELOG_WARN ( str ( boost::format ( "unknown tag %s\n" ) %name ) );
        }

        b_processing = false;
        return false;
    }

    return PlannerParameters::endElement ( name );
}

}

#endif
