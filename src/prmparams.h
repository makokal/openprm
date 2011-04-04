/**
 * Copyright (c) 2010-2011, Billy Okal
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 * must display the following acknowledgement:
 * This product includes software developed by the author.
 * 4. Neither the name of the author nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file prmparams.h
 * PRM Parameters
 */

#ifndef PRMPARAMS_H
#define PRMPARAMS_H

#include "oputils.h"

namespace openprm
{

class PRMParams : public PlannerBase::PlannerParameters
{
public:
    PRMParams();

    bool bPruneroadmap;
    bool bSmoothpath;
    unsigned int iNtries;
    unsigned int iMnodes;
    unsigned int iMedges;
	dReal fNeighthresh;

protected:

    bool bProcessing;
    virtual bool serialize ( std::ostream& O ) const;
    BaseXMLReader::ProcessElement startElement ( const std::string& name, const std::list<std::pair<std::string,std::string> >& atts );
    virtual bool endElement ( const std::string& name );
};



PRMParams::PRMParams () :
        bPruneroadmap ( false ),
        bSmoothpath ( false ),
        iNtries ( 10 ),
        iMnodes ( 100 ),
        iMedges ( 10 ),
        bProcessing ( false ),
        fNeighthresh ( 4.5 )
{

    _vXMLParameters.push_back ( "bpruneroadmap" );
    _vXMLParameters.push_back ( "bsmoothpath" );
    _vXMLParameters.push_back ( "ntries" );
	_vXMLParameters.push_back ( "mnodes" );
	_vXMLParameters.push_back ( "medges" );
	_vXMLParameters.push_back ( "neighthresh" );
}

bool PRMParams::serialize ( std::ostream& O ) const
{

    if ( !PlannerParameters::serialize ( O ) )
    {
        return false;
    }

    O << "<pruneroadmap>" << bPruneroadmap << "</pruneroadmap>" << endl;

    O << "<smoothpath>" << bSmoothpath << "</smoothpath>" << endl;

    O << "<ntries>" << iNtries << "</ntries>" << endl;
	
	O << "<mnodes>" << iMnodes << "</mnodes>" << endl;

	O << "<medges>" << iMedges << "</medges>" << endl;

	O << "<neighthresh>" << fNeighthresh << "</neighthresh>" << endl;
	
    return !!O;
}

BaseXMLReader::ProcessElement PRMParams::startElement ( const std::string& name, const std::list< pair< string, string > >& atts )
{
    if ( bProcessing )
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

    bProcessing = ( name == "bpruneroadmap" || 
					name == "bsmoothpath" || 
					name == "ntries" ||
					name == "mnodes" ||
					name == "medges" ||
					name == "neighthresh" );

    return bProcessing ? PE_Support : PE_Pass;
}

bool PRMParams::endElement ( const std::string& name )
{

    if ( bProcessing )
    {
        if ( name == "pruneroadmap" )
        {
            _ss >> bPruneroadmap;
        }
        else if ( name == "smoothpath" )
        {
            _ss >> bSmoothpath;
        }
        else if ( name == "ntries" )
        {
            _ss >> iNtries;
        }
        else if ( name == "mnodes" )
		{
			_ss >> iMnodes;
		}
		else if ( name == "medges" )
		{
			_ss >> iMedges;
		}
		else if ( name == "neighthresh" )
		{
			_ss >> fNeighthresh;
		}
        else
        {
            RAVELOG_WARN ( str ( boost::format ( "unknown tag %s\n" ) %name ) );
        }

        bProcessing = false;
        return false;
    }

    return PlannerParameters::endElement ( name );
}

}

#endif