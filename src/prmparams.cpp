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


#include <prmparams.h>

using namespace openprm;


PRMParameters::PRMParameters() :
    max_tries_(3),
    max_nodes_(100),
    max_edges_(10),
    neighbor_threshold_(4.5),
    processing_(false)
{
    _vXMLParameters::push_back("max_tries");
    _vXMLParameters::push_back("max_nodes");
    _vXMLParameters::push_back("max_edges");
    _vXMLParameters::push_back("neighbor_threshold");
}




bool PRMParameters::serialize(std::ostream &output_stream)
{
    if (!PlannerParameters::serialize(output_stream))
    {
        return false;
    }

    output_stream << "<max_tries>" << max_tries_ << "</max_tries>" << endl;
    output_stream << "<max_nodes>" << max_nodes_ << "</max_nodes>" << endl;
    output_stream << "<max_edges>" << max_edges_ << "</max_edges>" << endl;
    output_stream << "<neighbor_threshold>" << neighbor_threshold_ << "</neighbor_threshold>" << endl;

    return !!output_stream;
}




BaseXMLReader::ProcessElement PRMParameters::startElement(const string &name, const AttributesList &atts)
{
    if (processing_)
    {
        return PE_Ignore;
    }

    switch (PlannerBase::PlannerParameters::startElement(name, atts))
    {
    case PE_Pass:
        break;
    case PE_Support:
        return PE_Support;
    case PE_Ignore:
        return PE_Ignore;
    }

    processing_ = (
                name == "max_tries" ||
                name == "max_nodes" ||
                name == "max_edges" ||
                name == "neighbor_threshold"
                );

    return processing_ ? PE_Support : PE_Pass;
}




bool PRMParameters::endElement(const string &name)
{
    if (processing_)
    {
        if ( name == "max_tries" )
            _ss >> max_tries_;
        else if ( name == "max_nodes" )
            _ss >> max_nodes_;
        else if ( name == "max_edges" )
            _ss >> max_edges_;
        else if ( name == "neighbor_threshold" )
            _ss >> neighbor_threshold_;
        else
        {
            RAVELOG_WARN( str(boost::format("unknown tag %s\n")%name ));
        }


        processing_ = false;
        return false;
    }

    /// process the default parameters
    return PlannerParameters::endElement(name);
}

