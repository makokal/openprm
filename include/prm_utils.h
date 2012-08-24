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

#ifndef PRM_UTILS_H
#define PRM_UTILS_H

#include <rave/rave.h>


// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST
#else

#include <string>
#include <vector>
#include <list>
#include <map>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#include <stdint.h>
#include <fstream>
#include <iostream>

#include <boost/assert.hpp>
#include <boost/bind.hpp>

using namespace std;

#include <sys/timeb.h> // ftime(), struct timeb

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

template<class T>
inline T CLAMP_ON_RANGE(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline uint32_t timeGetTime()
{
#ifdef _WIN32
    _timeb t;
    _ftime(&t);
#else
    timeb t;
    ftime(&t);
#endif

    return (uint32_t)(t.time*1000+t.millitm);
}

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

inline uint64_t GetMicroTime()
{
#ifdef _WIN32
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
#else
    struct timeval t;
    gettimeofday(&t, NULL);
    return (uint64_t)t.tv_sec*1000000+t.tv_usec;
#endif
}

struct null_deleter
{
    void operator()(void const *) const {}
};



using namespace OpenRAVE;



namespace openprm
{

/// define some utils
}


#endif // PRM_UTILS_H
