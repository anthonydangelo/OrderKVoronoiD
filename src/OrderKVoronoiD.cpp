//============================================================================
// Name        : OrderKVoronoiD.cpp
// Author(s)   :- Sebastien Loriot, Sylvain Pion
//              - Sebastien Loriot, Nicolas Carrez
//              - Anthony D'Angelo (re-worked CGAL demos to output 
//                                  a json object of the OKVD arrangement 
//                                  clipped by a bounding box),
// Version     :
// Copyright   : From the CGAL demos:
/*
// Copyright (c) 2005-2009  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org); you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation; either version 3 of the License,
// or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL$
// $Id$
// 
//
*/
// Description : 
/*
** Modified CGAL-4.11.2/demo/CGAL_ipelets/multi_delaunay.cpp
** Computes the k-th order Voronoi diagram of n input points and outputs
** the arrangement (clipped to a bounding box) as a JSON object on stdout.
**
** Algorithm: for each k-subset of input points, compute a weighted point
** (centroid with weight = -sum of squared pairwise distances / k^2).
** The regular triangulation of these weighted points is the k-th order
** Delaunay; its dual is the k-th order Voronoi diagram.
**
** Special cases: k=1 uses standard Delaunay, k=2 and k=3 use optimized
** paths, k=n-1 uses a closed-form, general k uses brute-force enumeration.
** (Algorithm from CGAL's multi_delaunay ipelet demo.)   
*/
//============================================================================

#include <getopt.h>

#include "OKVD-Constants-Utilities.h"
#include "k_delaunay.h"
#include <CGAL/Iso_rectangle_2.h>

#include <chrono>  // for high_resolution_clock
#include <limits> 

#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */
#include <unistd.h>
#include <sys/file.h>
#include <errno.h>


using namespace okvd;

namespace  
{

    using std::stoi;

    struct OKVDResourceMutex {
        //FLOCK mutex
        int mutexFD = -1;
        OKVDResourceMutex() = default;
        ~OKVDResourceMutex() 
        { 
            if (mutexFD >= 0) 
            {
                close(mutexFD); 
            }
        }
        OKVDResourceMutex(const OKVDResourceMutex&) = delete;
        OKVDResourceMutex& operator=(const OKVDResourceMutex&) = delete;
        OKVDResourceMutex(OKVDResourceMutex&&) = delete;
        OKVDResourceMutex& operator=(OKVDResourceMutex&&) = delete;        
    };

    struct OKVDConfig
    {
        vector<MyPoint_2> inputListVec;
        int order = DEFAULT_ORDER;
        OKVDResourceMutex flockMutex;
        bool useMutex = true;
    };

    void PrintHelp()
    {
        ostringstream msg;
        msg << "\nOrderKVoronoiD / OrderKVoronoiD_d: Help\n"
               "====================================================================================================================\n"
               "--input-list (-i) < string s = \"[[x,y],[u,v],...]\">:  Calculate the VD of these 'n' points (need quotes around list)\n"
               "--order     (-o) < int k >:                           Calculate the k'th-order VD for 1 <= k < n (defaults to " << DEFAULT_ORDER << ")\n"
               "--no-mutex:                                           Do not use a resource mutex (currently a file lock with a blocking lock call).\n"
               "                                                      This option is useful, e.g., if you want to manage the mutex in an invoking bash script.\n"
               "--help      (-h):                                     Show help\n"
               "\ne.g., -o 2 -i \"[[1,0], [2,0], [3,0]]\"\n"
               "You must pass in at least \"-i\" and a list of at least 2 points.";

        cout << msg.str();
        return;
    }

    

    //https://codeyarns.com/2015/01/30/how-to-parse-program-options-in-c-using-getopt_long/
    bool ProcessArgs(int argc, char **argv, OKVDConfig &config) 
    {
        const char *const short_opts = "i:o:h";
        const option long_opts[] = {
            {"input-list", required_argument, nullptr, 'i'},
            {"order", required_argument, nullptr, 'o'},        
            {"no-mutex", no_argument, nullptr, 256},    // "Any value that won't collide with the short option chars. 256 is a common choice since it's outside the ASCII range."
            {"help", no_argument, nullptr, 'h'},
            {nullptr, no_argument, nullptr, 0}};
        string inPointsString = "";

        //note: string(optarg) is to take in and make a string
        while (true)
        {
            const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);

            if (-1 == opt)
            {
                break;
            }

            switch (opt)
            {
            case 'o':
                if (optarg)
                {
                    try
                    {
                        int temp = stoi(optarg);
                        if (temp > 0)
                        {
                            config.order = temp;
                        } else 
                        {
                            cerr << "ERROR: Invalid order. Using default of " << DEFAULT_ORDER << endl;
                        }
                    }
                    catch (...)
                    {
                        config.order = DEFAULT_ORDER;
                        cerr << endl << "Exception caught" << endl;
                        cerr << "ERROR: Invalid order. Trying to move forward with default of " << DEFAULT_ORDER << endl;
                    }
                }
                break;

            case 'i':
                if(optarg)
                {
                    inPointsString = string(optarg);
                    if ( !Utility_Functions::extractPointsFromJSON2DArrayString(inPointsString, config.inputListVec))
                    {
                        // malformed input point list. bounce back to user.
                        // error message printed in function
                        // config.inputListVec is returned empty
                        return false;
                    }
                }
                break;

            case 256:
                // request to not to use a resource mutex in the program.
                // this is useful if you want to manage a mutex in a bash script instead
                config.useMutex = false;
                break;

            case 'h': // -h or --help
            case '?': // Unrecognized option
            default:
                return false;
            }
        }
        return true;
    }

    void initializeOrDie(int argc, char **argv, OKVDConfig &config)
    {
        if ((1 == argc) 
            || !ProcessArgs(argc, argv, config) 
            || config.inputListVec.empty() 
            || config.order < 1 // order is defaulted or set in ProcessArgs
            || config.order >= config.inputListVec.size())
        {
            size_t numInPoints = config.inputListVec.size();
            cout << "\nThere was an error starting up the program. Review the following and ensure the inputs are legal." << endl;
            PrintHelp();
            cout << endl
                 << "You requested the order-" << config.order;
            cout << " Voronoi diagram on the " << numInPoints << " point(s): " << endl;
            cout << "\"";
            for (size_t ind = 0; ind < numInPoints; ++ind)
            {
                MyPoint_2 p = config.inputListVec[ind];
                cout << "( " << p.x() << ", " << p.y() << ") ";
                if (ind < numInPoints - 1)
                {
                    cout << ", ";
                }
            }
            cout << "\"" << endl;
            if (config.order >= numInPoints)
            {
                cout << "\nFor 'n' input points, the order number must not exceed 'n-1'.\n\n";
            }
            exit(1);
        }

        if(config.useMutex)
        {
            config.flockMutex.mutexFD = open(FLOCK_PATH, O_CREAT | O_RDWR, 0600);
            if (config.flockMutex.mutexFD == -1) 
            {
                int err = errno;
                cerr << "Failed to open the lock file \'" << FLOCK_PATH << "\' for the OKVD problem (used as a resource mutex): "
                        << strerror(err) << " (" << err << ")\n";
                exit(1);
            }

            // N.B.: the non-blocking version returns -1 if in use -- flock(mutexFD, LOCK_EX | LOCK_NB) 
            flock(config.flockMutex.mutexFD, LOCK_EX);
            // Lock is released automatically:
            // when you close(fd), when the process exits (even SIGKILL, even segfault)
            // for us, the destructor of the struct is called when it goes out of scope and calls close
        }

        return;
    }

    // Clips infinite Voronoi edges (rays, lines) to the bounding box,
    // converting them to finite segments suitable for output.   
    template <class T,class output_iterator>
    bool cast_into_seg(const T &obj, const MyKernel::Iso_rectangle_2 &bbox, output_iterator out_it) 
    {
        CGAL::Object obj_cgal = CGAL::intersection(obj, bbox);
        MySegment_2 s;
        bool ret=CGAL::assign(s, obj_cgal);
        if (ret) 
        {
            *out_it = s;
            ++out_it;
        }
        return ret;
    }

    // Clips infinite Voronoi edges (rays, lines) to the bounding box,
    // converting them to finite segments suitable for output.  
    template<class iterator,class output_iterator>
    void cast_into_seg(const iterator first, const iterator end,
                    const MyKernel::Iso_rectangle_2 &bbox, output_iterator out_it)
    {
        for (iterator it=first; it!=end; ++it)
        {
            cast_into_seg(*it,bbox,out_it);
        }
    }

    // Collects the dual (Voronoi) edges from the triangulation, filters
    // degenerate segments, clips to bbox, and deduplicates endpoints.   
    void build_dual_(Voronoi_from_tri &v_recup, const MyKernel::Iso_rectangle_2 &bbox, 
                    vector< MyPoint_2 > &endpointsVec,
                    vector< pair<size_t, size_t> > &segmentEndPtsVec)
    {
        vector<MySegment_2> seg_cont;
        //filter degenerate segments
        for(typename list<MySegment_2>::iterator iteS = v_recup.seg_list.begin(); iteS!=v_recup.seg_list.end();)
        {
            typename list<MySegment_2>::iterator itc=iteS++;
            if (itc->is_degenerate())
            {
                v_recup.seg_list.erase(itc);
            }
        }
        
        cast_into_seg(v_recup.ray_list.begin(),v_recup.ray_list.end(),bbox,std::back_inserter(seg_cont)); // clip rays to bbox
        cast_into_seg(v_recup.line_list.begin(),v_recup.line_list.end(),bbox,std::back_inserter(seg_cont)); // clip lines to bbox
        cast_into_seg(v_recup.seg_list.begin(),v_recup.seg_list.end(),bbox,std::back_inserter(seg_cont)); // clip segments to bbox
        Utility_Functions::insertSegmentEndpointsIntoPointSet(seg_cont, endpointsVec, segmentEndPtsVec);
    }

    template<class Triangulation>
    void build_dual_edge_list(Triangulation &T, const MyKernel::Iso_rectangle_2 &bbox, 
                                vector< MyPoint_2 > &endpointsVec, vector< pair<size_t, size_t> > &segmentEndPtsVec)
    {
        //~ template<class GT,class TDS>
        //~ void draw_dual_in_ipe(const CGAL::Triangulation_2<GT,TDS>& T,const Iso_rectangle_2& bbox) const{
        Voronoi_from_tri v_recup;
        T.draw_dual(v_recup);
        build_dual_(v_recup,bbox, endpointsVec, segmentEndPtsVec);
    }

} // anonymous namespace


int main(int argc, char **argv)
{    
    OKVDConfig config;
    vector< MyPoint_2 > endpointsVec;
    vector< pair<size_t, size_t> > segmentEndPtsVec;

    initializeOrDie(argc, argv, config);

    //https://www.pluralsight.com/blog/software-development/how-to-measure-execution-time-intervals-in-c--
    // Record start time
    auto start = std::chrono::high_resolution_clock::now();
    
    MyKernel::Iso_rectangle_2 bbox = Utility_Functions::computeBBox(config.inputListVec);

    MyDelaunay dt;
    MyRegularI rti;
    MyRegularTriangulation rt;
    
    dt.insert(config.inputListVec.begin(), config.inputListVec.end());


    if ((2 == config.order) || (3 == config.order)) 
    {
        // Delaunay and Voronoi for 2nd-3rd order
        // Order 2: for each edge of the 1st-order Delaunay, insert the midpoint
        // with weight = -(edge length)^2 / 4. The regular triangulation of these
        // is the 2nd-order Delaunay.   
        for (MyDelaunay::Finite_edges_iterator it=dt.finite_edges_begin(); it!=dt.finite_edges_end(); ++it)
        {
            MyPoint_2 pt0=it->first->vertex(MyDelaunay::cw(it->second))->point();
            MyPoint_2 pt1=it->first->vertex(MyDelaunay::ccw(it->second))->point();
            MyVertexI vertI_cgal = rti.insert(MyWeighted_point_2(CGAL::midpoint(pt0,pt1),-CGAL::to_double(CGAL::squared_distance(pt0,pt1))/4.));
            config.inputListVec.clear();
            config.inputListVec.push_back(pt0);
            config.inputListVec.push_back(pt1);
            vertI_cgal -> info() = config.inputListVec;
        }

        // Order 3: for each pair of adjacent edges in the 2nd-order RT sharing
        // a common original point, insert the centroid of the 3 original points
        // with weight = -(sum of squared pairwise distances) / 9.   
        if (3 == config.order)
        { 
            //Pour l'order 3
            //CAN WE ITERATE OVER DELAUNEY TRIANGLES???
            //WE MAY COUNT SEVERAL TIME SAME TRIANGLE WITH THE FOLLOWING METHOD
            //iterate over adjacent point in the regular triangulation and compute a new wpoint for those having one commun parent from delaunay
            for (MyRegularI::Finite_edges_iterator it = rti.finite_edges_begin(); it != rti.finite_edges_end(); ++it)
            {
                MyPoint_2 pt0_ori0 = it->first->vertex(MyDelaunay::cw(it->second))->info().front();
                MyPoint_2 pt0_ori1 = it->first->vertex(MyDelaunay::cw(it->second))->info().back();
                MyPoint_2 pt1_ori0 = it->first->vertex(MyDelaunay::ccw(it->second))->info().front();
                MyPoint_2 pt1_ori1 = it->first->vertex(MyDelaunay::ccw(it->second))->info().back();

                if (CGAL::compare_xy(pt0_ori0, pt1_ori0) == CGAL::EQUAL || CGAL::compare_xy(pt0_ori1, pt1_ori0) == CGAL::EQUAL)
                {
                    rt.insert(MyWeighted_point_2(CGAL::centroid(pt0_ori0, pt0_ori1, pt1_ori1), 
                                                                - CGAL::to_double(CGAL::squared_distance(pt0_ori0, pt0_ori1) 
                                                                                    + CGAL::squared_distance(pt0_ori0, pt1_ori1) 
                                                                                    + CGAL::squared_distance(pt1_ori1, pt0_ori1)) 
                                                                / 9.));
                }
                else if (CGAL::compare_xy(pt0_ori0, pt1_ori1) == CGAL::EQUAL || CGAL::compare_xy(pt0_ori1, pt1_ori1) == CGAL::EQUAL)
                {
                    rt.insert(MyWeighted_point_2(CGAL::centroid(pt0_ori0, pt0_ori1, pt1_ori0), 
                                                                -CGAL::to_double(CGAL::squared_distance(pt0_ori0, pt0_ori1) 
                                                                                    + CGAL::squared_distance(pt0_ori0, pt1_ori0) 
                                                                                    + CGAL::squared_distance(pt1_ori0, pt0_ori1)) 
                                                                / 9.));
                }
            }
        }
    } else if (config.order == (config.inputListVec.size() - 1))
    {
        // Order n-1: for each input point p, insert the centroid of all other
        // points with weight = -(total squared distance - distances to p) / (n-1)^2.
        // This is O(n^2) total vs. O(C(n,n-1)) = O(n) for the general case,
        // but the constant is much smaller.           
        double pt_x0 = 0; //base to compute centroid of n-1 points
        double pt_y0 = 0; //base to compute centroid of n-1 points
        double wt = 0;    //total weight : sum of all distances between two input points
        for (vector<MyPoint_2>::iterator it_pt = config.inputListVec.begin(); it_pt != config.inputListVec.end(); ++it_pt)
        {
            pt_x0 = pt_x0 + CGAL::to_double((*it_pt).x());
            pt_y0 = pt_y0 + CGAL::to_double((*it_pt).y());
            for (vector<MyPoint_2>::iterator it_pt2 = it_pt + 1; it_pt2 != config.inputListVec.end(); ++it_pt2)
            {
                wt = wt + CGAL::to_double(CGAL::squared_distance(*it_pt, *it_pt2));
            }
        }
        for (vector<MyPoint_2>::iterator it_pt = config.inputListVec.begin(); it_pt != config.inputListVec.end(); ++it_pt)
        {
            double w = wt;
            //compute centroid of the set of input points / *it_pt
            double pt_x = pt_x0 - CGAL::to_double((*it_pt).x());
            double pt_y = pt_y0 - CGAL::to_double((*it_pt).y());
            //remove from w the sum of distance of input point from *it_pt
            for (vector<MyPoint_2>::iterator it_pt2 = config.inputListVec.begin(); it_pt2 != config.inputListVec.end(); ++it_pt2)
            { //Weighted_point_2 equivalent
                w = w - CGAL::to_double(CGAL::squared_distance(*it_pt, *it_pt2));
            }
            w = -w / (double)(config.order * config.order);
            pt_x = pt_x / (double)config.order;
            pt_y = pt_y / (double)config.order;
            rt.insert(MyWeighted_point_2(MyPoint_2(pt_x, pt_y), w));
        }
    } 
    else
    {
        //k-th Delauney and Voronoi
        k_delaunay<MyKernel>(rt, config.inputListVec, config.order);
    }

    if (1 == config.order)
    {
        build_dual_edge_list(dt, bbox, endpointsVec, segmentEndPtsVec);
    } 
    else if (2 == config.order)
    {
        build_dual_edge_list(rti, bbox, endpointsVec, segmentEndPtsVec);
    }
    else 
    {
        build_dual_edge_list(rt, bbox, endpointsVec, segmentEndPtsVec);
    }

    ostringstream sStream;


    sStream << "{\n" << Pretty_Print::wrapStringInQuotes(ORDER_NAME_STRING) << ": \"" << config.order << "\",";
    sStream << "\n" << Pretty_Print::wrapStringInQuotes(VD_NAME_STRING) << ": ";
    sStream << Pretty_Print::voronoiDiagramToJSONString(endpointsVec, segmentEndPtsVec) << ",\n";

    sStream << Pretty_Print::wrapStringInQuotes(ELAPSED_TIME_NAME_STRING) << " : {";
    // Record end time
    auto finish = std::chrono::high_resolution_clock::now();


    std::chrono::duration<double> elapsed = finish - start;
    double timeInSecs = elapsed.count();
    long timeNumb = static_cast<long>(floor(timeInSecs));

    sStream << "\"s\":\"" << (timeNumb % 60) << "\", \"m\":\"";
    timeNumb /= 60;
    sStream << (timeNumb % 60) << "\", \"h\":\""; 
    timeNumb /= 60;
    sStream << timeNumb << "\" }\n }" << endl;
    cout << sStream.str();

    return 0;
}
