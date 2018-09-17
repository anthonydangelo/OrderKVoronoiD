//============================================================================
// Name        : OrderKVoronoiD.cpp
// Author      : Anthony D'Angelo (re-worked CGAL demo),
//                 // Author(s)     : Sebastien Loriot, Sylvain Pion
//                 // Author(s)     : Sebastien Loriot, Nicolas Carrez
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
//============================================================================

#include <getopt.h>

#include "OKVD-Constants-Utilities.h"
#include "k_delaunay.h"
#include <CGAL/Iso_rectangle_2.h>

#include <chrono>  // for high_resolution_clock
#include <limits> 

//Posix semaphores
//http://man7.org/linux/man-pages/man7/sem_overview.7.html
//http://man7.org/linux/man-pages/man3/sem_open.3.html
#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */
#include <semaphore.h>
#include <errno.h>

void PrintHelp()
{
    ostringstream msg;
    msg << "--inputList (-i) <string s = [[x,y],[u,v],...]>: Calculate the VD of these points\n"
           "--order (-o) <int k>: Calculate the k'th-order VD (defaults to " << DEFAULT_ORDER << ")\n"
           "--help (-h):              Show help\n";

    cout << msg.str();
    exit(1);
}

int order = DEFAULT_ORDER;
string inputListString = "";
vector< MyPoint_2 > endpointsVec;
vector< pair<size_t, size_t> > segmentEndPtsVec;

//https://codeyarns.com/2015/01/30/how-to-parse-program-options-in-c-using-getopt_long/
void ProcessArgs(int argc, char **argv, vector<MyPoint_2>& inputListVec)
{
    const char *const short_opts = "i:o:h";
    const option long_opts[] = {
        {"inputList", required_argument, nullptr, 'i'},
        {"order", required_argument, nullptr, 'o'},        
        {"help", no_argument, nullptr, 'h'},
        {nullptr, no_argument, nullptr, 0}};

    //note: string(optarg) is to take in and make a string
    while (true)
    {
        const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);

        if (-1 == opt)
            break;

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
                        order = temp;
                    } else 
                    {
                        cerr << "Invalid order. Using default of " << DEFAULT_ORDER << endl;
                    }
                }
                catch (...)
                {
                    order = DEFAULT_ORDER;
                    cerr << "Invalid order. Using default of " << DEFAULT_ORDER << endl;
                }
            }
            break;

        case 'i':
            if(optarg)
            {
                inputListString = string(optarg);
                Vasco_Rossi::extractPointsFromJSON2DArrayString(inputListString, inputListVec);              
            }
            break;

        case 'h': // -h or --help
        case '?': // Unrecognized option
        default:
            PrintHelp();
            break;
        }
    }
    return;
}

  template <class T,class output_iterator>
    bool cast_into_seg(const T& obj,const MyKernel::Iso_rectangle_2& bbox,output_iterator out_it) 
    {
      CGAL::Object obj_cgal = CGAL::intersection(obj,bbox);
      MySegment_2 s;
      bool ret=CGAL::assign(s, obj_cgal);
      if (ret) *out_it++=s;
      return ret;
    }

//Convert infinite objects into drawable segments
    template<class iterator,class output_iterator>
    void cast_into_seg(const iterator first,const iterator end,
                  const MyKernel::Iso_rectangle_2& bbox, output_iterator out_it)
    {
      for (iterator it=first; it!=end; ++it)
      {
        cast_into_seg(*it,bbox,out_it);
      }
    }

void build_dual_(Voronoi_from_tri& v_recup,const MyKernel::Iso_rectangle_2& bbox)
    {
      vector<MySegment_2> seg_cont;
      //filter degenerate segments
      for(typename list<MySegment_2>::iterator iteS = v_recup.seg_list.begin(); iteS!=v_recup.seg_list.end();){
        typename list<MySegment_2>::iterator itc=iteS++;
        if (itc->is_degenerate())
        {
          v_recup.seg_list.erase(itc);
        }
      }
      
      cast_into_seg(v_recup.ray_list.begin(),v_recup.ray_list.end(),bbox,std::back_inserter(seg_cont));//cast rays into segments in bbox
      cast_into_seg(v_recup.line_list.begin(),v_recup.line_list.end(),bbox,std::back_inserter(seg_cont));//cast lines into segments in bbox
      cast_into_seg(v_recup.seg_list.begin(),v_recup.seg_list.end(),bbox,std::back_inserter(seg_cont));//cast lines into segments in bbox
      Vasco_Rossi::insertSegmentEndpointsIntoPointSet(seg_cont, endpointsVec, segmentEndPtsVec);
    }

template<class Triangulation>
void build_dual_edge_list(Triangulation& T,const MyKernel::Iso_rectangle_2& bbox)
    {
    //~ template<class GT,class TDS>
    //~ void draw_dual_in_ipe(const CGAL::Triangulation_2<GT,TDS>& T,const Iso_rectangle_2& bbox) const{
      Voronoi_from_tri v_recup;
      T.draw_dual(v_recup);
      build_dual_(v_recup,bbox);
    }

int main(int argc, char **argv)
{

    vector<MyPoint_2> inputListVec;

    //There's some kind of CGAL destructor bug that blows up if we have this vector as a global...
    ProcessArgs(argc, argv, inputListVec);

    sem_t* pSem = sem_open(SEMAPHORE_NAME, O_CREAT, (S_IRWXU | S_IRWXG), SEMAPHORE_INIT_VALUE);

    if (SEM_FAILED == pSem)
    {
        perror("Failed to open a semaphore for OKVD problem");
        return 1;
    }

    sem_wait(pSem);

//https://www.pluralsight.com/blog/software-development/how-to-measure-execution-time-intervals-in-c--
    // Record start time
    auto start = std::chrono::high_resolution_clock::now();

    size_t numInPoints = inputListVec.size();

    MyKernel::Iso_rectangle_2 bbox = Vasco_Rossi::computeBBox(inputListVec);

    MyDelaunay dt;
    MyRegularI rti;
    MyRegularTriangulation rt;
    
    dt.insert(inputListVec.begin(),inputListVec.end());

    if ((2 == order) || (3 == order)) 
    {
        //Delaunay and Voronoi for 2nd-3rd order
        for (MyDelaunay::Finite_edges_iterator it=dt.finite_edges_begin(); it!=dt.finite_edges_end(); ++it)
        {
            MyPoint_2 pt0=it->first->vertex(MyDelaunay::cw(it->second))->point();
            MyPoint_2 pt1=it->first->vertex(MyDelaunay::ccw(it->second))->point();
            MyVertexI vertI_cgal = rti.insert(MyWeighted_point_2(CGAL::midpoint(pt0,pt1),-CGAL::to_double(CGAL::squared_distance(pt0,pt1))/4.));
            inputListVec.clear();
            inputListVec.push_back(pt0);
            inputListVec.push_back(pt1);
            vertI_cgal -> info() = inputListVec;
        }

        if (3 == order)
        { //Pour l'order 3
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
                    rt.insert(MyWeighted_point_2(CGAL::centroid(pt0_ori0, pt0_ori1, pt1_ori1), -CGAL::to_double(CGAL::squared_distance(pt0_ori0, pt0_ori1) +
                                                                                                                CGAL::squared_distance(pt0_ori0, pt1_ori1) +
                                                                                                                CGAL::squared_distance(pt1_ori1, pt0_ori1)) /
                                                                                                   9.));
                }
                else if (CGAL::compare_xy(pt0_ori0, pt1_ori1) == CGAL::EQUAL || CGAL::compare_xy(pt0_ori1, pt1_ori1) == CGAL::EQUAL)
                {
                    rt.insert(MyWeighted_point_2(CGAL::centroid(pt0_ori0, pt0_ori1, pt1_ori0), -CGAL::to_double(CGAL::squared_distance(pt0_ori0, pt0_ori1) +
                                                                                                                CGAL::squared_distance(pt0_ori0, pt1_ori0) + 
                                                                                                                CGAL::squared_distance(pt1_ori0, pt0_ori1)) /
                                                                                                   9.));
                }
            }
        }
    } else if (order == (numInPoints - 1))
    {
        double pt_x0 = 0; //base to compute centroid of n-1 points
        double pt_y0 = 0; //base to compute centroid of n-1 points
        double wt = 0;    //total weight : sum of all distances between two input points
        for (vector<MyPoint_2>::iterator it_pt = inputListVec.begin(); it_pt != inputListVec.end(); ++it_pt)
        {
            pt_x0 = pt_x0 + CGAL::to_double((*it_pt).x());
            pt_y0 = pt_y0 + CGAL::to_double((*it_pt).y());
            for (vector<MyPoint_2>::iterator it_pt2 = it_pt + 1; it_pt2 != inputListVec.end(); ++it_pt2)
            {
                wt = wt + CGAL::to_double(CGAL::squared_distance(*it_pt, *it_pt2));
            }
        }
        for (vector<MyPoint_2>::iterator it_pt = inputListVec.begin(); it_pt != inputListVec.end(); ++it_pt)
        {
            double w = wt;
            //compute centroid of the set of input points / *it_pt
            double pt_x = pt_x0 - CGAL::to_double((*it_pt).x());
            double pt_y = pt_y0 - CGAL::to_double((*it_pt).y());
            //remove from w the sum of distance of input point from *it_pt
            for (vector<MyPoint_2>::iterator it_pt2 = inputListVec.begin(); it_pt2 != inputListVec.end(); ++it_pt2)
            { //Weighted_point_2 equivalent
                w = w - CGAL::to_double(CGAL::squared_distance(*it_pt, *it_pt2));
            }
            w = -w / (double)(order * order);
            pt_x = pt_x / (double)order;
            pt_y = pt_y / (double)order;
            rt.insert(MyWeighted_point_2(MyPoint_2(pt_x, pt_y), w));
        }
    } 
    else
    {
        //k-th Delauney and Voronoi
        if (order < 1 || order >= numInPoints)
        {
            //Garbage request or the answer is the whole plane
            return 1;
        }
        k_delaunay<MyKernel>(rt, inputListVec, order);
    }

    if (1 == order)
    {
        build_dual_edge_list(dt,bbox);
    } 
    else if (2 == order)
    {
        build_dual_edge_list(rti,bbox);
    }
    else 
    {
        build_dual_edge_list(rt,bbox);
    }

    ostringstream sStream;


    sStream << "{\n" << Marisa_Tomei::wrapStringInQuotes(ORDER_NAME_STRING) << ": \"" << order << "\",";
    sStream << "\n" << Marisa_Tomei::wrapStringInQuotes(VD_NAME_STRING) << ": ";
    sStream << Marisa_Tomei::voronoiDiagramToJSONString(endpointsVec, segmentEndPtsVec) << ",\n";

    sStream << Marisa_Tomei::wrapStringInQuotes(ELAPSED_TIME_NAME_STRING) << " : {";
    // Record end time
    auto finish = std::chrono::high_resolution_clock::now();

    sem_post(pSem);
    //http://man7.org/linux/man-pages/man3/sem_close.3.html
    /*All open named semaphores are automatically closed on process
       termination, or upon execve(2). 
       Oh well. Do it anyway.*/
    sem_close(pSem);

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
