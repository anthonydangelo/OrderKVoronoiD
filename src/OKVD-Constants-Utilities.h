#ifndef OKVD_CONSTANTS_UTILITIES
#define OKVD_CONSTANTS_UTILITIES

#include <iostream>
#include <sstream> // for ostringstream
#include <string>
#include <vector>

#include <unistd.h>
#include <cstdlib>
#include <fstream>
#include <ctime>
#include <cstddef> // std::size_t

#include <math.h> /* fabs */

#include <utility>

//https://github.com/CGAL/cgal/blob/master/Generator/examples/Generator/random_grid.cpp
//https://doc.cgal.org/latest/Kernel_23/classCGAL_1_1Exact__predicates__exact__constructions__kernel.html
//https://doc.cgal.org/latest/Cone_spanners_2/index.html
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel_with_root_of.h>
#include <CGAL/algorithm.h>

#include <CGAL/Lazy_exact_nt.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Compute_cone_boundaries_2.h>

#include <CGAL/squared_distance_2.h> //for 2D functions
#include <CGAL/number_utils.h>       //for cgal::sqrt

#include <CGAL/ch_akl_toussaint.h>

#include <CGAL/Kernel/global_functions.h>
#include <CGAL/enum.h>
#include <CGAL/intersections.h>


//CGAL-4.11.2/demo/CGAL_ipelets/multi_delaunay.cpp

#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Regular_triangulation_face_base_2.h>
#include <CGAL/Regular_triangulation_vertex_base_2.h>
#include <CGAL/Regular_triangulation_2.h>



#define MY_LAZY_COMPS   (1)

//http://man7.org/linux/man-pages/man7/sem_overview.7.html
#define SEMAPHORE_NAME ("/ONE_ST_TREE_SEMAPHORE")
#define SEMAPHORE_INIT_VALUE (1)

#define MY_VERBOSE (0)

//magic numbers
#define DEFAULT_ORDER (1)
#define BBOX_OFFSET (10)

#define ELAPSED_TIME_NAME_STRING ("elapsedTime")
#define INPUT_POINTS_NAME_STRING ("inputPoints")
#define ORDER_NAME_STRING ("order")
#define VD_NAME_STRING ("voronoiDiagram")
#define VD_POINTS_NAME_STRING ("voronoiPoints")
#define VD_EDGE_LIST_PT_INDICES_NAME_STRING ("voronoiEdgesPtIndices")
#define VD_EDGE_NAME_STRING ("voronoiEdge")
#define VD_EDGE_FIRST_INDEX_NAME_STRING ("firstIndex")
#define VD_EDGE_SECOND_INDEX_NAME_STRING ("secondIndex")
//////////////////////////////

#define DOUBLE_EPSILON (0.000001)


namespace okvd 
{

    using std::cout;
    using std::cerr;
    using std::endl;
    using std::vector;
    using std::string;
    using std::ostringstream;
    using std::list;
    using std::size_t;
    using std::pair;

//https://doc.cgal.org/latest/Kernel_23/Kernel_23_2exact_8cpp-example.html#_a0
//https://doc.cgal.org/latest/Kernel_23/Kernel_23_2intersection_visitor_8cpp-example.html#_a0
//https://doc.cgal.org/latest/Kernel_d/group__PkgKernelDFunctions.html#ga0aa3e8b6bdf1bff509f8e2672ef194d1
//https://doc.cgal.org/latest/Cone_spanners_2/index.html
//typedef Exact_predicates_exact_constructions_kernel Kernel;
#   if (MY_LAZY_COMPS)
        typedef CGAL::Lazy_exact_nt<CORE::Expr> Lazy_FT;
        typedef CGAL::Simple_cartesian< Lazy_FT >  MyKernel;
#   else
        typedef CGAL::Exact_predicates_exact_constructions_kernel_with_root_of      MyKernel;
#   endif
//https://doc.cgal.org/latest/Arrangement_on_surface_2/index.html
    typedef MyKernel::FT                                                        My_Number_type;

    typedef MyKernel::Point_2                                                 MyPoint_2;
    typedef MyKernel::Segment_2                                               MySegment_2;
    typedef MyKernel::Line_2                                                  MyLine_2;
    typedef MyKernel::Intersect_2                                             MyIntersect_2;
    typedef MyKernel::Direction_2                                             MyDirection_2;

//  CGAL-4.11.2/demo/CGAL_ipelets/multi_delaunay.cpp
    typedef CGAL::Delaunay_triangulation_2<MyKernel>                                        MyDelaunay;
    typedef CGAL::Regular_triangulation_vertex_base_2<MyKernel>                             MyVb;
    typedef CGAL::Triangulation_vertex_base_with_info_2< vector<MyPoint_2>, MyKernel, MyVb> MyVbI;
    typedef CGAL::Regular_triangulation_face_base_2<MyKernel>                               MyFb;
    typedef CGAL::Triangulation_data_structure_2<MyVbI, MyFb>                               MyTds;
    typedef CGAL::Regular_triangulation_2<MyKernel, MyTds>                                  MyRegularI;
    typedef CGAL::Regular_triangulation_2<MyKernel>                                         MyRegularTriangulation;
    typedef MyRegularI::Vertex_handle                                                       MyVertexI;
    typedef typename CGAL::Weighted_point_2<MyKernel>                                       MyWeighted_point_2;
    typedef MyKernel::Ray_2                                                                 MyRay_2;
    typedef MyKernel::Segment_2                                                             MySegment_2;
    typedef MyKernel::Line_2                                                                MyLine_2;

    typedef struct Voronoi_from_tri
    { //Class using stream to get the voronoi diagram
        list<MyRay_2> ray_list;
        list<MyLine_2> line_list;
        list<MySegment_2> seg_list;

        void operator<<(const MyRay_2 &p) { ray_list.push_back(p); }
        void operator<<(const MyLine_2 &p) { line_list.push_back(p); }
        void operator<<(const MySegment_2 &p) { seg_list.push_back(p); }
    } Voronoi_from_tri;




/////////////////////
// Namespaces for utilities and printing.

/*
**
**
 */

    namespace Utility_Functions
    {

        bool extractPointsFromJSON2DArrayString(const string &inputString, vector< MyPoint_2 > &result);

        //Assume the numbers aren't too large or small for doubles
        inline bool pointsAreTooClose(const MyPoint_2 &first, const MyPoint_2 &second)
        {
            return ( (fabs(CGAL::to_double(first.x()) - CGAL::to_double(second.x())) < DOUBLE_EPSILON) 
                    && (fabs(CGAL::to_double(first.y()) - CGAL::to_double(second.y())) < DOUBLE_EPSILON) );
        }


        /*
        ** according to "the machine":
        ** A function template (as opposed to a full specialization) is already allowed to have identical definitions 
        ** in multiple translation units — that's the template ODR (ODR = One Definition Rule) rule. 
        ** inline adds nothing here.
        ** You'd only need inline if this were a full specialization (e.g. template<> bool findPointIndex<...>(...)), 
        ** where it's a regular function again and the normal one-definition rule applies.
        */
        //https://stackoverflow.com/questions/46485084/declare-template-function-to-accept-any-container-but-only-one-contained-type/46485265
        template < template < class ... > class Container, class ... Args >
        bool findPointIndex(const MyPoint_2 &pt, const Container< MyPoint_2, Args... > &myColl, size_t &myIndex)
        {
            myIndex = 0;
            if(!myColl.empty()){
                auto endIt = end(myColl);
                for (auto it = begin(myColl); it != endIt; ++it, ++myIndex) {
                    if(pointsAreTooClose(*it, pt)){
                        return true;
                    }
                }    
            }
            return false;
        }

        void insertSegmentEndpointsIntoPointSet(const vector<MySegment_2> &segmentList, 
                                                vector< MyPoint_2 > &endpointsVec,
                                                vector< pair<size_t, size_t> > &segmentEndPtsVec);  

        MyKernel::Iso_rectangle_2 computeBBox(const vector<MyPoint_2> &inputListVec);

    } // namespace Utility_Functions


/*
**
**
 */
    namespace Pretty_Print
    {
        inline string wrapStringInQuotes(const string &s)
        {
            return ("\"" + s + "\"");
        }

        inline string point2ToJSON(const MyPoint_2 &p)
        {
            ostringstream sStream;

            sStream << "{\"x\":\"" << p.x() << "\", "
                    << "\"y\":\"" << p.y() << "\"}";
            return sStream.str();
        } 

        inline string insertTabs(int level)
        {
            ostringstream sStream;
            for (int i = 0; i < level; ++i)
            {
                sStream << "\t";
            }
            return sStream.str();
        }

        string pointVectorToJSONString(string name, const vector< MyPoint_2 > &myColl, int tabLevel = 0);

        string segmentsIndicesToJSONString(string name, const vector< pair<size_t, size_t> > &segmentList, int tabLevel = 0);

        string voronoiDiagramToJSONString(const vector< MyPoint_2 > &endpointsVec, 
                                            const vector< pair<size_t, size_t> > &segmentEndPtsVec, 
                                            int tabLevel = 0);
    
    } // namespace Pretty_Print
} // namespace okvd
#endif
