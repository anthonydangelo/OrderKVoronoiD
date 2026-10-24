#include "OKVD-Constants-Utilities.h"

namespace okvd
{
/*
**
**
 */
    namespace Utility_Functions
    {
        using std::exception;

        void extractPointsFromJSON2DArrayString(string &inputString, vector< MyPoint_2 > &result)
        {
            //I'd like to use a regex here, but I don't know how to write the grammar...
            //http://www.cplusplus.com/reference/string/string/find_first_of/
            size_t strIndex = inputString.find_first_of("[");
            //I don't like the way I'm doing this parsing!
            while (strIndex != string::npos)
            {
                strIndex = inputString.find_first_of("[", strIndex + 1);
                if (strIndex != string::npos)
                {
                    try
                    {
                        //http://www.cplusplus.com/reference/string/stod/
                        string::size_type stodIndex;
                        double firstD = stod(inputString.substr(strIndex + 1), &stodIndex);
                        strIndex = inputString.find_first_of(",", stodIndex + strIndex);
                        if (strIndex != string::npos)
                        {
                            double secondD = stod(inputString.substr(strIndex + 1));
                            result.emplace_back(firstD, secondD);
                        }
                    }
                    catch (exception &e)
                    {
                        cerr << e.what() << endl;
                        cerr << "Input point list string malformed" << endl;
                        result.clear();
                        return;
                    }
                }
            }
            return;
        }   

        
        void insertSegmentEndpointsIntoPointSet(const vector<MySegment_2> &segmentList, 
                                                vector< MyPoint_2 > &endpointsVec,
                                                vector< pair<size_t, size_t> > &segmentEndPtsVec)
        {
            for(auto it = segmentList.begin(); it != segmentList.end(); ++it)
            {
                size_t firstIndex = 0;
                size_t secondIndex = 0;
                MyPoint_2 first = it->vertex(0);
                MyPoint_2 second = it->vertex(1);
                if (!findPointIndex(first, endpointsVec, firstIndex))
                {
                    firstIndex = endpointsVec.size();
                    endpointsVec.emplace_back(first.x(), first.y());
                }
                if (!findPointIndex(second, endpointsVec, secondIndex))
                {
                    secondIndex = endpointsVec.size();
                    endpointsVec.emplace_back(second.x(), second.y());
                }
                segmentEndPtsVec.emplace_back(pair<size_t, size_t>(firstIndex, secondIndex));            
            }    
            return;
        }    

        MyKernel::Iso_rectangle_2 computeBBox(const vector<MyPoint_2> &inputListVec)
        {
            double maxX = std::numeric_limits<double>::min();
            double maxY = std::numeric_limits<double>::min();
            double minX = std::numeric_limits<double>::max();
            double minY = std::numeric_limits<double>::max();

            size_t numInPoints = inputListVec.size();

            for (const MyPoint_2 &p : inputListVec)
            {
                if (p.x() > maxX)
                {
                    maxX = CGAL::to_double(p.x());
                }
                if (p.y() > maxY)
                {
                    maxY = CGAL::to_double(p.y());
                }
                if (p.x() < minX)
                {
                    minX = CGAL::to_double(p.x());
                }
                if (p.y() < minY)
                {
                    minY = CGAL::to_double(p.y());
                }
            }

            MyPoint_2 bboxTopRight(BBOX_OFFSET + maxX, BBOX_OFFSET + maxY);
            MyPoint_2 bboxBottomLeft(minX - BBOX_OFFSET, minY - BBOX_OFFSET);

            return MyKernel::Iso_rectangle_2(bboxBottomLeft, bboxTopRight);
        }


    } // namespace Utility_Functions
} // namespace okvd
