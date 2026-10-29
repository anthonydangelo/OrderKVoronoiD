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

        bool extractPointsFromJSON2DArrayString(const string &inputString, vector< MyPoint_2 > &result)
        {
            //NB: The string processing is a bit more flexible than the PrintHelp message suggests
            //TODO: consider switching to using a regex here

            size_t strIndex = inputString.find_first_of("(");
            if (strIndex != string::npos) 
            {
                cerr << "\nERROR: found '(' at position " << strIndex
                    << " but only '[' is expected. No points were read in.\n";
                return false;
            }

            strIndex = inputString.find_first_of("[");
            if (string::npos == strIndex)
            {
                cerr << "\nERROR: Could not read input list. Expected '[' to open the list but got: " << endl;
                cerr << inputString << endl;
                return false;
            }
            
            while (strIndex != string::npos)
            {
                strIndex = inputString.find_first_of("[", strIndex + 1);
                if (strIndex != string::npos)
                {
                    try
                    {
                        string::size_type stodIndex;
                        double firstD = stod(inputString.substr(strIndex + 1), &stodIndex);
                        // stodIndex is relative to the substring, not the original string
                        // stodIndex is the position of the first character in the substring after the number.
                        strIndex = inputString.find_first_of(",", stodIndex + strIndex);
                        if (strIndex != string::npos)
                        {
                            double secondD = stod(inputString.substr(strIndex + 1));
                            result.emplace_back(firstD, secondD);
                        }
                    }
                    catch (exception &e)
                    {
                        cerr << endl << "Exception caught: " << e.what() << endl;
                        cerr << "ERROR: Input point list string malformed. Try surrounding the list in quotes." << endl;
                        if (result.empty())
                        {
                            cerr << "No points were read in." << endl;                      
                        } else
                        {
                            cerr << "Successfully read in " << result.size() << " point(s). The last point read in successfully was: ";
                            MyPoint_2 p = result.back();
                            cerr << "( " << p.x() << ", " << p.y() << " )" << endl;
                            cerr << "List cleared." << endl << endl;
                            result.clear();                                  
                        }                  
                        return false;
                    }
                } 
            }
            return true;
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
