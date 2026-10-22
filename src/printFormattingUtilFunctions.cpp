#include "OKVD-Constants-Utilities.h"

namespace okvd
{
/*
**
**
 */
    namespace Pretty_Print
    {
        string pointVectorToJSONString(string name, const vector< MyPoint_2 >& myColl, int tabLevel)
        {
            ostringstream sStream;
            sStream << insertTabs(tabLevel);
            sStream << wrapStringInQuotes(name) << ": [\n";
        
            if(!myColl.empty()){
                auto endIt = end(myColl);
                for (auto it = begin(myColl); it != endIt; ++it) {
                    sStream << insertTabs(tabLevel + 2);
                    sStream << point2ToJSON(*it); 
                    if(next(it) != endIt){
                        sStream << ",";
                    } 
                    sStream << "\n";
                }    
            }

            sStream << insertTabs(tabLevel+2);
            sStream << "]"; 
        
            return sStream.str();
        }

        string segmentsIndicesToJSONString(string name, const vector< pair<size_t, size_t> >& segmentList, int tabLevel)
        {

            ostringstream sStream;
            sStream << insertTabs(tabLevel);
            sStream << wrapStringInQuotes(name) << ": [\n";
        
            if(!segmentList.empty()){
                auto endIt = end(segmentList);
                for (auto it = begin(segmentList); it != endIt; ++it) {
                    sStream << insertTabs(tabLevel + 2);
                    sStream << "{" << wrapStringInQuotes(VD_EDGE_NAME_STRING) << ":";
                    sStream << "{" << wrapStringInQuotes(VD_EDGE_FIRST_INDEX_NAME_STRING) << ":\"" << it->first << "\", ";
                    sStream << wrapStringInQuotes(VD_EDGE_SECOND_INDEX_NAME_STRING) << ":\"" << it->second << "\"}";
                    sStream << "}"; 
                    if(next(it) != endIt){
                        sStream << ",";
                    }
                    sStream << "\n";
                }    
            }

            sStream << insertTabs(tabLevel+2);
            sStream << "]";
        
            return sStream.str();
        }    

        string voronoiDiagramToJSONString(const vector< MyPoint_2 >& endpointsVec, 
                                            const vector< pair<size_t, size_t> >& segmentEndPtsVec, 
                                            int tabLevel)
        {
            ostringstream sStream;
            sStream << insertTabs(tabLevel);
            sStream << "{";
            sStream << pointVectorToJSONString(VD_POINTS_NAME_STRING, endpointsVec, tabLevel + 2) << ",\n";
            sStream << segmentsIndicesToJSONString(VD_EDGE_LIST_PT_INDICES_NAME_STRING, segmentEndPtsVec, tabLevel + 2);
            sStream << insertTabs(tabLevel);
            sStream << "}";
                
            return sStream.str();
        }

    } // namespace Pretty_Print
} // namespace okvd