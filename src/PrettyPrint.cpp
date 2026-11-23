#include "ConstantsAndUtilities.hpp"

namespace okvd
{
/*
**
**
 */
    namespace Pretty_Print
    {
        void writePointVectorJSON(std::ostream & out, const string & name, const vector< MyPoint_2 > & myColl, int tabLevel)
        {
            out << insertTabs(tabLevel);
            out << wrapStringInQuotes(name) << ": [\n";
        
            if(!myColl.empty()){
                auto endIt = end(myColl);
                for (auto it = begin(myColl); it != endIt; ++it) {
                    out << insertTabs(tabLevel + 2);
                    out << point2ToJSON(*it); 
                    if(next(it) != endIt){
                        out << ",";
                    } 
                    out << "\n";
                }    
            }

            out << insertTabs(tabLevel+2);
            out << "]"; 

            return;
        }

        void writeSegmentsIndicesJSON(std::ostream & out, const string & name, const vector< pair<size_t, size_t> > & segmentList, int tabLevel)
        {
            out << insertTabs(tabLevel);
            out << wrapStringInQuotes(name) << ": [\n";
        
            if(!segmentList.empty()){
                auto endIt = end(segmentList);
                for (auto it = begin(segmentList); it != endIt; ++it) {
                    out << insertTabs(tabLevel + 2);
                    out << "{" << wrapStringInQuotes(VD_EDGE_NAME_STRING) << ":";
                    out << "{" << wrapStringInQuotes(VD_EDGE_FIRST_INDEX_NAME_STRING) << ":\"" << it->first << "\", ";
                    out << wrapStringInQuotes(VD_EDGE_SECOND_INDEX_NAME_STRING) << ":\"" << it->second << "\"}";
                    out << "}"; 
                    if(next(it) != endIt){
                        out << ",";
                    }
                    out << "\n";
                }    
            }

            out << insertTabs(tabLevel+2);
            out << "]";
        
            return;
        }    

        void writeVoronoiDiagramJSON(std::ostream & out, const vector< MyPoint_2 > & endpointsVec, 
                                        const vector< pair<size_t, size_t> > & segmentEndPtsVec, 
                                        int tabLevel)
        {
            out << insertTabs(tabLevel);
            out << "{";
            writePointVectorJSON(out, VD_POINTS_NAME_STRING, endpointsVec, tabLevel + 2);
            out << ",\n";
            writeSegmentsIndicesJSON(out, VD_EDGE_LIST_PT_INDICES_NAME_STRING, segmentEndPtsVec, tabLevel + 2);
            out << insertTabs(tabLevel);
            out << "}";

            return;
        }

    } // namespace Pretty_Print
} // namespace okvd
