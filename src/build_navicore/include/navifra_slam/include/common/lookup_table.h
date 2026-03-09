/**
 * @class look up table
 * @brief  look up table for updating global grid map.
 * @author logan (donghak lee)  
 * contact : donghak.lee@navifra.com
*/
#ifndef LOOK_UP_TABLE_H
#define LOOK_UP_TABLE_H

#include <iostream>
#include <cmath>
#include <vector>

namespace NaviFra
{
    namespace SLAM2D
    {
        class LookUpTable
        {
        private:
            /* data */
            std::vector<float> logoddsTable;
            std::vector<uint8_t> logoddsTable_255;
            std::vector<int8_t> p2lTable;

            std::vector<float> logoddsTable_16;
            std::vector<uint16_t> logoddsTable_255_16;
            std::vector<int16_t> p2lTable_16;

        public:
            LookUpTable(/* args */);
            ~LookUpTable();

            static LookUpTable *GetInstance()
            {
                static LookUpTable s;
                return &s;
            }

            static constexpr int8_t CELLTYPE_MIN = -127;
            static constexpr int8_t CELLTYPE_MAX = 127;
            static constexpr int8_t P2LTABLE_SIZE = CELLTYPE_MAX;
            static constexpr std::size_t LOGODDS_LUT_ENTRIES = 1 << 8;

            static constexpr int16_t CELLTYPE_MIN_16 = -32767;
            static constexpr int16_t CELLTYPE_MAX_16 = 32767;
            static constexpr int16_t P2LTABLE_SIZE_16 = CELLTYPE_MAX_16;
            static constexpr std::size_t LOGODDS_LUT_ENTRIES_16 = 1 << 16;

            float LogOddsToProbability(const int8_t l);
            uint8_t l2p_255(const int8_t l);
            int8_t ProbabilityToLogOdds(const float p);

            float LogOddsToProbability_16(const int16_t l);
            uint16_t l2p_255_16(const int16_t l);
            int16_t ProbabilityToLogOdds_16(const float p);
        };
    }
}

#endif