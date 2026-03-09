#ifndef NAVIFRA_NAVICAN_MASTER_H
#define NAVIFRA_NAVICAN_MASTER_H

#include <lely/coapp/master.hpp>
#include <lely/io2/can.h>
#include "util/logger.hpp"

using namespace lely;
using namespace std::chrono_literals;

namespace NaviFra
{
    class NaviCANMaster : public canopen::AsyncMaster {
        public:
        NaviCANMaster(ev_exec_t * exec, io::TimerBase& timer, io::CanChannelBase& chan,
                const std::string& dcf_txt, const std::string& dcf_bin,
                uint8_t master_id) :
            canopen::AsyncMaster(exec, timer, chan, dcf_txt, dcf_bin, master_id),
            timer_(timer),
            chan_(chan) {
            }

        private:
            io::TimerBase& timer_;
            io::CanChannelBase& chan_;

    };
    
} // namespace NaviFra

#endif