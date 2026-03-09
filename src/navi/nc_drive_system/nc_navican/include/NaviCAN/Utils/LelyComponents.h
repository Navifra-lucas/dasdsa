#ifndef NAVIFRA_NAVICAN_LELY_COMPONENTS_H
#define NAVIFRA_NAVICAN_LELY_COMPONENTS_H

#include <lely/ev/loop.hpp>
#include <lely/ev/exec.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>

#include "util/logger.hpp"

#include <memory>

namespace NaviFra {
class LelyComponents {
public:
    lely::io::IoGuard io_guard;
    lely::io::Context ctx;
    lely::io::Poll poll;
    lely::ev::Loop loop;
    std::shared_ptr<lely::ev::Executor> exec;
    lely::io::Timer timer;
    lely::io::CanController ctrl;
    lely::io::CanChannel chan;

    int getCanBusState() const {
        return static_cast<int>(ctrl.get_state());
    }
  
    LelyComponents(const std::string& ifname)
        : io_guard(),
          ctx(),
          poll(ctx),
          loop(poll.get_poll()),
          exec(std::make_shared<lely::ev::Executor>(loop.get_executor())),
          timer(poll, *exec, CLOCK_MONOTONIC),
          ctrl(ifname.c_str()),
          chan(poll, *exec) {
        chan.open(ctrl);
    }
};
}

#endif