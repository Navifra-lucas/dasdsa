#ifndef NAVIFRA_ASTRA_MESSAGE_H
#define NAVIFRA_ASTRA_MESSAGE_H
#include "core_astra/zmq_handler.h"

#include <string>

namespace NaviFra {
void onMessageCollector(const void* sender, const std::string& msg);
void handleRouterRequest(const void* sender, RouterMsg& ev);
}  // namespace NaviFra

#endif  // NAVIFRA_ASTRA_MESSAGE_H