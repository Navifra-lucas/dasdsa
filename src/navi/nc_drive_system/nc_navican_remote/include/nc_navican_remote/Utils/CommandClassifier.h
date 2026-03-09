#ifndef NAVIFRA_NAVICAN_REMOTE_COMMAND_CLASSIFIER
#define NAVIFRA_NAVICAN_REMOTE_COMMAND_CLASSIFIER

#include "nc_navican_remote/Utils/types.h"

#include <string>
#include <unordered_map>

namespace NaviFra {
namespace NaviCAN {
namespace Remote {

// 명령어 분류 유틸리티
class CommandClassifier {
public:
    static CommandType classifyCommand(const std::string& command);

private:
    static std::unordered_map<std::string, CommandType> command_types_;
    static void initializeCommandTypes();
};
}  // namespace Remote
}  // namespace NaviCAN
}  // namespace NaviFra

#endif