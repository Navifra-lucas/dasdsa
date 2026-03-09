
// #include "nc_cheonil_agent/emul/nc_cheonil_Emul_Manager.h"
// #include "nc_cheonil_agent/data/PDUdefinition.h"

// using namespace NaviFra;

// CheonilEmulManager::CheonilEmulManager()
//     : cheonilController_(CheonilEmulState::IDLE)
// {
// }

// CheonilEmulManager& CheonilEmulManager::instance()
// {
//     static Poco::SingletonHolder<CheonilEmulManager> sh;
//     return *sh.get();
// }

// CheonilEmulManager::~CheonilEmulManager()
// {
//     worker_.join();
// }

// void CheonilEmulManager::initialize()
// {
//     bRun_ = true;
//     worker_.start(*this);
//     NLOG(info) << "start cheonil emulator";
// }

// void CheonilEmulManager::run()
// {
//     while (bRun_) {
//         cheonilController_.excute();
//         Poco::Thread::sleep(50);
//     }
// }
