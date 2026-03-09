#ifndef _NAVIFRA_CHEONIL_EMUL_MANAGER_H
#define _NAVIFRA_CHEONIL_EMUL_MANAGER_H

#include <nc_cheonil_agent/emul/nc_cheonil_Emul_State.h>
#include <nc_cheonil_agent/state/state_machine.h>
#include <Poco/Runnable.h>
#include <Poco/Thread.h>
#include <Poco/SingletonHolder.h>

namespace NaviFra {
// State machine class
class CheonilEmulManager : public Poco::Runnable {
public:
    CheonilEmulManager();
    virtual ~CheonilEmulManager();

    CheonilEmulManager(const CheonilEmulManager&) = delete;
    CheonilEmulManager& operator=(const CheonilEmulManager&) = delete;

    static CheonilEmulManager& instance();

    void initialize();

public:
    virtual void run();

private:
    Poco::Thread worker_;
    // CheonilEmulController cheonilController_;
    bool bRun_;
};
}  // namespace Navifra

#endif  //_NAVIFRA_CHEONIL_EMUL_MANAGER_H