#include "nc_task_manager/nc_task_manager_pch.h"

#include <Poco/Exception.h>
#include <boost/sml.hpp>
#include <nc_task_manager/nc_task_manager.h>

using namespace NaviFra;

void TaskManager::process()
{
    while (!activity_.isStopped()) {
        try {
            processImpl();
        }
        catch (Poco::Exception& e) {
            NLOG(error) << e.displayText();
        }
        if (!activity_.isStopped())
            Poco::Thread::sleep(n_thread_period_millisec_);
    }
}

#include <typeinfo>

void TaskManager::processImpl()
{
    // 변수를 담을 구조체 생성
    // TaskPubStatus();
    // NLOG(trace) << "NcTaskManager Current Task State " << current_state;
}
