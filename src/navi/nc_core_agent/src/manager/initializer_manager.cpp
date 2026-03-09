#include "core_agent/manager/initializer_manager.h"

#include "util/logger.hpp"

namespace NaviFra {

void InitializerManager::registerInitializer(Initializer* init)
{
    if (!init)
        return;
    std::lock_guard<std::mutex> lock(mtx_);
    if (ran_) {
        LOG_WARNING("[InitializerManager] register after run() ignored");
        return;
    }
    initializers_.push_back(init);
}

void InitializerManager::run()
{
    bool expected = false;
    if (!ran_.compare_exchange_strong(expected, true)) {
        LOG_INFO("[InitializerManager] run() already executed");
        return;
    }

    std::vector<Initializer*> sorted;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        sorted = initializers_;
    }

    std::sort(sorted.begin(), sorted.end(), [](Initializer* a, Initializer* b) { return a->priority() < b->priority(); });

    init_order_.clear();
    init_order_.reserve(sorted.size());

    for (auto* init : sorted) {
        try {
            init->initialize();
            init_order_.push_back(init);
            LOG_INFO("[InitializerManager] initialized priority=%d", init->priority());
        }
        catch (const std::exception& e) {
            LOG_ERROR("[InitializerManager] init failed: %s", e.what());
        }
    }
}

void InitializerManager::finalize()
{
    bool expected = false;
    if (!finalized_.compare_exchange_strong(expected, true)) {
        LOG_INFO("[InitializerManager] finalize() already executed");
        return;
    }

    std::vector<Initializer*> order;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        order = init_order_;
        if (order.empty()) {
            order = initializers_;
            std::sort(order.begin(), order.end(), [](Initializer* a, Initializer* b) { return a->priority() < b->priority(); });
        }
    }

    for (auto it = order.rbegin(); it != order.rend(); ++it) {
        if (!*it)
            continue;
        try {
            (*it)->finalize();
            LOG_INFO("[InitializerManager] finalized priority=%d", (*it)->priority());
        }
        catch (const std::exception& e) {
            LOG_WARNING("[InitializerManager] finalize failed: %s", e.what());
        }
    }

    init_order_.clear();
}

}  // namespace NaviFra
