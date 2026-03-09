#ifndef NAVIFRA_MEMORY_REPOSITORY_H
#define NAVIFRA_MEMORY_REPOSITORY_H

#include <Poco/SingletonHolder.h>
#include <boost/any.hpp>

#include <map>
#include <memory>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <typeindex>
namespace NaviFra {
class InMemoryRepository {
public:
    static InMemoryRepository& instance()
    {
        static Poco::SingletonHolder<InMemoryRepository> sh;
        return *sh.get();
    }

    template <typename T>
    std::shared_ptr<T> get(std::string key)
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        auto it = entities_.find(typeid(T));
        if (it != entities_.end()) {
            auto entityIt = it->second.find(key);
            if (entityIt != it->second.end()) {
                return boost::any_cast<std::shared_ptr<T>>(entityIt->second);
            }
        }

        auto newEntity = std::make_shared<T>();
        entities_[typeid(T)][key] = newEntity;
        return newEntity;
    }

    template <typename T>
    void add(std::string key, std::shared_ptr<T> entity)
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        entities_[typeid(T)][key] = entity;
    }

    template <typename T>
    void update(std::string key, std::shared_ptr<T> entity)
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        auto it = entities_.find(typeid(T));
        if (it != entities_.end()) {
            it->second[key] = entity;
        }
        else {
            throw std::runtime_error("Entity type not found");
        }
    }

    template <typename T>
    void remove(std::string key)
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        auto it = entities_.find(typeid(T));
        if (it != entities_.end()) {
            it->second.erase(key);
        }
    }

    template <typename T>
    bool exists(std::string key)
    {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        auto it = entities_.find(typeid(T));
        if (it != entities_.end()) {
            return it->second.find(key) != it->second.end();
        }
        return false;
    }

    ~InMemoryRepository() { clear(); }

    void clear()
    {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        entities_.clear();
    }

private:
    std::map<std::type_index, std::map<std::string, boost::any>> entities_;
    std::shared_mutex mutex_;
};
}  // namespace NaviFra

#endif
