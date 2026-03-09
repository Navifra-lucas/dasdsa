#ifndef NAVIFRA_CONFIG_H
#define NAVIFRA_CONFIG_H

#include <Poco/SingletonHolder.h>
#include <Poco/Util/LayeredConfiguration.h>
#include <Poco/Util/PropertyFileConfiguration.h>

namespace NaviFra {
class Config : public Poco::Util::MapConfiguration {
public:
    Config();
    ~Config() = default;

    static Config& instance()
    {
        static Poco::SingletonHolder<Config> sh;
        return *sh.get();
    }
};

}  // namespace NaviFra

#endif  // NAVIFRA_CONFIG_H