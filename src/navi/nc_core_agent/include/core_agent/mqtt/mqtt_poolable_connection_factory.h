#ifndef NAVIFRA_MQTT_OBJECT_FACTORY_H
#define NAVIFRA_MQTT_OBJECT_FACTORY_H

#include <Poco/ObjectPool.h>
#include <Poco/UUID.h>
#include <Poco/UUIDGenerator.h>
#include <mqtt/async_client.h>

namespace Poco {
class MQTTPoolableObjectFactory
    : public PoolableObjectFactory<mqtt::async_client, Poco::SharedPtr<mqtt::async_client>>
/// PoolableObjectFactory specialisation for Client. New connections
/// are created with the given address.
{
public:
    explicit MQTTPoolableObjectFactory(std::string hostname, std::string port)
        : hostname_(hostname)
        , port_(port)
    {
    }
    // Create a new Redis client object
    Poco::SharedPtr<mqtt::async_client> createObject()
    {
        mqtt::connect_options connOpts;
        connOpts.set_keep_alive_interval(20);
        // connOpts.set_clean_session(true);
        connOpts.set_automatic_reconnect(true);

        std::string serverURL = Poco::format("mqtt://%s:%s", hostname_, port_);
        Poco::SharedPtr<mqtt::async_client> client =
            new mqtt::async_client(serverURL, Poco::UUIDGenerator::defaultGenerator().create().toString());

        client->connect(connOpts)->wait();

        return client;
    }

    // Validate if the object is still valid
    bool validateObject(Poco::SharedPtr<mqtt::async_client> pObject) { return pObject->is_connected(); }

    // Activate the object (when borrowed from the pool)
    void activateObject(Poco::SharedPtr<mqtt::async_client> pObject) {}

    // Deactivate the object (when returned to the pool)
    void deactivateObject(Poco::SharedPtr<mqtt::async_client> pObject) {}

    // Destroy the object (when removed from the pool)
    void destroyObject(Poco::SharedPtr<mqtt::async_client> pObject)
    {
        if (pObject->is_connected()) {
            pObject->disconnect();
        }
        pObject = nullptr;
    }

    std::string hostname_;
    std::string port_;
};

class MQTTPooledConnection
/// Helper class for borrowing and returning a connection automatically from a pool.
{
public:
    MQTTPooledConnection(
        Poco::ObjectPool<mqtt::async_client, Poco::SharedPtr<mqtt::async_client>, MQTTPoolableObjectFactory>& pool,
        long timeoutMilliseconds = 0)
        : _pool(pool)
    {
#if POCO_VERSION >= 0x01080000
        _client = _pool.borrowObject(timeoutMilliseconds);
#else
        _client = _pool.borrowObject();
#endif
    }

    virtual ~MQTTPooledConnection()
    {
        try {
            _pool.returnObject(_client);
        }
        catch (...) {
            poco_unexpected();
        }
    }

    operator Poco::SharedPtr<mqtt::async_client>() { return _client; }

private:
    Poco::ObjectPool<mqtt::async_client, Poco::SharedPtr<mqtt::async_client>, MQTTPoolableObjectFactory>& _pool;
    Poco::SharedPtr<mqtt::async_client> _client;
};
}  // namespace Poco

#endif  // NAVIFRA_REDIS_OBJECT_FACTORY_H
