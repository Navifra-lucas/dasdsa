#ifndef NAVIFRA_REDIS_OBJECT_FACTORY_H
#define NAVIFRA_REDIS_OBJECT_FACTORY_H

#include <Poco/Net/SocketAddress.h>
#include <Poco/ObjectPool.h>
#include <Poco/Redis/Client.h>
#include <Poco/Version.h>

namespace Poco {

class RedisPoolableObjectFactory
    : public Poco::PoolableObjectFactory<Poco::Redis::Client, Poco::Redis::Client::Ptr>
/// PoolableObjectFactory specialisation for Client. New connections
/// are created with the given address.
{
public:
    explicit RedisPoolableObjectFactory(Poco::Net::SocketAddress& address)
        : _address(address)
    {
    }

    RedisPoolableObjectFactory(const std::string& address)
        : _address(address)
    {
    }

    // Create a new Redis client object
    Poco::Redis::Client::Ptr createObject()
    {
        Poco::SharedPtr<Poco::Redis::Client> client = new Poco::Redis::Client();
        Poco::Timespan timeout(5, 0);  // 5초 타임아웃
        client->connect(_address, timeout);
        client->setReceiveTimeout(timeout);
        return client;
    }

    // Validate if the object is still valid
    bool validateObject(Poco::Redis::Client::Ptr pObject) { return pObject->isConnected(); }

    // Activate the object (when borrowed from the pool)
    void activateObject(Poco::Redis::Client::Ptr pObject) {}

    // Deactivate the object (when returned to the pool)
    void deactivateObject(Poco::Redis::Client::Ptr pObject) {}

    // Destroy the object (when removed from the pool)
    void destroyObject(Poco::Redis::Client::Ptr pObject)
    {
        if (pObject->isConnected()) {
            pObject->disconnect();
        }
        pObject = nullptr;
    }

private:
    Poco::Net::SocketAddress _address;
};

class RedisPooledConnection
/// Helper class for borrowing and returning a connection automatically from a pool.
{
public:
    RedisPooledConnection(
        Poco::ObjectPool<Poco::Redis::Client, Poco::Redis::Client::Ptr, RedisPoolableObjectFactory>& pool, long timeoutMilliseconds = 0)
        : _pool(pool)
    {
#if POCO_VERSION >= 0x01080000
        _client = _pool.borrowObject(timeoutMilliseconds);
#else
        _client = _pool.borrowObject();
#endif
    }

    virtual ~RedisPooledConnection()
    {
        try {
            _pool.returnObject(_client);
        }
        catch (...) {
            poco_unexpected();
        }
    }

    operator Poco::Redis::Client::Ptr() { return _client; }

private:
    Poco::ObjectPool<Poco::Redis::Client, Poco::Redis::Client::Ptr, RedisPoolableObjectFactory>& _pool;
    Poco::Redis::Client::Ptr _client;
};
}  // namespace Poco

#endif  // NAVIFRA_REDIS_OBJECT_FACTORY_H
