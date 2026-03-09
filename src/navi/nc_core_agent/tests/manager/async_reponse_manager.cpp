#include "core_agent/manager/async_reponse_manager.h"

#include <gtest/gtest.h>

#include <thread>

using namespace NaviFra;

// 테스트용 데이터 타입 정의
struct can_frame {
    int id;
    std::string data;

    // 비교 연산자를 정의하여 can_frame 객체 간의 비교를 지원
    bool operator==(const can_frame& other) const { return id == other.id && data == other.data; }
};

// 사용자 정의 구조체
struct CustomData {
    int value;
    std::string name;

    // 비교 연산자를 정의하여 CustomData 객체 간의 비교를 지원
    bool operator==(const CustomData& other) const { return value == other.value && name == other.name; }
};

// Test Fixture 클래스 정의
class AsyncResponseManagerTest : public ::testing::Test {
protected:
    AsyncResponseManager& manager = AsyncResponseManager::instance();

    void SetUp() override
    {
        // 각 테스트 전에 초기화가 필요하면 여기에 작성
    }

    void TearDown() override {}
};

// 1. int 타입 ID에 대한 can_frame 타입의 데이터 처리 테스트
TEST_F(AsyncResponseManagerTest, CreatePromiseAndWaitForResponse_IntID)
{
    IDType id = 100;
    manager.createPromise<can_frame>(id);

    std::thread responder([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 지연 후 응답 설정
        can_frame frame{100, "Test CAN Data"};
        manager.setPromiseValue<can_frame>(id, frame);
    });

    can_frame expected_frame{100, "Test CAN Data"};
    try {
        can_frame received_frame = manager.waitForResponse<can_frame>(id, 1000);  // 1000ms 대기
        EXPECT_EQ(received_frame, expected_frame);  // 받은 값이 기대 값과 같은지 확인
    }
    catch (const std::exception& e) {
        FAIL() << "Exception thrown: " << e.what();
    }

    responder.join();  // 스레드 종료 대기
}

// 2. string 타입 ID에 대한 std::string 타입 데이터 처리 테스트
TEST_F(AsyncResponseManagerTest, CreatePromiseAndWaitForResponse_StringData)
{
    IDType id = std::string("TestString");
    manager.createPromise<std::string>(id);

    std::thread responder([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 지연 후 응답 설정
        manager.setPromiseValue<std::string>(id, "This is a test string");
    });

    std::string expected_data = "This is a test string";
    try {
        std::string received_data = manager.waitForResponse<std::string>(id, 1000);  // 1000ms 대기
        EXPECT_EQ(received_data, expected_data);  // 받은 값이 기대 값과 같은지 확인
    }
    catch (const std::exception& e) {
        FAIL() << "Exception thrown: " << e.what();
    }

    responder.join();  // 스레드 종료 대기
}

// 3. 사용자 정의 구조체를 사용하는 테스트
TEST_F(AsyncResponseManagerTest, CreatePromiseAndWaitForResponse_CustomData)
{
    IDType id = 200;
    manager.createPromise<CustomData>(id);

    std::thread responder([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 지연 후 응답 설정
        CustomData data{42, "Custom Object"};
        manager.setPromiseValue<CustomData>(id, data);
    });

    CustomData expected_data{42, "Custom Object"};
    try {
        CustomData received_data = manager.waitForResponse<CustomData>(id, 1000);  // 1000ms 대기
        EXPECT_EQ(received_data, expected_data);  // 받은 값이 기대 값과 같은지 확인
    }
    catch (const std::exception& e) {
        FAIL() << "Exception thrown: " << e.what();
    }

    responder.join();  // 스레드 종료 대기
}

// 4. int 타입 데이터를 사용하는 테스트
TEST_F(AsyncResponseManagerTest, CreatePromiseAndWaitForResponse_IntData)
{
    IDType id = std::string("IntData");
    manager.createPromise<int>(id);

    std::thread responder([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 지연 후 응답 설정
        manager.setPromiseValue<int>(id, 12345);
    });

    int expected_data = 12345;
    try {
        int received_data = manager.waitForResponse<int>(id, 1000);  // 1000ms 대기
        EXPECT_EQ(received_data, expected_data);  // 받은 값이 기대 값과 같은지 확인
    }
    catch (const std::exception& e) {
        FAIL() << "Exception thrown: " << e.what();
    }

    responder.join();  // 스레드 종료 대기
}

// 5. string 타입 ID에 대한 CustomData 타입 데이터 처리 테스트
TEST_F(AsyncResponseManagerTest, CreatePromiseAndWaitForResponse_StringID_CustomData)
{
    IDType id = std::string("CustomDataID");
    manager.createPromise<CustomData>(id);

    std::thread responder([&]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 지연 후 응답 설정
        CustomData data{256, "CustomData with String ID"};
        manager.setPromiseValue<CustomData>(id, data);
    });

    CustomData expected_data{256, "CustomData with String ID"};
    try {
        CustomData received_data = manager.waitForResponse<CustomData>(id, 1000);  // 1000ms 대기
        EXPECT_EQ(received_data, expected_data);  // 받은 값이 기대 값과 같은지 확인
    }
    catch (const std::exception& e) {
        FAIL() << "Exception thrown: " << e.what();
    }

    responder.join();  // 스레드 종료 대기
}
