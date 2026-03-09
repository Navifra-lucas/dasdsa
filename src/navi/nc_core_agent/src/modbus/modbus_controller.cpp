#include "core_agent/core_agent.h"

#include <core_agent/modbus/modbus_controller.h>

using namespace NaviFra;

ModbusController::ModbusController(
    boost::asio::io_context& io_context, PortType portType, const std::string& hostOrDevice, const std::string& portOrBaudrate)
    : io_context_(io_context)
    , port_type_(portType)
    , work_guard_(boost::asio::make_work_guard(io_context_))
{
    if (portType == PortType::TCP) {
        port_ = std::make_unique<TCPPort>(io_context_, hostOrDevice, portOrBaudrate);
    }
    else if (portType == PortType::RTU) {
        port_ = std::make_unique<RTUPort>(io_context_, hostOrDevice, std::stoi(portOrBaudrate));
    }
    port_->connect();
    io_thread_ = std::thread([this]() { io_context_.run(); });
}

ModbusController::~ModbusController()
{
    if (port_ && port_->isOpen()) {
        port_->disconnect();
    }
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

std::vector<bool> ModbusController::ReadCoils(uint8_t slaveID, uint16_t startingAddress, uint16_t nOfCoils)
{
    if (!port_ || !port_->isOpen()) {
        throw std::runtime_error("Port is not open");
    }

    std::lock_guard<std::mutex> lock(request_mutex_);

    std::promise<std::vector<bool>> promise;
    auto future = promise.get_future();

    ReadCoilsRequest request(slaveID, startingAddress, nOfCoils);
    sendRequest<ReadCoilsResponse>(request, [&promise](const ReadCoilsResponse& response) { promise.set_value(response.getCoilStatus()); });

    auto result = future.get();

    if (result.size() > nOfCoils) {
        result = std::vector<bool>(result.begin(), result.begin() + nOfCoils);
    }
    return result;
}

void ModbusController::WriteCoil(uint8_t slaveID, uint16_t coilAddress, bool value)
{
    if (!port_ || !port_->isOpen()) {
        throw std::runtime_error("Port is not open");
    }
    std::lock_guard<std::mutex> lock(request_mutex_);

    std::promise<void> promise;
    auto future = promise.get_future();

    // Write Single Coil 요청 생성
    WriteSingleCoilRequest request(slaveID, coilAddress, value);

    sendRequest<WriteSingleCoilResponse>(request, [&promise, coilAddress, value](const WriteSingleCoilResponse& response) {
        if (response.getAddress() == coilAddress && response.getValue() == value) {
            promise.set_value();  // 성공 시 완료
        }
        else {
            promise.set_exception(std::make_exception_ptr(std::runtime_error("Write coil response verification failed")));
        }
    });

    future.get();  // 완료될 때까지 대기
}

void ModbusController::WriteMultipleCoils(uint8_t slaveID, uint16_t startingAddress, const std::vector<bool>& values)
{
    if (!port_ || !port_->isOpen()) {
        throw std::runtime_error("Port is not open");
    }

    std::lock_guard<std::mutex> lock(request_mutex_);

    std::promise<void> promise;
    auto future = promise.get_future();

    // Write Multiple Coils 요청 생성
    WriteMultipleCoilsRequest request(slaveID, startingAddress, values);

    sendRequest<WriteMultipleCoilsResponse>(request, [&promise, startingAddress, values](const WriteMultipleCoilsResponse& response) {
        if (response.getStartingAddress() == startingAddress && response.getQuantityOfCoils() == values.size()) {
            promise.set_value();  // 성공 시 완료
        }
        else {
            promise.set_exception(std::make_exception_ptr(std::runtime_error("Write multiple coils response verification failed")));
        }
    });

    future.get();  // 완료될 때까지 대기
}

std::vector<int16_t> ModbusController::ReadHoldingRegisters(uint8_t slaveID, uint16_t startingAddress, uint16_t nOfHoldingRegister)
{
    std::lock_guard<std::mutex> lock(request_mutex_);
    // ReadHoldingRegistersRequest 생성
    ReadHoldingRegistersRequest request(slaveID, startingAddress, nOfHoldingRegister);

    // 비동기 응답을 기다리기 위한 Promise와 Future 설정
    std::promise<ReadHoldingRegistersResponse> promise;
    auto future = promise.get_future();

    // 요청을 보내고, 응답을 처리하는 람다 콜백
    sendRequest<ReadHoldingRegistersResponse>(
        request, [&promise](const ReadHoldingRegistersResponse& response) { promise.set_value(response); });

    // Future에서 응답을 기다림
    ReadHoldingRegistersResponse response = future.get();

    // 응답에서 레지스터 값을 가져와 int16_t 벡터로 변환
    const std::vector<uint16_t>& registerValues = response.getRegisterValues();
    std::vector<int16_t> result;
    result.reserve(registerValues.size());
    for (uint16_t value : registerValues) {
        result.push_back(static_cast<int16_t>(value));
    }

    return result;
}

std::vector<int16_t> ModbusController::ReadInputRegisters(uint8_t slaveID, uint16_t startingAddress, uint16_t nOfInputRegisters)
{
    std::lock_guard<std::mutex> lock(request_mutex_);
    // 1. ReadInputRegistersRequest 생성
    ReadInputRegistersRequest request(slaveID, startingAddress, nOfInputRegisters);

    // 2. 비동기 응답을 기다리기 위한 Promise와 Future 설정
    std::promise<ReadInputRegistersResponse> promise;
    auto future = promise.get_future();

    // 3. 요청을 보내고, 응답을 처리하는 람다 콜백
    sendRequest<ReadInputRegistersResponse>(
        request, [&promise](const ReadInputRegistersResponse& response) { promise.set_value(response); });

    // 4. Future에서 응답을 기다림
    ReadInputRegistersResponse response = future.get();

    // 5. 응답에서 레지스터 값을 가져와 int16_t 벡터로 변환
    const std::vector<uint16_t>& registerValues = response.getRegisterValues();
    std::vector<int16_t> result;
    result.reserve(registerValues.size());
    for (uint16_t value : registerValues) {
        result.push_back(static_cast<int16_t>(value));
    }

    return result;
}

void ModbusController::WriteSingleRegister(uint8_t slaveID, uint16_t address, uint16_t value)
{
    std::lock_guard<std::mutex> lock(request_mutex_);
    // WriteSingleRegisterRequest 생성
    WriteSingleRegisterRequest request(slaveID, address, value);

    // 비동기 응답을 기다리기 위한 Promise와 Future 설정
    std::promise<WriteSingleRegisterResponse> promise;
    auto future = promise.get_future();

    // 요청을 보내고, 응답을 처리하는 람다 콜백
    sendRequest<WriteSingleRegisterResponse>(
        request, [&promise](const WriteSingleRegisterResponse& response) { promise.set_value(response); });

    // Future에서 응답을 기다림
    WriteSingleRegisterResponse response = future.get();

    // 응답에서 주소와 값을 가져와 출력하거나 로깅
    if (response.getAddress() != address || response.getValue() != value) {
        throw std::runtime_error("Write verification failed: Address or value mismatch");
    }
}

void ModbusController::WriteMultipleRegisters(uint8_t slaveID, uint16_t startingAddress, const std::vector<uint16_t>& values)
{
    std::lock_guard<std::mutex> lock(request_mutex_);
    WriteMultipleRegistersRequest request(slaveID, startingAddress, values);

    std::promise<WriteMultipleRegistersResponse> promise;
    auto future = promise.get_future();

    sendRequest<WriteMultipleRegistersResponse>(
        request, [&promise](const WriteMultipleRegistersResponse& response) { promise.set_value(response); });

    WriteMultipleRegistersResponse response = future.get();

    if (response.getStartingAddress() != startingAddress || response.getQuantityOfRegisters() != values.size()) {
        throw std::runtime_error("Write Multiple Registers failed to write the correct number of registers.");
    }
}

std::vector<bool> ModbusController::ReadDiscreteInputs(uint8_t slaveID, uint16_t startingAddress, uint16_t nOfInputs)
{
    std::lock_guard<std::mutex> lock(request_mutex_);
    ReadDiscreteInputsRequest request(slaveID, startingAddress, nOfInputs);

    std::promise<std::vector<bool>> promise;
    auto future = promise.get_future();

    sendRequest<ReadDiscreteInputsResponse>(
        request, [&promise](const ReadDiscreteInputsResponse& response) { promise.set_value(response.getInputStatus()); });

    auto response = future.get();
    if (response.size() > nOfInputs) {
        response.resize(nOfInputs);
    }
    return response;
}
