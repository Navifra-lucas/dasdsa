#include "nc_can_handler/Can.h"

using namespace NaviFra::CanHandler;

bool Can::sendMessage(uint32_t can_id, const std::vector<uint8_t>& data)
{
    if (socket_fd_ < 0 || data.size() > 8)
        return false;

    can_frame frame{};
    frame.can_id = can_id;
    frame.can_dlc = static_cast<uint8_t>(data.size());
    std::copy(data.begin(), data.end(), frame.data);

    return sendFrame(frame);
}

bool Can::sendFrame(const can_frame& frame)
{
    if (socket_fd_ < 0)
        return false;

    ssize_t bytes_sent = write(socket_fd_, &frame, sizeof(can_frame));
    if (bytes_sent != sizeof(can_frame))
        return false;

    std::cout << "CAN message sent: ID=0x" << std::hex << std::setw(3) << std::setfill('0') << frame.can_id << std::dec
              << ", DLC=" << static_cast<int>(frame.can_dlc) << ", Data=[";

    for (int i = 0; i < frame.can_dlc; ++i) {
        std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame.data[i]) << std::dec;
        if (i < frame.can_dlc - 1)
            std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    return true;
}
bool Can::initialize()
{
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "Error creating socket: " << strerror(errno) << std::endl;
        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, ifname_.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Error getting interface index: " << strerror(errno) << std::endl;
        close(socket_fd_);
        return false;
    }

    struct sockaddr_can addr {
    };
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "Error binding socket: " << strerror(errno) << std::endl;
        close(socket_fd_);
        return false;
    }

    std::cout << "CAN socket initialized on interface: " << ifname_ << std::endl;
    return true;
}

bool Can::start()
{
    if (socket_fd_ < 0 || running_.load())
        return false;

    running_.store(true);
    read_thread_ = std::make_unique<std::thread>(&Can::readLoop, this);
    std::cout << "CAN reader started..." << std::endl;
    return true;
}

void Can::stop()
{
    if (running_.load()) {
        running_.store(false);
        if (read_thread_ && read_thread_->joinable()) {
            read_thread_->join();
        }
        read_thread_.reset();
    }

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    std::cout << "CAN reader stopped." << std::endl;
}

void Can::readLoop()
{
    std::cout << "CAN read thread started..." << std::endl;

    while (running_.load()) {
        can_frame frame{};
        ssize_t bytes_read = read(socket_fd_, &frame, sizeof(can_frame));

        if (bytes_read != sizeof(can_frame))
            continue;

        auto it = callbacks_.find(frame.can_id);
        if (it != callbacks_.end()) {
            try {
                (*it->second)(frame);
            }
            catch (const std::exception& e) {
                std::cerr << "Error in callback for CAN ID 0x" << std::hex << std::setw(3) << std::setfill('0') << frame.can_id << std::dec
                          << ": " << e.what() << std::endl;
            }
        }
        else {
            std::cout << "Unhandled CAN message: ID=0x" << std::hex << std::setw(3) << std::setfill('0') << frame.can_id << std::dec
                      << ", DLC=" << static_cast<int>(frame.can_dlc) << std::endl;
        }
    }

    std::cout << "CAN read thread endded." << std::endl;
}

void Can::listCallbacks() const
{
    std::cout << "\n=== Registered Auto Callbacks ===" << std::endl;
    if (callbacks_.empty()) {
        std::cout << "No callbacks registered." << std::endl;
    }
    else {
        for (const auto& [can_id, wrapper] : callbacks_) {
            std::cout << "CAN ID 0x" << std::hex << std::setw(3) << std::setfill('0') << can_id << std::dec << ": "
                      << wrapper->getTypeInfo() << " (" << wrapper->getRequiredBytes() << " bytes)" << std::endl;
        }
    }
    std::cout << "==================================" << std::endl << std::endl;
}
