#include "core_agent/core_agent.h"

#include <core_agent/data/robot_wheel_info.h>

using namespace NaviFra;

RobotWheel::RobotWheel(std::string id, Position position, float diameter)
    : degree_(0.0)
    , id_(id)
    , diameter_(diameter)
    , position_(position)
{
}

RobotWheel::~RobotWheel()
{
}

void RobotWheel::updateDegree(float deg)
{
    degree_ = deg;
}

Poco::JSON::Object RobotWheel::toObject()
{
    Poco::JSON::Object obj;
    Poco::JSON::Object position;
    obj.set("diameter", diameter_);
    position.set("x", position_.x);
    position.set("y", position_.y);
    position.set("z", 0);
    obj.set("position", position);
    obj.set("deg", degree_);

    return obj;
}

const std::string RobotWheelInfoStore::KEY = "RobotWheel";

RobotWheelInfoStore::RobotWheelInfoStore()
{
    try {
        int type = -1;
        ros::param::get("motion_base/n_kinematics_type", type);
        type_ = (ROBOT_TYPE)type;
        switch (type_) {
            case ROBOT_TYPE::ROBOT_TYPE_DD:
                initDD();
                break;
            case ROBOT_TYPE::ROBOT_TYPE_QD:
                initQD();
                break;
            case ROBOT_TYPE::ROBOT_TYPE_SD:
                initSD();
                break;
            case ROBOT_TYPE::ROBOT_TYPE_QD_OCT:
                initQDOCT();
                break;
        }
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

RobotWheelInfoStore::RobotWheelInfoStore(const RobotWheelInfoStore& store)
    : type_(store.type_)
    , b_init_(store.b_init_)
    , wheels_(store.wheels_)
{
}

RobotWheelInfoStore::~RobotWheelInfoStore()
{
    wheels_.clear();
}

void RobotWheelInfoStore::updateWheelInfo(core_msgs::MotorInfo::ConstPtr msg)
{
    try {
        std::lock_guard<std::mutex> lock(mutex_);

        int type = -1;
        WheelParams wheel_param;
        ros::param::get("motion_base/n_kinematics_type", type);
        ros::param::get("driver_wheel/f_FL_wheel_x_m", wheel_param_.FL_x);
        ros::param::get("driver_wheel/f_FR_wheel_x_m", wheel_param_.FR_x);
        ros::param::get("driver_wheel/f_RL_wheel_x_m", wheel_param_.RL_x);
        ros::param::get("driver_wheel/f_RR_wheel_x_m", wheel_param_.RR_x);
        ros::param::get("driver_wheel/f_FL_wheel_y_m", wheel_param_.FL_y);
        ros::param::get("driver_wheel/f_FR_wheel_y_m", wheel_param_.FR_y);
        ros::param::get("driver_wheel/f_RL_wheel_y_m", wheel_param_.RL_y);
        ros::param::get("driver_wheel/f_RR_wheel_y_m", wheel_param_.RR_y);
        if (type_ != type || wheel_param_ != wheel_param) {
            wheels_.clear();
            b_init_ = false;
        }
        if (!b_init_) {
            b_init_ = true;
            type_ = (ROBOT_TYPE)type;
            wheel_param_ = wheel_param;
            switch (type_) {
                case ROBOT_TYPE::ROBOT_TYPE_DD:
                    initDD();
                    break;
                case ROBOT_TYPE::ROBOT_TYPE_QD:
                    initQD();
                    break;
                case ROBOT_TYPE::ROBOT_TYPE_SD:
                    initSD();
                    break;
                case ROBOT_TYPE::ROBOT_TYPE_QD_OCT:
                    initQDOCT();
                    break;
            }
        }
        switch (type_) {
            case ROBOT_TYPE::ROBOT_TYPE_DD:
                updateDD(msg);
                break;
            case ROBOT_TYPE::ROBOT_TYPE_QD:
                updateQD(msg);
                break;
            case ROBOT_TYPE::ROBOT_TYPE_SD:
                updateSD(msg);
                break;
            case ROBOT_TYPE::ROBOT_TYPE_QD_OCT:
                updateQDOCT(msg);
                break;
        }
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotWheelInfoStore::initDD(int mode)
{
    try {
        float wheelbase_width = 0.f, leftY = 0.f, rightY = 0.f, leftdiameter = 0.f, rightdiameter = 0.f;

        ros::param::get("driver_wheel/f_FL_wheel_y_m", leftY);
        ros::param::get("driver_wheel/f_RR_wheel_y_m", rightY);
        ros::param::get("driver_wheel/f_FL_wheel_diameter_m", leftdiameter);
        ros::param::get("driver_wheel/f_RR_wheel_diameter_m", rightdiameter);
        // wheels_.insert(std::make_pair<int, RobotWheel>(
        //     static_cast<int>(DRIVE_DD::DRIVE_DD_LEFT), RobotWheel("left", Position{0, leftY}, leftdiameter)));
        // wheels_.insert(std::make_pair<int, RobotWheel>(
        //     static_cast<int>(DRIVE_DD::DRIVE_DD_RIGHT), RobotWheel("right", Position{0, rightY}, rightdiameter)));

        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_FL), RobotWheel("fl", Position{0, leftY}, leftdiameter)));
        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_FR), RobotWheel("fr", Position{0, rightY}, leftdiameter)));
        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_RL), RobotWheel("rl", Position{0, leftY}, rightdiameter)));
        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_RR), RobotWheel("rr", Position{0, rightY}, rightdiameter)));
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotWheelInfoStore::initQD(int mode)
{
    try {
        float leftX = 0.f, leftY = 0.f, rightX = 0.f, rightY = 0.f, diameterLeft = 0.f, diameterRight = 0.f;

        ros::param::get("driver_wheel/f_FL_wheel_x_m", leftX);
        ros::param::get("driver_wheel/f_FL_wheel_y_m", leftY);
        ros::param::get("driver_wheel/f_RR_wheel_x_m", rightX);
        ros::param::get("driver_wheel/f_RR_wheel_y_m", rightY);
        ros::param::get("driver_wheel/f_FL_wheel_diameter_m", diameterLeft);
        ros::param::get("driver_wheel/f_RR_wheel_diameter_m", diameterRight);
        // wheels_.insert(std::make_pair<int, RobotWheel>(
        //     static_cast<int>(DRIVE_QD::DRIVE_QD_LEFT), RobotWheel("left", Position{leftX, leftY}, diameterLeft)));
        // wheels_.insert(std::make_pair<int, RobotWheel>(
        //     static_cast<int>(DRIVE_QD::DRIVE_QD_RIGHT), RobotWheel("right", Position{rightX, rightY}, diameterRight)));

        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_FL), RobotWheel("fl", Position{leftX, leftY}, diameterLeft)));
        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_FR), RobotWheel("fr", Position{rightX, rightY}, diameterLeft)));
        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_RL), RobotWheel("rl", Position{leftX, leftY}, diameterRight)));
        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_RR), RobotWheel("rr", Position{rightX, rightY}, diameterRight)));
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotWheelInfoStore::initSD(int mode)
{
    try {
        float x = 0.f, y = 0.f, diameter = 0.f;
        ros::param::get("driver_wheel/f_FL_wheel_x_m", x);
        ros::param::get("driver_wheel/f_FL_wheel_y_m", y);
        ros::param::get("driver_wheel/f_FL_wheel_diameter_m", diameter);

        // wheels_.insert(
        //     std::make_pair<int, RobotWheel>(static_cast<int>(DRIVE_SD::DRIVE_SD_FRONT), RobotWheel("front", Position{x, y}, diameter)));

        wheels_.insert(
            std::make_pair<int, RobotWheel>(static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_FL), RobotWheel("fl", Position{x, y}, diameter)));
        wheels_.insert(
            std::make_pair<int, RobotWheel>(static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_FR), RobotWheel("fr", Position{x, y}, diameter)));
        wheels_.insert(
            std::make_pair<int, RobotWheel>(static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_RL), RobotWheel("rl", Position{x, y}, diameter)));
        wheels_.insert(
            std::make_pair<int, RobotWheel>(static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_RR), RobotWheel("rr", Position{x, y}, diameter)));
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotWheelInfoStore::initQDOCT(int mode)
{
    try {
        float frontLeftX = 0.f, frontLeftY = 0.f, frontRightX = 0.f, frontRightY = 0.f, frontDiameterLeft = 0.f, frontDiameterRight = 0.f,
              rearLeftX = 0.f, rearLeftY = 0.f, rearRightX = 0.f, rearRightY = 0.f, rearDiameterLeft = 0.f, rearDiameterRight = 0.f;

        ros::param::get("driver_wheel/f_FL_wheel_x_m", frontLeftX);
        ros::param::get("driver_wheel/f_FL_wheel_y_m", frontLeftY);
        ros::param::get("driver_wheel/f_FR_wheel_x_m", frontRightX);
        ros::param::get("driver_wheel/f_FR_wheel_y_m", frontRightY);
        ros::param::get("driver_wheel/f_FL_wheel_diameter_m", frontDiameterLeft);
        ros::param::get("driver_wheel/f_FR_wheel_diameter_m", frontDiameterRight);

        ros::param::get("driver_wheel/f_RL_wheel_x_m", rearLeftX);
        ros::param::get("driver_wheel/f_RL_wheel_y_m", rearLeftY);
        ros::param::get("driver_wheel/f_RR_wheel_x_m", rearRightX);
        ros::param::get("driver_wheel/f_RR_wheel_y_m", rearRightY);
        ros::param::get("driver_wheel/f_RL_wheel_diameter_m", rearDiameterLeft);
        ros::param::get("driver_wheel/f_RR_wheel_diameter_m", rearDiameterRight);

        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_FL), RobotWheel("fl", Position{frontLeftX, frontLeftY}, frontDiameterLeft)));
        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_FR), RobotWheel("fr", Position{frontRightX, frontRightY}, frontDiameterRight)));
        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_RL), RobotWheel("rl", Position{rearLeftX, rearLeftY}, rearDiameterLeft)));
        wheels_.insert(std::make_pair<int, RobotWheel>(
            static_cast<int>(DRIVE_WHEEL::DRIVE_WHEEL_RR), RobotWheel("rr", Position{rearRightX, rearRightY}, rearDiameterRight)));
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotWheelInfoStore::updateDD(core_msgs::MotorInfo::ConstPtr msg)
{
    try {
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_FL).updateDegree(0);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_FR).updateDegree(0);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_RL).updateDegree(0);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_RR).updateDegree(0);
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotWheelInfoStore::updateQD(core_msgs::MotorInfo::ConstPtr msg)
{
    try {
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_FL).updateDegree(msg->f_FL_steer_motor_feedback_deg);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_FR).updateDegree(msg->f_RR_steer_motor_feedback_deg);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_RL).updateDegree(msg->f_FL_steer_motor_feedback_deg);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_RR).updateDegree(msg->f_RR_steer_motor_feedback_deg);
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotWheelInfoStore::updateSD(core_msgs::MotorInfo::ConstPtr msg)
{
    try {
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_FL).updateDegree(msg->f_FL_steer_motor_feedback_deg);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_FR).updateDegree(msg->f_FL_steer_motor_feedback_deg);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_RL).updateDegree(msg->f_FL_steer_motor_feedback_deg);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_RR).updateDegree(msg->f_FL_steer_motor_feedback_deg);
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

void RobotWheelInfoStore::updateQDOCT(core_msgs::MotorInfo::ConstPtr msg)
{
    try {
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_FL).updateDegree(msg->f_FL_steer_motor_feedback_deg);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_FR).updateDegree(msg->f_FR_steer_motor_feedback_deg);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_RL).updateDegree(msg->f_RL_steer_motor_feedback_deg);
        wheels_.at(DRIVE_WHEEL::DRIVE_WHEEL_RR).updateDegree(msg->f_RR_steer_motor_feedback_deg);
    }
    catch (Poco::Exception& ex) {
        NLOG(error) << ex.displayText();
    }
}

Poco::JSON::Object RobotWheelInfoStore::toObject()
{
    Poco::JSON::Object robotWheels;
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& pair : wheels_) {
        robotWheels.set(pair.second.name(), pair.second.toObject());
    }

    return robotWheels;
}
