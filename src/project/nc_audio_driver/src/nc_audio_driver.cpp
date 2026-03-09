#include "nc_audio_driver.hpp"
using namespace NaviFra;

AudioDriver::AudioDriver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
    : nh_(nh)
    , nhp_(nhp)
{
    NLOG(info)<<"AudioDriver Create";

    gst_init(nullptr, nullptr);
    pipeline_ = nullptr;

    RegistListener();
}

AudioDriver::~AudioDriver()
{
    b_terminate_ = true;
    NLOG(info)<<"destructor";
}

void AudioDriver::RegistListener()
{
    sub_audio_ = nh_.subscribe("/navifra/sound", 5, &AudioDriver::SubscribeAudio, this);
}

void AudioDriver::play(const std::string& filepath) {
    stop();  

    std::string uri = "/home/navifra/navifra_solution/navicore/sounds/" + filepath + ".mp3";

    // 재생 가능한 장치 후보 목록
    std::vector<std::string> device_candidates = {
        "plughw:0", "plughw:1", "plughw:2", "default"
    };

    for (const auto& device : device_candidates) {
        GError* error = nullptr;

        std::string pipeline_str = "filesrc location=" + uri +
            " ! decodebin ! audioconvert ! audioresample ! alsasink device=" + device;

        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

        if (!pipeline_) {
            if (error && error->message) {
                g_printerr("GStreamer pipeline error on device %s: %s\n", device.c_str(), error->message);
                NLOG(warning) << "GStreamer pipeline error on device " << device << ": " << error->message;
            } else {
                g_printerr("GStreamer pipeline error on device %s: unknown error\n", device.c_str());
                NLOG(warning) << "GStreamer pipeline error on device " << device << ": unknown error";
            }
            g_clear_error(&error);
            continue;
        }

        GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            g_printerr("Failed to play on device %s\n", device.c_str());
            NLOG(warning) << "Failed to set pipeline to PLAYING on device " << device;
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(GST_OBJECT(pipeline_));
            pipeline_ = nullptr;
            continue;
        }

        NLOG(info) << "Playing " << filepath << " on device " << device;
        return;  // 성공한 경우 바로 종료
    }

    NLOG(error) << "Audio playback failed on all known devices.";
}


void AudioDriver::stop() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(GST_OBJECT(pipeline_));
        pipeline_ = nullptr;
        current_file_.clear();
    }
}

void AudioDriver::SubscribeAudio(const std_msgs::String::ConstPtr& msg)
{
    NLOG(info) << "[AudioDriver] Subcribe Audio : " << msg->data.c_str();
    std::string cmd = msg->data;
    play(msg->data);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nc_audio_driver");

    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    AudioDriver nc_audio_driver(nh, nhp);
    ros::spin();

    return 0;
}