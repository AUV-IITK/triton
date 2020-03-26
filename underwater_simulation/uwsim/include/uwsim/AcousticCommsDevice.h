#ifndef AcousticCommsDevice_ECHO_H_
#define AcousticCommsDevice_ECHO_H_
#include "ConfigXMLParser.h"
#include "ROSInterface.h"
#include "SimulatedDevice.h"
#include <dccomms_ros_msgs/AddAcousticDevice.h>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/LinkDeviceToChannel.h>
#include <dccomms_ros_msgs/RemoveDevice.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <uwsim/CommsDevice.h>

using namespace uwsim;

class AcousticCommsDevice_Config : public CommsDevice_Config {

public:
  // XML members
  double range, pT, frequency, L, K, turnOnEnergy, turnOffEnergy, preamble,
      pTConsume, pRConsume, pIdle, codingEff, bitErrorRate, energyLevel;
  uint32_t symbolsPerSecond;
  // constructor
  AcousticCommsDevice_Config(std::string type_)
      : CommsDevice_Config(type_), range(100), pT(0.2818), frequency(25),
        L(0.0), K(2.0), turnOnEnergy(0.0), turnOffEnergy(0.0), preamble(0.0),
        pTConsume(0.660), pRConsume(0.395), pIdle(0.0) {}
};

class AcousticCommsDevice : public UWSimCommsDevice {
public:
  AcousticCommsDevice_Config *config;
  AcousticCommsDevice(AcousticCommsDevice_Config *cfg,
                      osg::ref_ptr<osg::Node> target, SimulatedIAUV *auv);
  void AddToNetSim();
  static uint32_t nDevsReady;
  static uint32_t nDevs;
  void SetConfig(CommsDevice_Config *cfg);
  CommsDevice_Config *GetConfig();

protected:
  bool _AddToNetSim();

private:
  ros::ServiceClient _addService, _linkToChannelService;
};

/* You will need to add your code HERE */
class AcousticCommsDevice_Factory : public CommsDevice_Factory {
public:
  // this is the only place the device/interface type is set
  AcousticCommsDevice_Factory(std::string type_ = "AcousticCommsDevice")
      : CommsDevice_Factory(type_){};
  UWSimCommsDevice *Create(CommsDevice_Config *cfg,
                           osg::ref_ptr<osg::Node> target, SimulatedIAUV *auv);
  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node *node,
                                           ConfigFile *config);
};

class AcousticCommsDevice_ROSPublisher : public CommsDevice_ROSPublisher {
public:
  AcousticCommsDevice_ROSPublisher(AcousticCommsDevice *dev, std::string topic,
                                   int rate)
      : CommsDevice_ROSPublisher(dev, topic, rate) {}

  ~AcousticCommsDevice_ROSPublisher() {}
};

#endif
