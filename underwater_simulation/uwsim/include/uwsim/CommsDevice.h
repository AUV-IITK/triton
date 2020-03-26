#ifndef CommsDevice_ECHO_H_
#define CommsDevice_ECHO_H_
#include "ConfigXMLParser.h"
#include "ROSInterface.h"
#include "SimulatedDevice.h"
#include <chrono>
#include <dccomms_ros_msgs/CheckDevice.h>
#include <dccomms_ros_msgs/RemoveDevice.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

using namespace uwsim;

class CommsDevice_Config : public SimulatedDeviceConfig {

public:
  // XML members
  std::string relativeTo, tfId, relativeTfId, dccommsId, logLevel;
  PacketBuilderConfig txPacketBuilderConfig, rxPacketBuilderConfig;
  unsigned int mac, channelId, txFifoSize, maxBackoffSlots;
  double position[3]{0, 0, 0}, orientation[3]{0, 0, 0};
  Mesh mesh;
  int disable = 0;
  std::string macProtocol;
  double macDistance;

  CommsDevice_Config(std::string type_) : SimulatedDeviceConfig(type_) {}
};

class UWSimCommsDevice : public SimulatedDevice {
public:
  osg::Node *parent;
  SimulatedIAUV *auv;
  osg::ref_ptr<osg::Node> target;
  ros::NodeHandle node;
  std::string targetTfId, tfId;
  bool render;
  dccomms::PacketBuilderPtr txPacketBuilder, rxPacketBuilder;

  UWSimCommsDevice(CommsDevice_Config *cfg, osg::ref_ptr<osg::Node> target,
                   SimulatedIAUV *auv);

  void Init(CommsDevice_Config *cfg, osg::ref_ptr<osg::Node> target,
            SimulatedIAUV *auv);

  void AddToNetSim();
  virtual CommsDevice_Config *GetConfig() = 0;
  virtual void SetConfig(CommsDevice_Config *cfg) = 0;
  void SetPacketBuilder(PACKET_TYPE pType, CommsDevice_Config *cfg,
                        PacketBuilderConfig *pbcfg);

protected:
  virtual bool _AddToNetSim() = 0;

private:
  ros::ServiceClient _checkService, _rmService;
  bool _Check();
  bool _Remove();
};

/* You will need to add your code HERE */
class CommsDevice_Factory : public SimulatedDeviceFactory {
public:
  // this is the only place the device/interface type is set
  CommsDevice_Factory(std::string type_ = "CommsDevice")
      : SimulatedDeviceFactory(type_){};
  virtual UWSimCommsDevice *Create(CommsDevice_Config *cfg,
                                   osg::ref_ptr<osg::Node> target,
                                   SimulatedIAUV *auv) = 0;

  virtual void processCommonConfig(const xmlpp::Node *node, ConfigFile *config,
                                   CommsDevice_Config *commonConfig);

  // Methods from SimulatedDevice
  virtual bool applyConfig(SimulatedIAUV *auv, Vehicle &vehicleChars,
                           SceneBuilder *sceneBuilder, size_t iteration);
  virtual std::vector<std::shared_ptr<ROSInterface>>
  getInterface(ROSInterfaceInfo &rosInterface,
               std::vector<std::shared_ptr<SimulatedIAUV>> &iauvFile);
  virtual SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node *node,
                                                   ConfigFile *config) = 0;
};

class CommsDevice_ROSPublisher : public ROSPublisherInterface {
public:
  CommsDevice_ROSPublisher(UWSimCommsDevice *dev, std::string topic, int rate)
      : ROSPublisherInterface(topic, rate), dev(dev) {}

  UWSimCommsDevice *dev;
  virtual void createPublisher(ros::NodeHandle &nh);
  virtual void publish();

  ~CommsDevice_ROSPublisher() {}

private:
  tf::TransformBroadcaster _tfBr;
};

#endif
