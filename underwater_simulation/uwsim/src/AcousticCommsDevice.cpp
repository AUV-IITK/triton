#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <uwsim/AcousticCommsDevice.h>
#include <uwsim/SimulatedIAUV.h>

/* You will need to add your code HERE */
#include <tf/transform_broadcaster.h>
#include <thread>
#include <uwsim/UWSimUtils.h>

uint32_t AcousticCommsDevice::nDevsReady = 0;
uint32_t AcousticCommsDevice::nDevs = 0;

SimulatedDeviceConfig::Ptr
AcousticCommsDevice_Factory::processConfig(const xmlpp::Node *node,
                                           ConfigFile *config) {
  AcousticCommsDevice_Config *cfg = new AcousticCommsDevice_Config(getType());
  processCommonConfig(node, config, cfg);
  xmlpp::Node::NodeList list = node->get_children();

  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end();
       ++iter) {

    const xmlpp::Node *child = dynamic_cast<const xmlpp::Node *>(*iter);
    if (child->get_name() == "range")
      config->extractFloatChar(child, cfg->range);
    else if (child->get_name() == "frequency")
      config->extractFloatChar(child, cfg->frequency);
    else if (child->get_name() == "L")
      config->extractFloatChar(child, cfg->L);
    else if (child->get_name() == "K")
      config->extractFloatChar(child, cfg->K);
    else if (child->get_name() == "preamble")
      config->extractFloatChar(child, cfg->preamble);
    else if (child->get_name() == "macProtocol")
      config->extractStringChar(child, cfg->macProtocol);
    else if (child->get_name() == "symbolsPerSecond")
      config->extractUIntChar(child, cfg->symbolsPerSecond);
    else if (child->get_name() == "codingEff")
      config->extractFloatChar(child, cfg->codingEff);
    else if (child->get_name() == "bitErrorRate")
      config->extractFloatChar(child, cfg->bitErrorRate);
    else if (child->get_name() == "energyModel") {
      xmlpp::Node::NodeList emAttributes = child->get_children();
      for (xmlpp::Node::NodeList::iterator subiter = emAttributes.begin();
           subiter != emAttributes.end(); ++subiter) {
        const xmlpp::Node *ema = dynamic_cast<const xmlpp::Node *>(*subiter);
        if (ema->get_name() == "turnOnEnergy")
          config->extractFloatChar(ema, cfg->turnOnEnergy);
        else if (ema->get_name() == "turnOffEnergy")
          config->extractFloatChar(ema, cfg->turnOffEnergy);
        else if (ema->get_name() == "pTConsume")
          config->extractFloatChar(ema, cfg->pTConsume);
        else if (ema->get_name() == "pRConsume")
          config->extractFloatChar(ema, cfg->pRConsume);
        else if (ema->get_name() == "pIdle")
          config->extractFloatChar(ema, cfg->pIdle);
        else if (ema->get_name() == "energyLevel")
          config->extractFloatChar(ema, cfg->energyLevel);
        else if (ema->get_name() == "pT")
          config->extractFloatChar(ema, cfg->pT);
      }
    }
  }
  return SimulatedDeviceConfig::Ptr(cfg);
}

bool AcousticCommsDevice::_AddToNetSim() {
  auto netsim = NetSim::GetSim();

  dccomms_ros_msgs::AddAcousticDeviceRequest srv;

  srv.frameId = this->config->tfId;
  srv.dccommsId = this->config->dccommsId;
  srv.mac = this->config->mac;
  srv.maxTxFifoSize = this->config->txFifoSize;
  srv.frequency = this->config->frequency;
  srv.K = this->config->K;
  srv.L = this->config->L;
  srv.range = this->config->range;
  srv.PT = this->config->pT;
  srv.turnOnEnergy = this->config->turnOnEnergy;
  srv.turnOffEnergy = this->config->turnOffEnergy;
  srv.preamble = this->config->preamble;
  srv.PTConsume = this->config->pTConsume;
  srv.PRConsume = this->config->pRConsume;
  srv.batteryEnergy = this->config->energyLevel;
  srv.PIdle = this->config->pIdle;
  srv.bitErrorRate = this->config->bitErrorRate;
  srv.symbolsPerSecond = this->config->symbolsPerSecond;
  srv.codingEff = this->config->codingEff;
  srv.logLevel = this->config->logLevel;

  srv.macProtocol = this->config->macProtocol;
  srv.macDistance = this->config->macDistance;

  ROS_INFO("AcousticCommsDevice  ID = %s ; Frame = %s", srv.dccommsId.c_str(),
           srv.frameId.c_str());

  netsim->AddAcousticDevice(srv);

  ROS_INFO("AcousticCommsDevice '%s' added", srv.dccommsId.c_str());

  // link dev to channel
  dccomms_ros_msgs::LinkDeviceToChannelRequest ldchSrv;
  ldchSrv.dccommsId = this->config->dccommsId;
  ldchSrv.channelId = this->config->channelId;

  netsim->LinkDevToChannel(ldchSrv);

  ROS_INFO("comms dev linked to channel");
}
void AcousticCommsDevice::SetConfig(CommsDevice_Config *cfg) {
  config = dynamic_cast<AcousticCommsDevice_Config *>(cfg);
}

CommsDevice_Config *AcousticCommsDevice::GetConfig() { return config; }

AcousticCommsDevice::AcousticCommsDevice(AcousticCommsDevice_Config *cfg,
                                         osg::ref_ptr<osg::Node> target,
                                         SimulatedIAUV *auv)
    : UWSimCommsDevice(cfg, target, auv) {
  Init(cfg, target, auv);
}

UWSimCommsDevice *
AcousticCommsDevice_Factory::Create(CommsDevice_Config *cfg,
                                    osg::ref_ptr<osg::Node> target,
                                    SimulatedIAUV *auv) {
  AcousticCommsDevice_Config *config =
      dynamic_cast<AcousticCommsDevice_Config *>(cfg);
  return new AcousticCommsDevice(config, target, auv);
}

PLUGINLIB_EXPORT_CLASS(AcousticCommsDevice_Factory,
                       uwsim::SimulatedDeviceFactory)
