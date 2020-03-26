#ifndef UWSIM_NETSIM_
#define UWSIM_NETSIM_

#include <dccomms/DataLinkFrame.h>
#include <dccomms_ros/simulator/ROSCommsSimulator.h>
#include <class_loader/multi_library_class_loader.hpp>

namespace uwsim {
using namespace dccomms_ros;

class NetSimTracing : public cpplogging::Logger {
public:
  NetSimTracing();
  void Run();
  virtual void Configure();
  virtual void DoRun();
};

typedef std::shared_ptr<NetSimTracing> NetSimTracingPtr;
class NetSim {
public:
  static ns3::Ptr<ROSCommsSimulator> GetSim();
  static void LoadDefaultTracingScript();
  static void LoadTracingScript(const std::string &className,
                                const std::string &libPath);
  static NetSimTracingPtr GetScript();
private:
  static NetSimTracingPtr _script;
  static std::shared_ptr<class_loader::ClassLoader> _loader;

};

}

#endif
