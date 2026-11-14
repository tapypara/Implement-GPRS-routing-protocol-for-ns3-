#ifndef GPSR_HELPER_H
#define GPSR_HELPER_H

#include "ns3/ipv4-routing-helper.h"
#include "ns3/object-factory.h"
#include "ns3/node-container.h"
#include "ns3/node.h"

namespace ns3 {

/**
 * \ingroup gpsr
 * \brief Helper class that adds GPSR routing to nodes
 */
class GpsrHelper : public Ipv4RoutingHelper
{
public:
  GpsrHelper ();
  GpsrHelper (const GpsrHelper &o);
  GpsrHelper* Copy (void) const;

  virtual Ptr<Ipv4RoutingProtocol> Create (Ptr<Node> node) const;
  void Set (std::string name, const AttributeValue &value);

private:
  ObjectFactory m_agentFactory;
};

} // namespace ns3

#endif /* GPSR_HELPER_H */
