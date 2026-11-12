#include "gpsr-helper.h"
#include "../model/gpsr-routing-protocol.h"

namespace ns3 {

GpsrHelper::GpsrHelper ()
{
  m_agentFactory.SetTypeId ("ns3::gpsr::RoutingProtocol");
}

GpsrHelper::GpsrHelper (const GpsrHelper &o)
  : m_agentFactory (o.m_agentFactory)
{
}

GpsrHelper*
GpsrHelper::Copy (void) const
{
  return new GpsrHelper (*this);
}

Ptr<Ipv4RoutingProtocol>
GpsrHelper::Create (Ptr<Node> node) const
{
  Ptr<gpsr::RoutingProtocol> agent = m_agentFactory.Create<gpsr::RoutingProtocol> ();
  node->AggregateObject (agent);
  return agent;
}

void
GpsrHelper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}

} // namespace ns3
