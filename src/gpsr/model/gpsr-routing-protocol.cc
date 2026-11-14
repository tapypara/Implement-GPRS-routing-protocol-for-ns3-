/*
 * GPSR: Greedy Perimeter Stateless Routing (Karp & Kung, MobiCom 2000)
 * Written against the ns-3.40 API.
 */
#include "gpsr-routing-protocol.h"

#include "ns3/arp-cache.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/inet-socket-address.h"
#include "ns3/llc-snap-header.h"
#include "ns3/log.h"
#include "ns3/node-list.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/simulator.h"
#include "ns3/txop.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-net-device.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

NS_LOG_COMPONENT_DEFINE("GpsrRoutingProtocol");

namespace ns3
{
namespace gpsr
{

NS_OBJECT_ENSURE_REGISTERED(RoutingProtocol);
NS_OBJECT_ENSURE_REGISTERED(HelloHeader);
NS_OBJECT_ENSURE_REGISTERED(PerimeterTag);
NS_OBJECT_ENSURE_REGISTERED(ArrivalTag);

const uint32_t RoutingProtocol::GPSR_PORT = 666;

namespace
{
const double GEO_EPS = 1e-6;

inline double
Dist(const Vector& a, const Vector& b)
{
    return CalculateDistance(a, b);
}

/**
 * Intersection of segments AB and CD. On success, `out` is the intersection point.
 */
bool
SegmentIntersection(const Vector& A, const Vector& B, const Vector& C, const Vector& D, Vector& out)
{
    double rx = B.x - A.x;
    double ry = B.y - A.y;
    double sx = D.x - C.x;
    double sy = D.y - C.y;
    double denom = rx * sy - ry * sx;
    if (std::abs(denom) < 1e-12)
    {
        return false; // parallel / collinear
    }
    double t = ((C.x - A.x) * sy - (C.y - A.y) * sx) / denom;
    double u = ((C.x - A.x) * ry - (C.y - A.y) * rx) / denom;
    if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0)
    {
        return false;
    }
    out = Vector(A.x + t * rx, A.y + t * ry, 0.0);
    return true;
}

void
WriteVec2(TagBuffer& i, const Vector& v)
{
    i.WriteDouble(v.x);
    i.WriteDouble(v.y);
}

Vector
ReadVec2(TagBuffer& i)
{
    double x = i.ReadDouble();
    double y = i.ReadDouble();
    return Vector(x, y, 0.0);
}
} // namespace

/********************************************************************
 * HelloHeader (beacon payload: 2 x float32 = 8 bytes, Sec. 2.1)
 ********************************************************************/

HelloHeader::HelloHeader(Vector pos)
    : m_pos(pos)
{
}

TypeId
HelloHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::gpsr::HelloHeader")
                            .SetParent<Header>()
                            .SetGroupName("Gpsr")
                            .AddConstructor<HelloHeader>();
    return tid;
}

TypeId
HelloHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
HelloHeader::GetSerializedSize() const
{
    return 2 * sizeof(uint32_t);
}

void
HelloHeader::Serialize(Buffer::Iterator start) const
{
    float fx = static_cast<float>(m_pos.x);
    float fy = static_cast<float>(m_pos.y);
    uint32_t ux;
    uint32_t uy;
    std::memcpy(&ux, &fx, sizeof(ux));
    std::memcpy(&uy, &fy, sizeof(uy));
    start.WriteHtonU32(ux);
    start.WriteHtonU32(uy);
}

uint32_t
HelloHeader::Deserialize(Buffer::Iterator start)
{
    uint32_t ux = start.ReadNtohU32();
    uint32_t uy = start.ReadNtohU32();
    float fx;
    float fy;
    std::memcpy(&fx, &ux, sizeof(fx));
    std::memcpy(&fy, &uy, sizeof(fy));
    m_pos = Vector(fx, fy, 0.0);
    return GetSerializedSize();
}

void
HelloHeader::Print(std::ostream& os) const
{
    os << "Pos: " << m_pos;
}

/********************************************************************
 * PerimeterTag
 ********************************************************************/

PerimeterTag::PerimeterTag()
    : m_inRec(0),
      m_recPos(Vector(0, 0, 0)),
      m_lastPos(Vector(0, 0, 0)),
      m_dstPos(Vector(0, 0, 0)),
      m_lastFacePos(Vector(0, 0, 0)),
      m_prevHop(Ipv4Address::GetZero()),
      m_e0Sender(Ipv4Address::GetZero()),
      m_e0Receiver(Ipv4Address::GetZero()),
      m_hops(0)
{
}

TypeId
PerimeterTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::gpsr::PerimeterTag")
                            .SetParent<Tag>()
                            .SetGroupName("Gpsr")
                            .AddConstructor<PerimeterTag>();
    return tid;
}

TypeId
PerimeterTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
PerimeterTag::GetSerializedSize() const
{
    return 1 + 4 * 2 * sizeof(double) + 3 * sizeof(uint32_t) + sizeof(uint16_t);
}

void
PerimeterTag::Serialize(TagBuffer i) const
{
    i.WriteU8(m_inRec);
    WriteVec2(i, m_recPos);
    WriteVec2(i, m_lastPos);
    WriteVec2(i, m_dstPos);
    WriteVec2(i, m_lastFacePos);
    i.WriteU32(m_prevHop.Get());
    i.WriteU32(m_e0Sender.Get());
    i.WriteU32(m_e0Receiver.Get());
    i.WriteU16(m_hops);
}

void
PerimeterTag::Deserialize(TagBuffer i)
{
    m_inRec = i.ReadU8();
    m_recPos = ReadVec2(i);
    m_lastPos = ReadVec2(i);
    m_dstPos = ReadVec2(i);
    m_lastFacePos = ReadVec2(i);
    m_prevHop = Ipv4Address(i.ReadU32());
    m_e0Sender = Ipv4Address(i.ReadU32());
    m_e0Receiver = Ipv4Address(i.ReadU32());
    m_hops = i.ReadU16();
}

void
PerimeterTag::Print(std::ostream& os) const
{
    os << "M=" << (m_inRec ? "perimeter" : "greedy") << " Lp=" << m_recPos << " D=" << m_dstPos
       << " Lf=" << m_lastFacePos << " e0=(" << m_e0Sender << "->" << m_e0Receiver
       << ") prev=" << m_prevHop << " hops=" << m_hops;
}

/********************************************************************
 * ArrivalTag
 ********************************************************************/

ArrivalTag::ArrivalTag() = default;

ArrivalTag::ArrivalTag(const PerimeterTag& t)
    : PerimeterTag(t)
{
}

TypeId
ArrivalTag::GetTypeId()
{
    static TypeId tid = TypeId("ns3::gpsr::ArrivalTag")
                            .SetParent<PerimeterTag>()
                            .SetGroupName("Gpsr")
                            .AddConstructor<ArrivalTag>();
    return tid;
}

TypeId
ArrivalTag::GetInstanceTypeId() const
{
    return GetTypeId();
}

/********************************************************************
 * PositionTable
 ********************************************************************/

PositionTable::PositionTable()
    : m_entryLifeTime(Seconds(6.75)) // T = 4.5 B with B = 1.5 s
{
}

void
PositionTable::AddEntry(Ipv4Address id, Vector pos)
{
    NeighborEntry e;
    e.position = pos;
    e.lastSeen = Simulator::Now();
    m_table[id] = e;
}

void
PositionTable::DeleteEntry(Ipv4Address id)
{
    m_table.erase(id);
}

void
PositionTable::Purge()
{
    for (auto it = m_table.begin(); it != m_table.end();)
    {
        if (Simulator::Now() - it->second.lastSeen > m_entryLifeTime)
        {
            it = m_table.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void
PositionTable::Clear()
{
    m_table.clear();
}

bool
PositionTable::Lookup(Ipv4Address id, Vector& pos)
{
    Purge();
    auto it = m_table.find(id);
    if (it == m_table.end())
    {
        return false;
    }
    pos = it->second.position;
    return true;
}

bool
PositionTable::IsNeighbor(Ipv4Address id)
{
    Purge();
    return m_table.find(id) != m_table.end();
}

uint32_t
PositionTable::Size()
{
    Purge();
    return m_table.size();
}

void
PositionTable::Print(std::ostream& os)
{
    Purge();
    for (const auto& kv : m_table)
    {
        os << kv.first << " @ " << kv.second.position << " (seen "
           << kv.second.lastSeen.GetSeconds() << "s)\n";
    }
}

Ipv4Address
PositionTable::BestNeighbor(Vector dstPos, Vector myPos)
{
    Purge();
    double best = Dist(myPos, dstPos); // must be strictly better than ourselves (Sec. 2.4)
    Ipv4Address bestId = Ipv4Address::GetZero();
    for (const auto& kv : m_table)
    {
        double d = Dist(kv.second.position, dstPos);
        if (d < best)
        {
            best = d;
            bestId = kv.first;
        }
    }
    return bestId; // zero => local maximum
}

bool
PositionTable::IsRngEdge(Vector myPos, Ipv4Address vId, Vector vPos)
{
    double duv = Dist(myPos, vPos);
    for (const auto& kv : m_table)
    {
        if (kv.first == vId)
        {
            continue; // w != v
        }
        double duw = Dist(myPos, kv.second.position);
        double dvw = Dist(vPos, kv.second.position);
        if (duv > std::max(duw, dvw))
        {
            return false; // witness inside the lune
        }
    }
    return true;
}

bool
PositionTable::IsGabrielGraphEdge(Vector myPos, Ipv4Address vId, Vector vPos)
{
    double duv = Dist(myPos, vPos);
    double duv2 = duv * duv;
    for (const auto& kv : m_table)
    {
        if (kv.first == vId)
        {
            continue;
        }
        double duw = Dist(myPos, kv.second.position);
        double dvw = Dist(vPos, kv.second.position);
        if (duv2 > duw * duw + dvw * dvw)
        {
            return false; // witness inside the circle with diameter uv
        }
    }
    return true;
}

bool
PositionTable::IsPlanarEdge(Vector myPos, Ipv4Address vId, Vector vPos, PlanarMode mode)
{
    return (mode == GG) ? IsGabrielGraphEdge(myPos, vId, vPos) : IsRngEdge(myPos, vId, vPos);
}

double
PositionTable::GetAngle(Vector myPos, Vector refPos, Vector neighborPos)
{
    double a0 = std::atan2(refPos.y - myPos.y, refPos.x - myPos.x);
    double a1 = std::atan2(neighborPos.y - myPos.y, neighborPos.x - myPos.x);
    double d = a1 - a0;
    while (d < 0)
    {
        d += 2 * M_PI;
    }
    while (d >= 2 * M_PI)
    {
        d -= 2 * M_PI;
    }
    if (d < 1e-9)
    {
        d = 2 * M_PI; // same direction as the reference edge: it is the last one swept
    }
    return d;
}

Ipv4Address
PositionTable::BestAngle(Vector myPos, Vector refPos, Ipv4Address exclude, PlanarMode mode)
{
    Purge();
    Ipv4Address best = Ipv4Address::GetZero();
    double minAngle = std::numeric_limits<double>::max();
    for (const auto& kv : m_table)
    {
        if (kv.first == exclude)
        {
            continue;
        }
        if (!IsPlanarEdge(myPos, kv.first, kv.second.position, mode))
        {
            continue;
        }
        double a = GetAngle(myPos, refPos, kv.second.position);
        if (a < minAngle)
        {
            minAngle = a;
            best = kv.first;
        }
    }
    if (best == Ipv4Address::GetZero() && exclude != Ipv4Address::GetZero() &&
        m_table.find(exclude) != m_table.end())
    {
        return exclude; // dead end: right-hand rule turns the packet around
    }
    return best;
}

/********************************************************************
 * RoutingProtocol
 ********************************************************************/

TypeId
RoutingProtocol::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::gpsr::RoutingProtocol")
            .SetParent<Ipv4RoutingProtocol>()
            .SetGroupName("Gpsr")
            .AddConstructor<RoutingProtocol>()
            .AddAttribute("HelloInterval",
                          "Mean beacon interval B (neighbor timeout is 4.5 B)",
                          TimeValue(Seconds(1.5)),
                          MakeTimeAccessor(&RoutingProtocol::m_helloInterval),
                          MakeTimeChecker())
            .AddAttribute("PlanarMode",
                          "Planarization for perimeter mode: 0 = RNG, 1 = GG",
                          UintegerValue(0),
                          MakeUintegerAccessor(&RoutingProtocol::m_planarMode),
                          MakeUintegerChecker<uint32_t>(0, 1))
            .AddAttribute("Promiscuous",
                          "Overhear data packets for other nodes to learn neighbor positions, "
                          "and reset the beacon timer whenever a data packet is sent (Sec. 2.1, 2.5)",
                          BooleanValue(true),
                          MakeBooleanAccessor(&RoutingProtocol::m_promiscuous),
                          MakeBooleanChecker())
            .AddTraceSource("Delivered",
                            "A data packet reached its destination: packet, hops, source, destination",
                            MakeTraceSourceAccessor(&RoutingProtocol::m_deliveredTrace),
                            "ns3::gpsr::RoutingProtocol::DeliveredTracedCallback");
    return tid;
}

RoutingProtocol::RoutingProtocol()
    : m_helloInterval(Seconds(1.5)),
      m_helloTimer(Timer::CANCEL_ON_DESTROY),
      m_ipv4(nullptr),
      m_planarMode(0),
      m_mainIface(-1),
      m_promiscuous(true),
      m_wifi(nullptr)
{
    m_uniformRandomVariable = CreateObject<UniformRandomVariable>();
}

RoutingProtocol::~RoutingProtocol()
{
}

void
RoutingProtocol::DoDispose()
{
    for (auto& kv : m_socketAddresses)
    {
        kv.first->Close();
    }
    m_socketAddresses.clear();
    m_helloTimer.Cancel();
    m_neighbors.Clear();
    m_ucb = UnicastForwardCallback();
    m_wifi = nullptr;
    m_ipv4 = nullptr;
    Ipv4RoutingProtocol::DoDispose();
}

Vector
RoutingProtocol::GetMyPosition() const
{
    Ptr<MobilityModel> mm = m_ipv4->GetObject<Node>()->GetObject<MobilityModel>();
    return mm ? mm->GetPosition() : Vector(0, 0, 0);
}

bool
RoutingProtocol::GetDestinationPosition(Ipv4Address dst, Vector& pos) const
{
    // Idealized location service (paper Sec. 3.1): the source learns D's true position.
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Ipv4> ipv4 = (*it)->GetObject<Ipv4>();
        if (!ipv4)
        {
            continue;
        }
        for (uint32_t i = 0; i < ipv4->GetNInterfaces(); ++i)
        {
            for (uint32_t j = 0; j < ipv4->GetNAddresses(i); ++j)
            {
                if (ipv4->GetAddress(i, j).GetLocal() == dst)
                {
                    Ptr<MobilityModel> mm = (*it)->GetObject<MobilityModel>();
                    if (mm)
                    {
                        pos = mm->GetPosition();
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void
RoutingProtocol::SetIpv4(Ptr<Ipv4> ipv4)
{
    NS_ASSERT(ipv4);
    NS_ASSERT(!m_ipv4);
    m_ipv4 = ipv4;
    m_neighbors.SetEntryLifeTime(Seconds(m_helloInterval.GetSeconds() * 4.5)); // T = 4.5 B
    m_helloTimer.SetFunction(&RoutingProtocol::SendHello, this);
}

void
RoutingProtocol::ScheduleNextHello()
{
    // Jitter: interval uniformly distributed in [0.5 B, 1.5 B] (mean B), Sec. 2.1
    double b = m_helloInterval.GetSeconds();
    m_helloTimer.Schedule(Seconds(m_uniformRandomVariable->GetValue(0.5 * b, 1.5 * b)));
}

namespace
{
uint64_t g_beaconTx = 0;
}

uint64_t
RoutingProtocol::GetBeaconTxCount()
{
    return g_beaconTx;
}

void
RoutingProtocol::SendHello()
{
    HelloHeader hh(GetMyPosition());
    for (const auto& kv : m_socketAddresses)
    {
        Ptr<Packet> packet = Create<Packet>();
        packet->AddHeader(hh);
        kv.first->SendTo(packet, 0, InetSocketAddress(kv.second.GetBroadcast(), GPSR_PORT));
        ++g_beaconTx;
    }
    ScheduleNextHello();
}

void
RoutingProtocol::RecvGPSR(Ptr<Socket> socket)
{
    Address src;
    Ptr<Packet> packet = socket->RecvFrom(src);
    HelloHeader hh;
    packet->RemoveHeader(hh);
    Ipv4Address sender = InetSocketAddress::ConvertFrom(src).GetIpv4();
    for (const auto& kv : m_socketAddresses)
    {
        if (kv.second.GetLocal() == sender)
        {
            return; // our own beacon
        }
    }
    m_neighbors.AddEntry(sender, hh.GetPosition());
}

void
RoutingProtocol::NotifyTxError(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu)
{
    // Sec. 2.5: MAC-layer failure feedback. Retry limit exceeded on a unicast frame means the
    // next hop has left radio range. We are inside the MAC's own callback, so do the work
    // (touching the MAC queue, re-sending) in a separate event.
    if (!m_ipv4 || reason != WIFI_MAC_DROP_REACHED_RETRY_LIMIT)
    {
        return;
    }
    const WifiMacHeader& hdr = mpdu->GetHeader();
    Mac48Address a1 = hdr.GetAddr1();
    if (!hdr.IsData() || a1.IsGroup())
    {
        return;
    }
    Simulator::ScheduleNow(&RoutingProtocol::HandleLinkFailure, this, mpdu, a1);
}

void
RoutingProtocol::DeleteNeighborsByMac(Mac48Address mac)
{
    Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol>();
    for (const auto& kv : m_socketAddresses)
    {
        int32_t idx = m_ipv4->GetInterfaceForAddress(kv.second.GetLocal());
        if (idx < 0)
        {
            continue;
        }
        Ptr<ArpCache> cache = l3->GetInterface(idx)->GetArpCache();
        if (!cache)
        {
            continue;
        }
        for (ArpCache::Entry* e : cache->LookupInverse(mac))
        {
            NS_LOG_DEBUG("MAC retry limit: removing neighbor " << e->GetIpv4Address());
            m_neighbors.DeleteEntry(e->GetIpv4Address());
        }
    }
}

void
RoutingProtocol::HandleLinkFailure(Ptr<const WifiMpdu> failed, Mac48Address receiver)
{
    if (!m_ipv4)
    {
        return;
    }
    DeleteNeighborsByMac(receiver);

    // Packets to re-forward: the one that just failed, plus every IPv4 packet still waiting in the
    // interface queue for the same receiver (otherwise each would block the interface in turn).
    std::list<Ptr<Packet>> pkts;
    pkts.push_back(failed->GetPacket()->Copy());
    if (m_wifi && m_wifi->GetMac() && m_wifi->GetMac()->GetTxop())
    {
        Ptr<WifiMacQueue> q = m_wifi->GetMac()->GetTxop()->GetWifiMacQueue();
        WifiContainerQueueId qid = WifiMacQueueContainer::GetQueueId(failed);
        Ptr<WifiMpdu> m = q->PeekByQueueId(qid);
        while (m)
        {
            Ptr<WifiMpdu> next = q->PeekByQueueId(qid, m); // compute before removing m
            Ptr<Packet> copy = m->GetPacket()->Copy();
            LlcSnapHeader llc;
            copy->PeekHeader(llc);
            if (llc.GetType() == Ipv4L3Protocol::PROT_NUMBER) // leave ARP etc. alone
            {
                pkts.push_back(m->GetPacket()->Copy());
                q->Remove(m);
            }
            m = next;
        }
    }
    for (const auto& p : pkts)
    {
        ReForward(p);
    }
}

void
RoutingProtocol::ReForward(Ptr<Packet> pkt)
{
    if (m_ucb.IsNull() || m_mainIface < 0)
    {
        return;
    }
    LlcSnapHeader llc;
    pkt->RemoveHeader(llc);
    if (llc.GetType() != Ipv4L3Protocol::PROT_NUMBER)
    {
        return;
    }
    Ipv4Header iph;
    pkt->RemoveHeader(iph);
    ArrivalTag arr;
    if (!pkt->RemovePacketTag(arr))
    {
        return; // not a GPSR data packet
    }
    PerimeterTag stale;
    pkt->RemovePacketTag(stale);

    PerimeterTag tag(arr); // state exactly as when the packet arrived at this node
    Ipv4Address nextHop;
    if (!DecideNextHop(tag, iph.GetDestination(), nextHop))
    {
        return; // no alternative next hop: drop
    }
    pkt->AddPacketTag(tag);
    pkt->AddPacketTag(arr);
    if (iph.GetTtl() < 255)
    {
        iph.SetTtl(iph.GetTtl() + 1); // IpForward will decrement it again
    }
    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(iph.GetDestination());
    route->SetGateway(nextHop);
    route->SetSource(iph.GetSource());
    route->SetOutputDevice(m_ipv4->GetNetDevice(m_mainIface));
    m_ucb(route, pkt, iph);
    ResetHelloTimer();
}

void
RoutingProtocol::PromiscRecv(Ptr<NetDevice> dev,
                             Ptr<const Packet> p,
                             uint16_t protocol,
                             const Address& from,
                             const Address& to,
                             NetDevice::PacketType type)
{
    // Only frames addressed to other nodes; frames for us are handled in RouteInput.
    if (type != NetDevice::PACKET_OTHERHOST || !m_ipv4)
    {
        return;
    }
    PerimeterTag tag;
    if (p->PeekPacketTag(tag) && tag.GetPrevHop() != Ipv4Address::GetZero())
    {
        m_neighbors.AddEntry(tag.GetPrevHop(), tag.GetLastPos());
    }
}

void
RoutingProtocol::ResetHelloTimer()
{
    // Sec. 2.1: any data packet we send also announces our position (to neighbors that overhear
    // it), so the inter-beacon timer is restarted. Only valid when overhearing is enabled.
    if (m_promiscuous && m_helloTimer.IsRunning())
    {
        m_helloTimer.Cancel();
        ScheduleNextHello();
    }
}

uint32_t
RoutingProtocol::GetNeighborCount()
{
    return m_neighbors.Size();
}

void
RoutingProtocol::TryCreateSocket(uint32_t interface)
{
    Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol>();
    if (!l3 || l3->GetNAddresses(interface) == 0 || !m_ipv4->IsUp(interface))
    {
        return;
    }
    Ipv4InterfaceAddress iface = l3->GetAddress(interface, 0);
    if (iface.GetLocal() == Ipv4Address::GetLoopback())
    {
        return;
    }
    for (const auto& kv : m_socketAddresses)
    {
        if (kv.second.GetLocal() == iface.GetLocal())
        {
            return; // already set up
        }
    }

    Ptr<Socket> socket = Socket::CreateSocket(m_ipv4->GetObject<Node>(),
                                              UdpSocketFactory::GetTypeId());
    socket->SetRecvCallback(MakeCallback(&RoutingProtocol::RecvGPSR, this));
    socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), GPSR_PORT));
    socket->BindToNetDevice(l3->GetNetDevice(interface));
    socket->SetAllowBroadcast(true);
    m_socketAddresses.insert(std::make_pair(socket, iface));

    if (m_mainIface < 0)
    {
        m_mainIface = interface;
    }

    // MAC failure feedback (ns-3.40: "DroppedMpdu"; the old "TxErrHeader" is obsolete)
    Ptr<WifiNetDevice> wifi = DynamicCast<WifiNetDevice>(l3->GetNetDevice(interface));
    if (wifi && wifi->GetMac())
    {
        wifi->GetMac()->TraceConnectWithoutContext(
            "DroppedMpdu",
            MakeCallback(&RoutingProtocol::NotifyTxError, this));
        if (static_cast<int32_t>(interface) == m_mainIface)
        {
            m_wifi = wifi;
        }
    }
    if (m_promiscuous && static_cast<int32_t>(interface) == m_mainIface)
    {
        m_ipv4->GetObject<Node>()->RegisterProtocolHandler(
            MakeCallback(&RoutingProtocol::PromiscRecv, this),
            Ipv4L3Protocol::PROT_NUMBER,
            l3->GetNetDevice(interface),
            true);
    }

    if (!m_helloTimer.IsRunning())
    {
        ScheduleNextHello();
    }
}

void
RoutingProtocol::NotifyInterfaceUp(uint32_t interface)
{
    TryCreateSocket(interface);
}

void
RoutingProtocol::NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address)
{
    TryCreateSocket(interface);
}

void
RoutingProtocol::NotifyInterfaceDown(uint32_t interface)
{
    Ptr<Ipv4L3Protocol> l3 = m_ipv4->GetObject<Ipv4L3Protocol>();
    if (l3->GetNAddresses(interface) == 0)
    {
        return;
    }
    Ipv4Address local = l3->GetAddress(interface, 0).GetLocal();
    for (auto it = m_socketAddresses.begin(); it != m_socketAddresses.end(); ++it)
    {
        if (it->second.GetLocal() == local)
        {
            it->first->Close();
            m_socketAddresses.erase(it);
            break;
        }
    }
    if (static_cast<int32_t>(interface) == m_mainIface)
    {
        m_mainIface = m_socketAddresses.empty()
                          ? -1
                          : m_ipv4->GetInterfaceForAddress(
                                m_socketAddresses.begin()->second.GetLocal());
    }
    if (m_socketAddresses.empty())
    {
        m_helloTimer.Cancel();
        m_neighbors.Clear();
    }
}

void
RoutingProtocol::NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address)
{
}

void
RoutingProtocol::PrintRoutingTable(Ptr<OutputStreamWrapper> stream, Time::Unit unit) const
{
    std::ostream* os = stream->GetStream();
    *os << "GPSR neighbor table at t=" << Simulator::Now().As(unit) << "\n";
    const_cast<PositionTable&>(m_neighbors).Print(*os);
}

/**
 * Per-hop decision, paper Sec. 2.4.
 *
 *  greedy  : next hop = neighbor closest to D, if closer than this node.
 *            None => local maximum => enter perimeter mode: Lp = here, Lf = here,
 *            first edge = first planar edge counterclockwise from line (here -> D),
 *            e0 = that edge.
 *  perimeter: if here is closer to D than Lp => back to greedy.
 *            Otherwise right-hand rule from the edge to the previous hop.
 *            - candidate edge == e0  => second traversal of the first edge: D unreachable, drop.
 *            - candidate edge crosses segment Lp-D at a point y closer to D than Lf:
 *              Lf = y and move to the next face: send along the next edge counterclockwise
 *              from the candidate edge, and record that edge as the new e0.
 */
bool
RoutingProtocol::DecideNextHop(PerimeterTag& tag, Ipv4Address dst, Ipv4Address& nextHop)
{
    const Vector myPos = GetMyPosition();
    const Vector dstPos = tag.GetDstPos();
    const Ipv4Address myIp = m_ipv4->GetAddress(m_mainIface, 0).GetLocal();
    const Ipv4Address zero = Ipv4Address::GetZero();
    const PlanarMode mode = static_cast<PlanarMode>(m_planarMode);

    m_neighbors.Purge();
    nextHop = zero;

    if (m_neighbors.IsNeighbor(dst))
    {
        // D is within radio range: deliver directly (D's stamped position may be slightly stale).
        tag.SetInRec(0);
        nextHop = dst;
    }
    else
    {
        // Perimeter -> greedy as soon as we are closer to D than the point Lp where greedy failed.
        if (tag.GetInRec() == 1 && Dist(myPos, dstPos) < Dist(tag.GetRecPos(), dstPos))
        {
            tag.SetInRec(0);
            NS_LOG_DEBUG(myIp << " leaves perimeter mode for " << dst);
        }

        if (tag.GetInRec() == 0)
        {
            nextHop = m_neighbors.BestNeighbor(dstPos, myPos);
            if (nextHop == zero)
            {
                // Local maximum: enter perimeter mode.
                nextHop = m_neighbors.BestAngle(myPos, dstPos, zero, mode);
                if (nextHop == zero)
                {
                    return false; // isolated
                }
                tag.SetInRec(1);
                tag.SetRecPos(myPos);      // Lp
                tag.SetLastFacePos(myPos); // Lf
                tag.SetE0(myIp, nextHop);  // e0
                NS_LOG_DEBUG(myIp << " enters perimeter mode for " << dst);
            }
        }
        else
        {
            // Perimeter mode: right-hand rule about the edge to the previous hop.
            Ipv4Address prev = tag.GetPrevHop();
            Vector ref = tag.GetLastPos();
            Vector fresh;
            if (m_neighbors.Lookup(prev, fresh))
            {
                ref = fresh;
            }
            nextHop = m_neighbors.BestAngle(myPos, ref, prev, mode);
            if (nextHop == zero)
            {
                return false;
            }

            // Second traversal of the first edge on this face => D is unreachable.
            if (tag.GetE0Sender() == myIp && tag.GetE0Receiver() == nextHop)
            {
                NS_LOG_DEBUG(myIp << ": e0 repeated, " << dst << " unreachable");
                return false;
            }

            // Face change.
            const Vector lp = tag.GetRecPos();
            const uint32_t maxIter = m_neighbors.Size() + 1;
            for (uint32_t g = 0; g < maxIter; ++g)
            {
                Vector nPos;
                Vector y;
                if (!m_neighbors.Lookup(nextHop, nPos) ||
                    !SegmentIntersection(myPos, nPos, lp, dstPos, y) ||
                    !(Dist(y, dstPos) < Dist(tag.GetLastFacePos(), dstPos) - GEO_EPS))
                {
                    break;
                }
                tag.SetLastFacePos(y); // Lf = y
                Ipv4Address alt = m_neighbors.BestAngle(myPos, nPos, nextHop, mode);
                if (alt == zero || alt == nextHop)
                {
                    tag.SetE0(myIp, nextHop);
                    break;
                }
                nextHop = alt; // first edge of the next face
                tag.SetE0(myIp, nextHop);
                NS_LOG_DEBUG(myIp << " changes face, next hop " << nextHop);
            }
        }
    }

    if (nextHop == zero)
    {
        return false;
    }

    // Stamp ourselves as previous hop: used by the next node's right-hand rule and
    // to refresh our entry in its neighbor table (position piggybacking, Sec. 2.1).
    tag.SetPrevHop(myIp);
    tag.SetLastPos(myPos);
    tag.SetHops(tag.GetHops() + 1);
    return true;
}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput(Ptr<Packet> p,
                             const Ipv4Header& header,
                             Ptr<NetDevice> oif,
                             Socket::SocketErrno& sockerr)
{
    Ipv4Address dst = header.GetDestination();
    if (!p || !m_ipv4 || m_mainIface < 0)
    {
        sockerr = Socket::ERROR_NOROUTETOHOST;
        return nullptr;
    }
    const Ipv4Address myIp = m_ipv4->GetAddress(m_mainIface, 0).GetLocal();

    // Normally Ipv4L3Protocol handles broadcasts itself; be defensive.
    if (dst.IsBroadcast() ||
        dst == m_ipv4->GetAddress(m_mainIface, 0).GetBroadcast())
    {
        Ptr<Ipv4Route> r = Create<Ipv4Route>();
        r->SetDestination(dst);
        r->SetGateway(dst);
        r->SetSource(myIp);
        r->SetOutputDevice(m_ipv4->GetNetDevice(m_mainIface));
        sockerr = Socket::ERROR_NOTERROR;
        return r;
    }
    if (dst.IsMulticast())
    {
        sockerr = Socket::ERROR_NOROUTETOHOST;
        return nullptr;
    }
    if (dst == myIp)
    {
        Ptr<Ipv4Route> r = Create<Ipv4Route>();
        r->SetDestination(dst);
        r->SetGateway(Ipv4Address::GetLoopback());
        r->SetSource(myIp);
        r->SetOutputDevice(m_ipv4->GetNetDevice(0));
        sockerr = Socket::ERROR_NOTERROR;
        return r;
    }

    // The source stamps D's location in the packet; it is never changed afterwards.
    Vector dstPos;
    if (!GetDestinationPosition(dst, dstPos))
    {
        sockerr = Socket::ERROR_NOROUTETOHOST;
        return nullptr;
    }

    PerimeterTag tag; // mode = greedy
    tag.SetDstPos(dstPos);
    const ArrivalTag arrival(tag); // state before this node's decision (for re-forwarding)
    Ipv4Address nextHop;
    if (!DecideNextHop(tag, dst, nextHop))
    {
        sockerr = Socket::ERROR_NOROUTETOHOST;
        return nullptr;
    }

    PerimeterTag old;
    p->RemovePacketTag(old);
    ArrivalTag oldArrival;
    p->RemovePacketTag(oldArrival);
    p->AddPacketTag(tag);
    p->AddPacketTag(arrival);
    ResetHelloTimer();

    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);
    route->SetGateway(nextHop);
    route->SetSource(myIp);
    route->SetOutputDevice(m_ipv4->GetNetDevice(m_mainIface));
    sockerr = Socket::ERROR_NOTERROR;
    return route;
}

bool
RoutingProtocol::RouteInput(Ptr<const Packet> p,
                            const Ipv4Header& header,
                            Ptr<const NetDevice> idev,
                            const UnicastForwardCallback& ucb,
                            const MulticastForwardCallback& mcb,
                            const LocalDeliverCallback& lcb,
                            const ErrorCallback& ecb)
{
    if (!m_ipv4 || m_mainIface < 0)
    {
        return false;
    }
    if (m_ucb.IsNull())
    {
        m_ucb = ucb; // L3's forwarding callback is the same for every call; keep it for re-forwarding
    }
    const Ipv4Address dst = header.GetDestination();
    const int32_t iif = m_ipv4->GetInterfaceForDevice(idev);

    // Position piggybacking: every GPSR data packet carries the previous hop's position.
    PerimeterTag seen;
    const bool hasTag = p->PeekPacketTag(seen);
    if (hasTag && seen.GetPrevHop() != Ipv4Address::GetZero() &&
        !m_ipv4->IsDestinationAddress(seen.GetPrevHop(), iif))
    {
        m_neighbors.AddEntry(seen.GetPrevHop(), seen.GetLastPos());
    }

    // Local delivery (unicast to us, or broadcast such as beacons).
    if (m_ipv4->IsDestinationAddress(dst, iif))
    {
        if (hasTag && !dst.IsBroadcast() && !dst.IsMulticast())
        {
            m_deliveredTrace(p, seen.GetHops(), header.GetSource(), dst);
        }
        if (!lcb.IsNull())
        {
            lcb(p, header, iif);
        }
        return true;
    }
    if (dst.IsMulticast())
    {
        return false;
    }

    // Forward.
    Ptr<Packet> packet = p->Copy();
    PerimeterTag tag;
    if (!packet->RemovePacketTag(tag))
    {
        Vector d;
        if (!GetDestinationPosition(dst, d))
        {
            ecb(p, header, Socket::ERROR_NOROUTETOHOST);
            return true;
        }
        tag.SetDstPos(d);
    }
    ArrivalTag staleArrival;
    packet->RemovePacketTag(staleArrival);
    const ArrivalTag arrival(tag); // state as it arrived here (for re-forwarding after a MAC failure)

    Ipv4Address nextHop;
    if (!DecideNextHop(tag, dst, nextHop))
    {
        NS_LOG_DEBUG("Drop packet for " << dst);
        ecb(p, header, Socket::ERROR_NOROUTETOHOST);
        return true;
    }
    packet->AddPacketTag(tag);
    packet->AddPacketTag(arrival);
    ResetHelloTimer();

    Ptr<Ipv4Route> route = Create<Ipv4Route>();
    route->SetDestination(dst);
    route->SetGateway(nextHop);
    route->SetSource(header.GetSource());
    route->SetOutputDevice(m_ipv4->GetNetDevice(m_mainIface));
    ucb(route, packet, header);
    return true;
}

} // namespace gpsr
} // namespace ns3
