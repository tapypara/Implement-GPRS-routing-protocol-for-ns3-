/*
 * GPSR: Greedy Perimeter Stateless Routing (Karp & Kung, MobiCom 2000)
 * Written against the ns-3.40 API.
 */
#ifndef GPSR_ROUTING_PROTOCOL_H
#define GPSR_ROUTING_PROTOCOL_H

#include "ns3/header.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"
#include "ns3/tag.h"
#include "ns3/timer.h"
#include "ns3/vector.h"
#include "ns3/traced-callback.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/wifi-net-device.h"

#include <map>
#include <ostream>

namespace ns3
{
namespace gpsr
{

/** Planarization used for perimeter forwarding (paper Sec. 2.3). */
enum PlanarMode
{
    RNG = 0, //!< Relative Neighborhood Graph (the one used in the paper's results)
    GG = 1   //!< Gabriel Graph
};

/**
 * Beacon: sender id is the IP source; payload is position as two 4-byte
 * floats (paper Sec. 2.1) => 8 bytes.
 */
class HelloHeader : public Header
{
  public:
    HelloHeader(Vector pos = Vector(0, 0, 0));
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    Vector GetPosition() const
    {
        return m_pos;
    }

    void SetPosition(Vector pos)
    {
        m_pos = pos;
    }

  private:
    Vector m_pos;
};

/**
 * Per-packet GPSR state (paper Table 1): D, Lp, Lf, e0, M.
 * Additionally carries the previous hop (id + position), needed by the
 * right-hand rule and used to learn neighbor positions (piggybacking).
 */
class PerimeterTag : public Tag
{
  public:
    PerimeterTag();
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
    uint32_t GetSerializedSize() const override;
    void Serialize(TagBuffer i) const override;
    void Deserialize(TagBuffer i) override;
    void Print(std::ostream& os) const override;

    uint8_t GetInRec() const { return m_inRec; }
    void SetInRec(uint8_t v) { m_inRec = v; }
    Vector GetRecPos() const { return m_recPos; }       //!< Lp
    void SetRecPos(Vector v) { m_recPos = v; }
    Vector GetDstPos() const { return m_dstPos; }       //!< D
    void SetDstPos(Vector v) { m_dstPos = v; }
    Vector GetLastFacePos() const { return m_lastFacePos; } //!< Lf
    void SetLastFacePos(Vector v) { m_lastFacePos = v; }
    Vector GetLastPos() const { return m_lastPos; }     //!< position of previous hop
    void SetLastPos(Vector v) { m_lastPos = v; }
    Ipv4Address GetPrevHop() const { return m_prevHop; }
    void SetPrevHop(Ipv4Address a) { m_prevHop = a; }
    Ipv4Address GetE0Sender() const { return m_e0Sender; }
    Ipv4Address GetE0Receiver() const { return m_e0Receiver; }
    uint16_t GetHops() const { return m_hops; }         //!< hops travelled so far
    void SetHops(uint16_t v) { m_hops = v; }

    void SetE0(Ipv4Address s, Ipv4Address r)
    {
        m_e0Sender = s;
        m_e0Receiver = r;
    }

  private:
    uint8_t m_inRec;        //!< M: 0 greedy, 1 perimeter
    Vector m_recPos;        //!< Lp
    Vector m_lastPos;       //!< previous hop position
    Vector m_dstPos;        //!< D
    Vector m_lastFacePos;   //!< Lf
    Ipv4Address m_prevHop;  //!< previous hop id
    Ipv4Address m_e0Sender; //!< e0
    Ipv4Address m_e0Receiver;
    uint16_t m_hops;        //!< number of hops travelled (statistics only)
};

/**
 * Copy of the PerimeterTag as it was when the packet ARRIVED at this node (before this
 * node's forwarding decision changed it). Needed to re-forward a packet after a MAC
 * failure (Sec. 2.5) exactly as if it had just arrived.
 */
class ArrivalTag : public PerimeterTag
{
  public:
    ArrivalTag();
    explicit ArrivalTag(const PerimeterTag& t);
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;
};

struct NeighborEntry
{
    Vector position;
    Time lastSeen;
};

/** One-hop neighbor table with RNG / GG planarization on demand. */
class PositionTable
{
  public:
    PositionTable();
    void AddEntry(Ipv4Address id, Vector pos);
    void DeleteEntry(Ipv4Address id);
    bool Lookup(Ipv4Address id, Vector& pos);
    bool IsNeighbor(Ipv4Address id);
    uint32_t Size();
    void Purge();
    void Clear();
    void SetEntryLifeTime(Time t) { m_entryLifeTime = t; }
    void Print(std::ostream& os);

    /** Greedy: neighbor strictly closer to dstPos than myPos and closest overall; zero if none. */
    Ipv4Address BestNeighbor(Vector dstPos, Vector myPos);

    /**
     * Right-hand rule on the planarized graph: first planar edge counterclockwise
     * about myPos starting from the direction myPos->refPos. The neighbor `exclude`
     * (previous hop) is skipped, but is returned if it is the only planar edge left
     * (dead end: the packet goes back). Pass zero for `exclude` when entering perimeter mode.
     */
    Ipv4Address BestAngle(Vector myPos, Vector refPos, Ipv4Address exclude, PlanarMode mode);

    bool IsPlanarEdge(Vector myPos, Ipv4Address vId, Vector vPos, PlanarMode mode);
    bool IsRngEdge(Vector myPos, Ipv4Address vId, Vector vPos);
    bool IsGabrielGraphEdge(Vector myPos, Ipv4Address vId, Vector vPos);

    /** Counterclockwise angle in (0, 2*pi] from direction me->ref to direction me->nbr. */
    static double GetAngle(Vector myPos, Vector refPos, Vector neighborPos);

  private:
    std::map<Ipv4Address, NeighborEntry> m_table;
    Time m_entryLifeTime;
};

class RoutingProtocol : public Ipv4RoutingProtocol
{
  public:
    static TypeId GetTypeId();
    static const uint32_t GPSR_PORT;

    RoutingProtocol();
    ~RoutingProtocol() override;

    Ptr<Ipv4Route> RouteOutput(Ptr<Packet> p,
                               const Ipv4Header& header,
                               Ptr<NetDevice> oif,
                               Socket::SocketErrno& sockerr) override;
    bool RouteInput(Ptr<const Packet> p,
                    const Ipv4Header& header,
                    Ptr<const NetDevice> idev,
                    const UnicastForwardCallback& ucb,
                    const MulticastForwardCallback& mcb,
                    const LocalDeliverCallback& lcb,
                    const ErrorCallback& ecb) override;
    void NotifyInterfaceUp(uint32_t interface) override;
    void NotifyInterfaceDown(uint32_t interface) override;
    void NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address) override;
    void NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address) override;
    void SetIpv4(Ptr<Ipv4> ipv4) override;
    void PrintRoutingTable(Ptr<OutputStreamWrapper> stream,
                           Time::Unit unit = Time::S) const override;

    void RecvGPSR(Ptr<Socket> socket);

    /** Number of live entries in the one-hop neighbor table (paper Sec. 3.6: state per node). */
    uint32_t GetNeighborCount();

    /** Total beacon packets sent network-wide since program start (paper Fig. 10 overhead). */
    static uint64_t GetBeaconTxCount();

  protected:
    void DoDispose() override;

  private:
    void SendHello();
    void ScheduleNextHello();
    void TryCreateSocket(uint32_t interface);
    void NotifyTxError(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu);
    /** Sec. 2.5: drop the failed neighbor, empty the interface queue of its packets, re-forward. */
    void HandleLinkFailure(Ptr<const WifiMpdu> failed, Mac48Address receiver);
    void DeleteNeighborsByMac(Mac48Address mac);
    void ReForward(Ptr<Packet> llcPacket);
    /** Sec. 2.1/2.5: overhear data packets addressed to other nodes (promiscuous mode). */
    void PromiscRecv(Ptr<NetDevice> dev,
                     Ptr<const Packet> p,
                     uint16_t protocol,
                     const Address& from,
                     const Address& to,
                     NetDevice::PacketType type);
    void ResetHelloTimer();

    Vector GetMyPosition() const;
    bool GetDestinationPosition(Ipv4Address dst, Vector& pos) const;

    /**
     * Core of GPSR (paper Sec. 2.4). Given the packet state `tag`, decide the next hop at
     * this node, updating the tag (mode switches, Lp, Lf, e0, previous hop).
     * \return false if the packet must be dropped.
     */
    bool DecideNextHop(PerimeterTag& tag, Ipv4Address dst, Ipv4Address& nextHop);

    Time m_helloInterval; //!< B
    Timer m_helloTimer;
    PositionTable m_neighbors;
    Ptr<Ipv4> m_ipv4;
    std::map<Ptr<Socket>, Ipv4InterfaceAddress> m_socketAddresses;
    Ptr<UniformRandomVariable> m_uniformRandomVariable;
    uint32_t m_planarMode;
    int32_t m_mainIface; //!< the (single) wireless interface used for forwarding
    bool m_promiscuous;  //!< overhear data packets + reset beacon timer when sending data
    Ptr<WifiNetDevice> m_wifi;
    UnicastForwardCallback m_ucb; //!< cached from RouteInput, used for re-forwarding
    /// Fired when a data packet reaches its destination: packet, hops travelled, src, dst.
    TracedCallback<Ptr<const Packet>, uint32_t, Ipv4Address, Ipv4Address> m_deliveredTrace;
};

} // namespace gpsr
} // namespace ns3

#endif /* GPSR_ROUTING_PROTOCOL_H */
