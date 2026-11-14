/*
 * Kiểm thử chế độ perimeter của GPSR trên topo tĩnh có "hố".
 *
 *            C(400,300) ---- E(600,300)
 *           /                          \
 *     B(200,220)                     F(800,220)
 *        |                                |
 *  S(0,0) - A(200,0)      (hố)       G(800,0) - D(1000,0)
 *
 * Tầm sóng 250 m. A gần D hơn mọi neighbor của nó (S, B) nên greedy dừng ở A
 * (cực đại cục bộ). Gói chỉ tới được D nếu chế độ perimeter đi vòng qua
 * B-C-E-F-G. Đường ngắn nhất: S-A-B-C-E-F-G-D = 7 chặng.
 *
 * Chạy: ./ns3 run "scratch/gpsr-void-test --planarMode=0"   (RNG)
 *       ./ns3 run "scratch/gpsr-void-test --planarMode=1"   (GG)
 * Kết quả mong đợi: nhận đủ số gói gửi, mỗi gói đi 7 chặng.
 */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/gpsr-helper.h"
#include "ns3/gpsr-routing-protocol.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/neighbor-cache-helper.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <iostream>
#include <map>

using namespace ns3;

static std::map<uint32_t, uint32_t> g_hopCount; // số chặng -> số gói

static void
OnDelivered(Ptr<const Packet>, uint32_t hops, Ipv4Address, Ipv4Address)
{
    g_hopCount[hops]++;
}

int
main(int argc, char* argv[])
{
    uint32_t planarMode = 0;
    double simTime = 60.0;
    CommandLine cmd;
    cmd.AddValue("planarMode", "0 = RNG, 1 = GG", planarMode);
    cmd.AddValue("simTime", "Thời gian mô phỏng (s)", simTime);
    cmd.Parse(argc, argv);

    // S, A, B, C, E, F, G, D
    std::vector<Vector> pos = {{0, 0, 0},
                               {200, 0, 0},
                               {200, 220, 0},
                               {400, 300, 0},
                               {600, 300, 0},
                               {800, 220, 0},
                               {800, 0, 0},
                               {1000, 0, 0}};
    NodeContainer nodes;
    nodes.Create(pos.size());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("DsssRate2Mbps"),
                                 "ControlMode",
                                 StringValue("DsssRate1Mbps"));
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(250.0));
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
    for (const auto& p : pos)
    {
        alloc->Add(p);
    }
    mobility.SetPositionAllocator(alloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    GpsrHelper gpsr;
    gpsr.Set("PlanarMode", UintegerValue(planarMode));
    gpsr.Set("HelloInterval", TimeValue(Seconds(1.5)));
    InternetStackHelper stack;
    stack.SetRoutingHelper(gpsr);
    stack.Install(nodes);
    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer ifs = address.Assign(devices);
    NeighborCacheHelper().PopulateNeighborCache();

    // Một luồng CBR S -> D, bắt đầu sau khi beacon đã dựng xong bảng neighbor.
    UdpServerHelper server(9);
    ApplicationContainer sApp = server.Install(nodes.Get(7));
    sApp.Start(Seconds(1.0));
    UdpClientHelper client(ifs.GetAddress(7), 9);
    client.SetAttribute("MaxPackets", UintegerValue(1000000));
    client.SetAttribute("Interval", TimeValue(Seconds(0.25)));
    client.SetAttribute("PacketSize", UintegerValue(64));
    ApplicationContainer cApp = client.Install(nodes.Get(0));
    cApp.Start(Seconds(10.0));
    cApp.Stop(Seconds(simTime - 1.0));

    Config::ConnectWithoutContext("/NodeList/*/$ns3::gpsr::RoutingProtocol/Delivered",
                                  MakeCallback(&OnDelivered));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    uint64_t sent = DynamicCast<UdpClient>(cApp.Get(0))->GetTotalTx() / 64;
    uint64_t recv = DynamicCast<UdpServer>(sApp.Get(0))->GetReceived();
    std::cout << "planar=" << (planarMode == 0 ? "RNG" : "GG") << " gui=" << sent
              << " nhan=" << recv << " PDR=" << (sent ? 100.0 * recv / sent : 0.0) << "%\n";
    for (const auto& kv : g_hopCount)
    {
        std::cout << "  " << kv.second << " goi di " << kv.first << " chang\n";
    }
    Simulator::Destroy();
    return 0;
}
