/*
 * Kịch bản mô phỏng GPSR theo mục 3 của bài báo (Karp & Kung, MobiCom 2000).
 *
 * Thông số lấy từ bài báo:
 *  - 50 / 112 / 200 nút, vùng 1500x300 / 2250x450 / 3000x600 m (Bảng 2), 1 nút / 9000 m^2
 *  - tầm sóng 250 m, 802.11 2 Mbps, random waypoint, vận tốc tối đa 20 m/s
 *  - pause time 0, 30, 60, 120 s
 *  - 30 luồng CBR do 22 nút gửi, 2 Kbps, gói 64 byte, mô phỏng 900 s
 *  - mỗi cấu hình chạy 6 lần với chuyển động ngẫu nhiên khác nhau (--runIndex=1..6)
 *  - B = 1.5 s (một trong các giá trị ở Hình 9), RNG
 *
 * Quyết định riêng (bài báo không nêu), cần ghi trong báo cáo:
 *  - vận tốc tối thiểu 1 m/s (tránh speed decay của random waypoint)
 *  - thời điểm bắt đầu mỗi luồng: ngẫu nhiên trong [flowStartMin, flowStartMax]
 *  - cbrInterval = 0.25 s (4 gói/s x 64 B x 8 = 2048 bps ~ "2 Kbps")
 *  - RTS/CTS TẮT mặc định: bài báo không nêu. Thử nghiệm (B=1.0, pause 0, 50 nút, 900 s):
 *    bật -> PDR_Conn ~61%, tắt -> ~92%. Ghi rõ lựa chọn này trong báo cáo.
 *  - ARP TĨNH mặc định (--staticArp=1, NeighborCacheHelper). ARP của ns-3 làm mất phần lớn gói:
 *    khi GPSR chọn một neighbor đã ra khỏi tầm (thông tin vị trí cũ), ARP request không được trả lời,
 *    gói bị bỏ (hàng chờ ARP chỉ 3 gói) mà KHÔNG có phản hồi lỗi MAC (Sec. 2.5) nên neighbor không
 *    bị xóa; entry ARP còn bị đánh dấu "dead" 100 s, mọi gói tới nút đó bị bỏ âm thầm kể cả khi nó
 *    quay lại tầm. Đo thử (50 nút, B=3, 300 s): 88% số gói mất là do ARP. Tắt bằng --staticArp=0.
 *  - Dòng DIAG: số gói bị bỏ theo nguyên nhân (ARP; IP: 1=TTL hết, 5=route error;
 *    MAC: 1=hết hạn trong hàng chờ, 2=quá số lần gửi lại).
 *
 * Chế độ:
 *  --protocol=gpsr (mặc định) | dsr   : DSR của ns-3 làm đường so sánh (KHÔNG phải DSR của ns-2 CMU
 *                                       mà bài báo dùng, nên chỉ so sánh xu hướng, không so từng số)
 *  --pathStats=1                      : đo phân bố số chặng vượt quá đường ngắn nhất thật (Hình 11),
 *                                       chỉ cho GPSR; dòng HOPHIST
 *  --stateTime=300                    : lấy mẫu kích thước bảng neighbor tại giây này (mục 3.6);
 *                                       dòng STATE, chỉ cho GPSR
 *
 * Với DSR, FlowMonitor không phân loại được gói IP giao thức 48 nên PDR_Flow/độ trễ/số chặng = 0;
 * PDR dùng UdpServer (PDR_Gen) và "overhead" là số gói điều khiển DSR (RREQ/RREP/RERR...) đã gửi,
 * đếm ở lớp IP theo từng lần phát (mỗi lần chuyển tiếp RREQ tính một gói, như bài báo).
 */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/dsr-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/neighbor-cache-helper.h"
#include "ns3/arp-l3-protocol.h"

#include "ns3/gpsr-helper.h"
#include "ns3/gpsr-routing-protocol.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("GpsrPaperScenario");

namespace
{
const double RADIO_RANGE = 250.0;

NodeContainer g_nodes;
std::map<uint64_t, int> g_optHops;  // uid của gói CBR -> số chặng đường ngắn nhất tại lúc gửi
std::vector<uint64_t> g_hist(6, 0); // số chặng vượt quá tối ưu: 0,1,2,3,4,>=5
uint64_t g_histTotal = 0;
uint64_t g_dsrControl = 0;
bool g_pathStats = false;
// Định nghĩa PDR của bài báo (mục 3.2): chỉ tính gói có đường đi tới đích tại thời điểm gửi.
std::set<uint64_t> g_connectedSent; // uid các gói CBR được gửi khi nguồn–đích còn kết nối
uint64_t g_connectedRx = 0;         // số gói trong số đó đến được UdpServer
std::set<uint64_t> g_connectedRxSeen; // chống đếm trùng (gói tái chuyển tiếp sau lỗi MAC có thể tới 2 lần)
double g_avgNeighbors = -1.0;
// --- chan doan (diag) ---
uint64_t g_arpDrop = 0;
std::map<int, uint64_t> g_ipDrop;
std::map<int, uint64_t> g_macDrop;
void OnArpDrop(Ptr<const Packet>) { ++g_arpDrop; }
void OnIpDrop(const Ipv4Header& h, Ptr<const Packet>, Ipv4L3Protocol::DropReason r, Ptr<Ipv4>, uint32_t)
{ if (!h.GetDestination().IsBroadcast() && h.GetDestination() != Ipv4Address("10.1.255.255")) ++g_ipDrop[r]; }
void OnMacDrop(WifiMacDropReason r, Ptr<const WifiMpdu> m)
{ if (m->GetHeader().IsData() && !m->GetHeader().GetAddr1().IsGroup()) ++g_macDrop[r]; }

// Đường ngắn nhất (số chặng) trên đồ thị đơn vị (khoảng cách <= tầm sóng) tại thời điểm gọi.
int
ShortestHops(uint32_t s, uint32_t d)
{
    const uint32_t n = g_nodes.GetN();
    std::vector<Vector> pos(n);
    for (uint32_t i = 0; i < n; ++i)
    {
        pos[i] = g_nodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
    }
    std::vector<int> dist(n, -1);
    std::vector<uint32_t> queue;
    queue.push_back(s);
    dist[s] = 0;
    for (std::size_t head = 0; head < queue.size(); ++head)
    {
        uint32_t u = queue[head];
        if (u == d)
        {
            return dist[u];
        }
        for (uint32_t v = 0; v < n; ++v)
        {
            if (dist[v] < 0 && CalculateDistance(pos[u], pos[v]) <= RADIO_RANGE)
            {
                dist[v] = dist[u] + 1;
                queue.push_back(v);
            }
        }
    }
    return -1; // không có đường
}

void
OnClientTx(uint32_t s, uint32_t d, Ptr<const Packet> p)
{
    int h = ShortestHops(s, d);
    if (h > 0)
    {
        g_connectedSent.insert(p->GetUid());
        if (g_pathStats)
        {
            g_optHops[p->GetUid()] = h;
        }
    }
}

void
OnServerRx(Ptr<const Packet> p)
{
    if (g_connectedSent.count(p->GetUid()) > 0 && g_connectedRxSeen.insert(p->GetUid()).second)
    {
        ++g_connectedRx;
    }
}

void
OnDelivered(Ptr<const Packet> p, uint32_t hops, Ipv4Address, Ipv4Address)
{
    auto it = g_optHops.find(p->GetUid());
    if (it == g_optHops.end())
    {
        return;
    }
    int excess = static_cast<int>(hops) - it->second;
    if (excess < 0)
    {
        excess = 0; // topology changed while the packet was in flight
    }
    g_hist[std::min(excess, 5)]++;
    g_histTotal++;
    g_optHops.erase(it);
}

// Đếm gói điều khiển DSR (messageType == 1) mỗi lần được phát ở lớp IP.
void
OnIpTx(Ptr<const Packet> p, Ptr<Ipv4>, uint32_t)
{
    Ptr<Packet> c = p->Copy();
    Ipv4Header ip;
    c->RemoveHeader(ip);
    if (ip.GetProtocol() != 48) // DsrRouting::PROT_NUMBER
    {
        return;
    }
    dsr::DsrFsHeader h;
    c->PeekHeader(h);
    if (h.GetMessageType() == 1)
    {
        ++g_dsrControl;
    }
}

// Mục 3.6: kích thước bảng neighbor trung bình tại một thời điểm.
void
SampleState()
{
    double sum = 0.0;
    uint32_t cnt = 0;
    for (uint32_t i = 0; i < g_nodes.GetN(); ++i)
    {
        Ptr<gpsr::RoutingProtocol> r = g_nodes.Get(i)->GetObject<gpsr::RoutingProtocol>();
        if (r)
        {
            sum += r->GetNeighborCount();
            ++cnt;
        }
    }
    if (cnt > 0)
    {
        g_avgNeighbors = sum / cnt;
    }
}
} // namespace

int
main(int argc, char* argv[])
{
    std::string protocol = "gpsr";
    bool pathStats = false;
    double stateTime = 300.0;
    uint32_t nNodes = 50;
    double simTime = 900.0;
    double minSpeed = 1.0;
    double maxSpeed = 20.0;
    double pauseTime = 0.0;
    uint32_t planarMode = 0;    // 0 = RNG (bài báo), 1 = GG
    double helloInterval = 1.5; // B
    uint32_t runIndex = 1;
    uint32_t nFlows = 30;
    uint32_t nSources = 22;
    double cbrInterval = 0.25; // 64 byte * 8 * 4 gói/s = 2048 bps
    uint32_t packetSize = 64;
    double flowStartMin = 10.0;
    double flowStartMax = 100.0;
    bool rtsCts = false; // bài báo không nêu; xem ghi chú đầu file
    bool paperPdr = true; // đo PDR theo bài báo (tắt để cô lập lỗi)
    double areaX = 0.0;
    double areaY = 0.0;
    bool staticArp = true; // xem ghi chu ARP o dau file
    bool diag = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("protocol", "gpsr hoặc dsr", protocol);
    cmd.AddValue("pathStats", "Đo phân bố số chặng vượt quá tối ưu (Hình 11, chỉ GPSR)", pathStats);
    cmd.AddValue("stateTime", "Giây lấy mẫu kích thước bảng neighbor (mục 3.6, chỉ GPSR)", stateTime);
    cmd.AddValue("nNodes", "Số nút (50, 112, 200)", nNodes);
    cmd.AddValue("simTime", "Thời gian mô phỏng (s)", simTime);
    cmd.AddValue("minSpeed", "Vận tốc tối thiểu (m/s)", minSpeed);
    cmd.AddValue("maxSpeed", "Vận tốc tối đa (m/s)", maxSpeed);
    cmd.AddValue("pauseTime", "Pause time (s): 0, 30, 60, 120", pauseTime);
    cmd.AddValue("planarMode", "0 = RNG, 1 = GG", planarMode);
    cmd.AddValue("helloInterval", "Chu kỳ beacon B (s)", helloInterval);
    cmd.AddValue("runIndex", "Lần chạy (1..6): đổi chuyển động ngẫu nhiên", runIndex);
    cmd.AddValue("nFlows", "Số luồng CBR", nFlows);
    cmd.AddValue("nSources", "Số nút gửi", nSources);
    cmd.AddValue("cbrInterval", "Khoảng cách giữa 2 gói CBR (s)", cbrInterval);
    cmd.AddValue("packetSize", "Kích thước gói CBR (byte)", packetSize);
    cmd.AddValue("flowStartMin", "Thời điểm sớm nhất bắt đầu luồng (s)", flowStartMin);
    cmd.AddValue("flowStartMax", "Thời điểm muộn nhất bắt đầu luồng (s)", flowStartMax);
    cmd.AddValue("rtsCts", "Bật RTS/CTS cho mọi gói unicast", rtsCts);
    cmd.AddValue("paperPdr", "Đo PDR theo bài báo (0 = tắt hook Tx/Rx)", paperPdr);
    cmd.AddValue("areaX", "Ghi đè chiều dài vùng (m)", areaX);
    cmd.AddValue("areaY", "Ghi đè chiều rộng vùng (m)", areaY);
    cmd.AddValue("staticArp", "Dien san bang ARP (bo qua anh huong ARP cua ns-3)", staticArp);
    cmd.AddValue("diag", "In thong ke goi bi bo theo nguyen nhan", diag);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(runIndex);

    // Vùng theo Bảng 2; số nút khác thì giữ mật độ 9000 m^2/nút và tỉ lệ 5:1.
    if (areaX == 0.0 || areaY == 0.0)
    {
        if (nNodes == 50)
        {
            areaX = 1500.0;
            areaY = 300.0;
        }
        else if (nNodes == 112)
        {
            areaX = 2250.0;
            areaY = 450.0;
        }
        else if (nNodes == 200)
        {
            areaX = 3000.0;
            areaY = 600.0;
        }
        else
        {
            areaX = std::sqrt(5.0 * 9000.0 * nNodes);
            areaY = areaX / 5.0;
        }
    }
    nSources = std::max<uint32_t>(1, std::min(nSources, nNodes));
    const bool useDsr = (protocol == "dsr");
    NS_ABORT_MSG_IF(!useDsr && protocol != "gpsr", "--protocol phải là gpsr hoặc dsr");

    // 1. Nút
    NodeContainer nodes;
    nodes.Create(nNodes);
    g_nodes = nodes;

    // 2. Wi-Fi: 802.11b ad-hoc, 2 Mbps, tầm sóng 250 m.
    //    LƯU Ý: RangePropagationLossModel là hình tròn cứng, không có nhiễu/carrier sense
    //    ngoài 250 m. ns-2 mặc định nghe sóng xa hơn nhiều (khoảng 550 m), nên số va chạm
    //    sẽ khác. Cần ghi rõ điều này khi nhận xét kết quả.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("DsssRate2Mbps"),
                                 "ControlMode",
                                 StringValue("DsssRate1Mbps"));
    if (rtsCts)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue(0));
    }

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel",
                                   "MaxRange",
                                   DoubleValue(250.0));
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    // 3. Di chuyển: random waypoint trong areaX x areaY.
    MobilityHelper mobility;
    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
    pos.Set("X",
            StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(areaX) + "]"));
    pos.Set("Y",
            StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(areaY) + "]"));
    Ptr<PositionAllocator> alloc = pos.Create()->GetObject<PositionAllocator>();
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                              "Speed",
                              StringValue("ns3::UniformRandomVariable[Min=" +
                                          std::to_string(minSpeed) +
                                          "|Max=" + std::to_string(maxSpeed) + "]"),
                              "Pause",
                              StringValue("ns3::ConstantRandomVariable[Constant=" +
                                          std::to_string(pauseTime) + "]"),
                              "PositionAllocator",
                              PointerValue(alloc));
    mobility.SetPositionAllocator(alloc);
    mobility.Install(nodes);

    // 4. Giao thức định tuyến
    InternetStackHelper stack;
    if (useDsr)
    {
        DsrHelper dsrHelper;
        DsrMainHelper dsrMain;
        stack.Install(nodes);
        dsrMain.Install(dsrHelper, nodes);
    }
    else
    {
        GpsrHelper gpsr;
        gpsr.Set("PlanarMode", UintegerValue(planarMode));
        gpsr.Set("HelloInterval", TimeValue(Seconds(helloInterval)));
        stack.SetRoutingHelper(gpsr);
        stack.Install(nodes);
    }

    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    if (staticArp)
    {
        NeighborCacheHelper nch;
        nch.PopulateNeighborCache();
    }

    // 5. Luồng CBR: nSources nút gửi (mỗi nút ít nhất 1 luồng), tổng nFlows luồng,
    //    đích ngẫu nhiên khác nguồn. Nguồn/đích/thời điểm bắt đầu phụ thuộc runIndex.
    const uint16_t dataPortBase = 9000;
    const uint16_t beaconPort = 666; // gpsr::RoutingProtocol::GPSR_PORT

    Ptr<UniformRandomVariable> rv = CreateObject<UniformRandomVariable>();
    std::vector<uint32_t> idx(nNodes);
    std::iota(idx.begin(), idx.end(), 0);
    for (uint32_t i = nNodes - 1; i > 0; --i)
    {
        std::swap(idx[i], idx[rv->GetInteger(0, i)]);
    }

    std::vector<Ptr<UdpServer>> servers;
    double generated = 0.0; // số gói CBR mà các nguồn ĐÃ CỐ gửi (theo lịch)
    for (uint32_t f = 0; f < nFlows; ++f)
    {
        uint32_t src = (f < nSources) ? idx[f] : idx[rv->GetInteger(0, nSources - 1)];
        uint32_t dst;
        do
        {
            dst = rv->GetInteger(0, nNodes - 1);
        } while (dst == src);

        double start = rv->GetValue(flowStartMin, flowStartMax);

        UdpServerHelper server(dataPortBase + f);
        ApplicationContainer sApp = server.Install(nodes.Get(dst));
        sApp.Start(Seconds(1.0));
        sApp.Stop(Seconds(simTime));
        servers.push_back(DynamicCast<UdpServer>(sApp.Get(0)));

        UdpClientHelper client(interfaces.GetAddress(dst), dataPortBase + f);
        client.SetAttribute("MaxPackets", UintegerValue(4000000));
        client.SetAttribute("Interval", TimeValue(Seconds(cbrInterval)));
        client.SetAttribute("PacketSize", UintegerValue(packetSize));
        ApplicationContainer cApp = client.Install(nodes.Get(src));
        cApp.Start(Seconds(start));
        cApp.Stop(Seconds(simTime));
        if (!useDsr && paperPdr)
        {
            cApp.Get(0)->TraceConnectWithoutContext("Tx", MakeBoundCallback(&OnClientTx, src, dst));
            sApp.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&OnServerRx));
        }

        generated += std::ceil(std::max(0.0, simTime - start) / cbrInterval - 1e-9);
    }

    // 6. FlowMonitor
    g_pathStats = pathStats && !useDsr;
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    if (pathStats && !useDsr)
    {
        Config::ConnectWithoutContext("/NodeList/*/$ns3::gpsr::RoutingProtocol/Delivered",
                                      MakeCallback(&OnDelivered));
    }
    if (useDsr)
    {
        Config::ConnectWithoutContext("/NodeList/*/$ns3::Ipv4L3Protocol/Tx", MakeCallback(&OnIpTx));
    }
    else if (stateTime > 0.0 && stateTime < simTime)
    {
        Simulator::Schedule(Seconds(stateTime), &SampleState);
    }

    if (diag)
    {
        Config::ConnectWithoutContext("/NodeList/*/$ns3::ArpL3Protocol/Drop", MakeCallback(&OnArpDrop));
        Config::ConnectWithoutContext("/NodeList/*/$ns3::Ipv4L3Protocol/Drop", MakeCallback(&OnIpDrop));
        Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/DroppedMpdu",
                                      MakeCallback(&OnMacDrop));
    }
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // 7. Thống kê. Chỉ tính luồng dữ liệu CBR (cổng 9000..); beacon (cổng 666) tách riêng.
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    uint64_t dataTx = 0;
    uint64_t dataRx = 0;
    uint64_t forwards = 0;
    double delaySum = 0.0;
    uint64_t beaconTx = 0;
    for (auto const& kv : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
        if (t.destinationPort == beaconPort)
        {
            beaconTx += kv.second.txPackets;
        }
        else if (t.destinationPort >= dataPortBase && t.destinationPort < dataPortBase + nFlows)
        {
            dataTx += kv.second.txPackets;
            dataRx += kv.second.rxPackets;
            forwards += kv.second.timesForwarded;
            delaySum += kv.second.delaySum.GetSeconds();
        }
    }

    uint64_t appRx = 0;
    for (auto const& s : servers)
    {
        appRx += s->GetReceived();
    }

    if (!useDsr)
    {
        beaconTx = gpsr::RoutingProtocol::GetBeaconTxCount();
    }
    if (useDsr)
    {
        beaconTx = g_dsrControl; // gói điều khiển DSR, không phải beacon
    }

    // PDR theo FlowMonitor: gói bị từ chối ngay tại nguồn (không có route) KHÔNG nằm trong dataTx.
    double pdrFlowMon = dataTx > 0 ? 100.0 * dataRx / dataTx : 0.0;
    // PDR theo số gói CBR được lên lịch: gói bị từ chối tại nguồn được tính là mất.
    double pdrGenerated = generated > 0 ? 100.0 * appRx / generated : 0.0;
    // PDR theo bài báo (mục 3.2): mẫu số chỉ gồm gói có đường đi tới đích lúc gửi.
    double pdrConnected = g_connectedSent.size() > 0
                              ? 100.0 * g_connectedRx / g_connectedSent.size()
                              : 0.0;
    double avgDelayMs = dataRx > 0 ? 1000.0 * delaySum / dataRx : 0.0;
    // Xấp xỉ: timesForwarded tính cả gói về sau bị mất.
    double avgHops = dataRx > 0 ? static_cast<double>(forwards) / dataRx + 1.0 : 0.0;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "nNodes=" << nNodes << " area=" << areaX << "x" << areaY
              << " pause=" << pauseTime << " B=" << helloInterval
              << " protocol=" << protocol
              << " planar=" << (planarMode == 0 ? "RNG" : "GG") << " run=" << runIndex << "\n";
    std::cout << "  CBR duoc len lich:        " << static_cast<uint64_t>(generated) << "\n";
    std::cout << "  FlowMonitor tx / rx:      " << dataTx << " / " << dataRx << "\n";
    std::cout << "  UdpServer nhan:           " << appRx << "\n";
    std::cout << "  PDR (FlowMonitor):        " << pdrFlowMon << " %\n";
    std::cout << "  PDR (so goi len lich):    " << pdrGenerated << " %\n";
    std::cout << "  PDR (co duong, theo bai): " << pdrConnected << " %  ["
              << g_connectedRx << "/" << g_connectedSent.size() << "]\n";
    std::cout << "  Beacon da gui (overhead): " << beaconTx << " goi\n";
    std::cout << "  Do tre trung binh:        " << avgDelayMs << " ms\n";
    std::cout << "  So chang trung binh:      " << avgHops << "\n";
    std::cout << "RESULT," << nNodes << "," << pauseTime << "," << runIndex << ","
              << pdrGenerated << "," << pdrFlowMon << "," << beaconTx << "," << avgHops << ","
              << avgDelayMs << "," << pdrConnected << "\n";

    if (diag)
    {
        std::cout << "DIAG arpDrop=" << g_arpDrop;
        for (auto& kv : g_ipDrop) std::cout << " ipDrop[" << kv.first << "]=" << kv.second;
        for (auto& kv : g_macDrop) std::cout << " macDrop[" << kv.first << "]=" << kv.second;
        std::cout << "\n";
    }
    if (!useDsr && g_avgNeighbors >= 0.0)
    {
        std::cout << "STATE," << nNodes << "," << pauseTime << "," << runIndex << ","
                  << g_avgNeighbors << "\n";
    }
    if (pathStats && !useDsr && g_histTotal > 0)
    {
        std::cout << "HOPHIST," << nNodes << "," << pauseTime << "," << runIndex << ","
                  << g_histTotal;
        for (uint32_t k = 0; k < 6; ++k)
        {
            std::cout << "," << static_cast<double>(g_hist[k]) / g_histTotal;
        }
        std::cout << "\n";
    }

    Simulator::Destroy();
    return 0;
}
