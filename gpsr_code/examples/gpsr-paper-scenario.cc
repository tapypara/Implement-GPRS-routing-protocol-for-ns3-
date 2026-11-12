/*
 * Kịch bản mô phỏng GPSR theo mục 3 của bài báo (Karp & Kung, MobiCom 2000), cho ns-3.43.
 *
 * Thông số lấy từ bài báo:
 *  - 50 / 112 / 200 nút, vùng 1500x300 / 2250x450 / 3000x600 m (Bảng 2), 1 nút / 9000 m^2
 *  - tầm sóng 250 m, 802.11 2 Mbps, random waypoint, vận tốc tối đa 20 m/s
 *  - pause time 0, 30, 60, 120 s
 *  - 30 luồng CBR do 22 nút gửi, 2 Kbps, gói 64 byte, mô phỏng 900 s
 *  - mỗi cấu hình chạy 6 lần với chuyển động ngẫu nhiên khác nhau (--runIndex=1..6)
 *  - B = 1.5 s (một trong các giá trị ở Hình 9), RNG
 *
 * Quyết định riêng của nhóm (bài báo không nêu), cần ghi trong báo cáo:
 *  - vận tốc tối thiểu 1 m/s (tránh speed decay của random waypoint)
 *  - thời điểm bắt đầu mỗi luồng: ngẫu nhiên trong [flowStartMin, flowStartMax]
 */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include "../helper/gpsr-helper.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("GpsrPaperScenario");

int
main(int argc, char* argv[])
{
    uint32_t nNodes = 50;
    double simTime = 900.0;
    double minSpeed = 1.0;
    double maxSpeed = 20.0;
    double pauseTime = 0.0;
    uint32_t planarMode = 0; // 0 = RNG (bài báo), 1 = GG
    double helloInterval = 1.5; // B
    uint32_t runIndex = 1;
    uint32_t nFlows = 30;
    uint32_t nSources = 22;
    double cbrInterval = 0.256; // 64 byte * 8 / 0.256 s = 2 Kbps
    uint32_t packetSize = 64;
    double flowStartMin = 10.0;
    double flowStartMax = 100.0;
    bool rtsCts = false;
    double areaX = 0.0;
    double areaY = 0.0;

    CommandLine cmd(__FILE__);
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
    cmd.AddValue("rtsCts", "Bật RTS/CTS cho mọi gói unicast (xem lưu ý trong mã)", rtsCts);
    cmd.AddValue("areaX", "Ghi đè chiều dài vùng (m)", areaX);
    cmd.AddValue("areaY", "Ghi đè chiều rộng vùng (m)", areaY);
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
    nSources = std::min(nSources, nNodes);

    // 1. Nút
    NodeContainer nodes;
    nodes.Create(nNodes);

    // 2. Wi-Fi: 802.11b ad-hoc, 2 Mbps, tầm sóng 250 m.
    //    LƯU Ý: RangePropagationLossModel là hình tròn cứng, không có nhiễu/carrier sense
    //    ngoài 250 m. ns-2 mặc định nghe sóng xa hơn nhiều (khoảng 550 m), nên số va chạm
    //    sẽ khác. Cần đối chiếu với ns-default.tcl của ns-2.1b6 khi nhận xét kết quả.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("DsssRate2Mbps"),
                                 "ControlMode",
                                 StringValue("DsssRate1Mbps"));
    if (rtsCts)
    {
        // Kiểm tra thuộc tính này có tồn tại trong ns-3.43 của bạn; nếu báo lỗi thì bỏ cờ --rtsCts.
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

    // 4. GPSR
    GpsrHelper gpsr;
    gpsr.Set("PlanarMode", UintegerValue(planarMode));
    gpsr.Set("HelloInterval", TimeValue(Seconds(helloInterval)));
    InternetStackHelper stack;
    stack.SetRoutingHelper(gpsr);
    stack.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

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

        generated += std::ceil((simTime - start) / cbrInterval - 1e-9);
    }

    // 6. FlowMonitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

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

    // PDR theo FlowMonitor: gói bị từ chối ngay tại nguồn (không có route) KHÔNG nằm trong dataTx.
    double pdrFlowMon = dataTx > 0 ? 100.0 * dataRx / dataTx : 0.0;
    // PDR theo số gói CBR được lên lịch: gói bị từ chối tại nguồn được tính là mất (gần với cách đo của bài báo).
    double pdrGenerated = generated > 0 ? 100.0 * appRx / generated : 0.0;
    double avgDelayMs = dataRx > 0 ? 1000.0 * delaySum / dataRx : 0.0;
    // Xấp xỉ: timesForwarded tính cả gói về sau bị mất.
    double avgHops = dataRx > 0 ? static_cast<double>(forwards) / dataRx + 1.0 : 0.0;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "nNodes=" << nNodes << " area=" << areaX << "x" << areaY
              << " pause=" << pauseTime << " B=" << helloInterval
              << " planar=" << (planarMode == 0 ? "RNG" : "GG") << " run=" << runIndex << "\n";
    std::cout << "  CBR duoc len lich:        " << static_cast<uint64_t>(generated) << "\n";
    std::cout << "  FlowMonitor tx / rx:      " << dataTx << " / " << dataRx << "\n";
    std::cout << "  UdpServer nhan:           " << appRx << "\n";
    std::cout << "  PDR (FlowMonitor):        " << pdrFlowMon << " %\n";
    std::cout << "  PDR (so goi len lich):    " << pdrGenerated << " %\n";
    std::cout << "  Beacon da gui (overhead): " << beaconTx << " goi\n";
    std::cout << "  Do tre trung binh:        " << avgDelayMs << " ms\n";
    std::cout << "  So chang trung binh:      " << avgHops << "\n";
    // Một dòng dễ gom lại thành bảng/biểu đồ:
    std::cout << "RESULT," << nNodes << "," << pauseTime << "," << runIndex << ","
              << pdrGenerated << "," << pdrFlowMon << "," << beaconTx << "," << avgHops << ","
              << avgDelayMs << "\n";

    Simulator::Destroy();
    return 0;
}
