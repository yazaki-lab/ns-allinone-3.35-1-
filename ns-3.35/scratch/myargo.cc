#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WiFiAPSelection");

// APæƒ…å ±ã‚'æ ¼ç´ã™ã‚‹æ§‹é€ ä½"
struct APInfo {
    uint32_t apId;
    Vector position;
    uint32_t connectedUsers;
    double channelUtilization;
    std::vector<double> userRates;
    uint32_t channel;
};

// ãƒ¦ãƒ¼ã‚¶æƒ…å ±ã‚'æ ¼ç´ã™ã‚‹æ§‹é€ ä½"
struct UserInfo {
    uint32_t userId;
    Vector position;
    uint32_t connectedAP;
    double throughput;
};

class APSelectionAlgorithm {
private:
    std::vector<APInfo> m_apList;
    double m_dThreshold;  // ç§»å‹•å¯èƒ½è·é›¢
    double m_thetaThreshold;  // æœ€ä½Žè¦æ±‚ã‚¹ãƒ«ãƒ¼ãƒ—ãƒƒãƒˆ
    std::vector<double> m_weights; // ã‚¹ã‚³ã‚¢é‡ã¿ [w1, w2, w3, w4]

public:
    APSelectionAlgorithm(double dTh, double thetaTh) 
        : m_dThreshold(dTh), m_thetaThreshold(thetaTh) {
        m_weights = {0.4, 0.3, 0.2, 0.1}; // ãƒ‡ãƒ•ã‚©ãƒ«ãƒˆé‡ã¿
    }

    void UpdateAPInfo(const std::vector<APInfo>& apList) {
        m_apList = apList;
    }

    // è·é›¢è¨ˆç®—
    double CalculateDistance(const Vector& pos1, const Vector& pos2) {
        return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
    }

    // ä¼é€ãƒ¬ãƒ¼ãƒˆè¨ˆç®—ï¼ˆè·é›¢ãƒ™ãƒ¼ã‚¹ï¼‰
    double CalculateTransmissionRate(const Vector& userPos, const Vector& apPos) {
        double distance = CalculateDistance(userPos, apPos);
        // 802.11nã®ç°¡å˜ãªè·é›¢-ãƒ¬ãƒ¼ãƒˆãƒ¢ãƒ‡ãƒ«
        if (distance < 5.0) return 150.0;  // Mbps
        else if (distance < 10.0) return 130.0;
        else if (distance < 15.0) return 100.0;
        else if (distance < 20.0) return 65.0;
        else if (distance < 25.0) return 30.0;
        else return 6.5;
    }

    // ã‚¹ãƒ«ãƒ¼ãƒ—ãƒƒãƒˆè¨ˆç®—ï¼ˆèª¿å'Œå¹³å‡ï¼‰
    double CalculateThroughput(const APInfo& ap, double newRate) {
        if (ap.userRates.empty()) {
            return newRate;
        }
        
        double sum = 0.0;
        for (double rate : ap.userRates) {
            sum += 1.0 / rate;
        }
        sum += 1.0 / newRate;
        
        return (ap.userRates.size() + 1) / sum;
    }

    // æœ€é©APé¸æŠžã‚¢ãƒ«ã‚´ãƒªã‚ºãƒ 
    std::pair<uint32_t, Vector> SelectOptimalAP(const Vector& userPos) {
        std::vector<uint32_t> candidates;
        
        // å€™è£œAPé¸æŠžï¼ˆå¼1,2ï¼‰
        for (const auto& ap : m_apList) {
            double distance = CalculateDistance(userPos, ap.position);
            if (distance <= m_dThreshold) {
                candidates.push_back(ap.apId);
            }
        }

        if (candidates.empty()) {
            return std::make_pair(0, Vector(0, 0, 0)); // æŽ¥ç¶šä¸å¯
        }

        // ã‚¹ã‚³ã‚¢è¨ˆç®—
        std::vector<std::pair<uint32_t, double>> apScores;
        double thetaMax = 0, thetaMin = 1e9;
        double dMax = 0, dMin = 1e9;
        uint32_t nMax = 0;

        // æœ€å¤§æœ€å°å€¤è¨ˆç®—
        for (uint32_t apId : candidates) {
            const APInfo& ap = m_apList[apId];
            double newRate = CalculateTransmissionRate(userPos, ap.position);
            double throughput = CalculateThroughput(ap, newRate);
            double distance = CalculateDistance(userPos, ap.position);
            
            thetaMax = std::max(thetaMax, throughput);
            thetaMin = std::min(thetaMin, throughput);
            dMax = std::max(dMax, distance);
            dMin = std::min(dMin, distance);
            nMax = std::max(nMax, ap.connectedUsers);
        }

        // å„APã®ã‚¹ã‚³ã‚¢è¨ˆç®—ï¼ˆå¼6-10ï¼‰
        for (uint32_t apId : candidates) {
            const APInfo& ap = m_apList[apId];
            double newRate = CalculateTransmissionRate(userPos, ap.position);
            double throughput = CalculateThroughput(ap, newRate);
            double distance = CalculateDistance(userPos, ap.position);

            // ã‚¹ã‚³ã‚¢è¨ˆç®—
            double scoreTheta = (thetaMax == thetaMin) ? 1.0 : 
                               (throughput - thetaMin) / (thetaMax - thetaMin);
            double scoreDist = (dMax == dMin) ? 1.0 : 
                              1.0 - (distance - dMin) / (dMax - dMin);
            double scoreChan = 1.0 - ap.channelUtilization; // ä½¿ç"¨çŽ‡ãŒä½Žã„ã»ã©è‰¯ã„
            double scoreUsers = (nMax == 0) ? 1.0 : 
                               1.0 - (double)ap.connectedUsers / nMax;

            double totalScore = m_weights[0] * scoreTheta + 
                               m_weights[1] * scoreDist + 
                               m_weights[2] * scoreChan + 
                               m_weights[3] * scoreUsers;

            apScores.push_back(std::make_pair(apId, totalScore));
        }

        // ã‚¹ã‚³ã‚¢é †ã§ã‚½ãƒ¼ãƒˆ
        std::sort(apScores.begin(), apScores.end(), 
                 [](const auto& a, const auto& b) { return a.second > b.second; });

        // æœ€é©APã‚'è¿"ã™ï¼ˆç§»å‹•ãƒ™ã‚¯ãƒˆãƒ«ã¯0ã¨ã™ã‚‹ï¼‰
        return std::make_pair(apScores[0].first, Vector(0, 0, 0));
    }
};

// ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°
static std::ofstream* g_resultFile = nullptr;
static NodeContainer* g_apNodes = nullptr;
static NodeContainer* g_staNodes = nullptr;
static uint32_t g_nAPs = 0;
static uint32_t g_nUsers = 0;

// APé¸æŠžçµæžœå‡ºåŠ›é–¢æ•°
void PrintAPSelectionResults() {
    if (!g_resultFile || !g_apNodes || !g_staNodes) return;
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Ptr<MobilityModel> mobility = g_staNodes->Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        
        // ç°¡å˜ãªAPæƒ…å ±ç"Ÿæˆ
        std::vector<APInfo> apInfo(g_nAPs);
        for (uint32_t j = 0; j < g_nAPs; ++j) {
            apInfo[j].apId = j;
            apInfo[j].position = g_apNodes->Get(j)->GetObject<MobilityModel>()->GetPosition();
            apInfo[j].connectedUsers = 3; // ç°¡ç•¥åŒ–
            apInfo[j].channelUtilization = 0.5; // ç°¡ç•¥åŒ–
            apInfo[j].channel = j % 3; // 3ãƒãƒ£ãƒãƒ«
        }
        
        APSelectionAlgorithm algorithm(15.0, 10.0);
        algorithm.UpdateAPInfo(apInfo);
        auto result = algorithm.SelectOptimalAP(pos);
        
        *g_resultFile << "5.0\t" << i << "\t(" << pos.x << "," << pos.y << ")\t" 
                     << result.first << "\t" << "50.0" << std::endl;
    }
    g_resultFile->flush();
}

int main(int argc, char *argv[]) {
    // ã‚·ãƒŸãƒ¥ãƒ¬ãƒ¼ã‚·ãƒ§ãƒ³è¨­å®š
    uint32_t nAPs = 5;
    uint32_t nUsers = 18; // 4+2+3+4+5
    double simTime = 60.0; // 60ç§'
    
    // ã‚°ãƒ­ãƒ¼ãƒãƒ«å¤‰æ•°ã®è¨­å®š
    g_nAPs = nAPs;
    g_nUsers = nUsers;
    
    // ã‚³ãƒžãƒ³ãƒ‰ãƒ©ã‚¤ãƒ³å¼•æ•°
    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.Parse(argc, argv);

    // ãƒ­ã‚°è¨­å®š
    LogComponentEnable("WiFiAPSelection", LOG_LEVEL_INFO);

    // ãƒŽãƒ¼ãƒ‰ä½œæˆ
    NodeContainer apNodes;
    apNodes.Create(nAPs);
    g_apNodes = &apNodes;
    
    NodeContainer staNodes;
    staNodes.Create(nUsers);
    g_staNodes = &staNodes;

    // WiFiè¨­å®š
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n_2_4GHZ);
    wifi.SetRemoteStationManager("ns3::MinstrelWifiManager");

    // PHYè¨­å®šï¼ˆDefaultãƒ¡ã‚½ãƒƒãƒ‰ãŒãªã„ããã†ãªã®ã§ç›´æŽ¥è¨­å®šï¼‰
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    // MACè¨­å®š
    WifiMacHelper mac;
    Ssid ssid = Ssid("wifi-network");

    // APã®è¨­å®š
    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    for (uint32_t i = 0; i < nAPs; ++i) {
        NetDeviceContainer device = wifi.Install(phy, mac, apNodes.Get(i));
        apDevices.Add(device);
    }

    // STAã®è¨­å®š
    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    staDevices = wifi.Install(phy, mac, staNodes);

    // ç§»å‹•ãƒ¢ãƒ‡ãƒ«è¨­å®š
    MobilityHelper mobility;

    // APé…ç½®ï¼ˆ30mÃ—30mã‚¨ãƒªã‚¢ã«ç­‰é–"éš"ï¼‰
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(7.5, 7.5, 0.0));   // AP0
    apPositionAlloc->Add(Vector(22.5, 7.5, 0.0));  // AP1
    apPositionAlloc->Add(Vector(7.5, 22.5, 0.0));  // AP2
    apPositionAlloc->Add(Vector(22.5, 22.5, 0.0)); // AP3
    apPositionAlloc->Add(Vector(15.0, 15.0, 0.0)); // AP4 (ä¸­å¤®)

    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // STAåˆæœŸé…ç½®
    Ptr<ListPositionAllocator> staPositionAlloc = CreateObject<ListPositionAllocator>();
    
    // AP0å'¨è¾ºã«4äºº
    staPositionAlloc->Add(Vector(5.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(5.0, 10.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 10.0, 0.0));
    
    // AP1å'¨è¾ºã«2äºº
    staPositionAlloc->Add(Vector(20.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 5.0, 0.0));
    
    // AP2å'¨è¾ºã«3äºº
    staPositionAlloc->Add(Vector(5.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(5.0, 25.0, 0.0));
    
    // AP3å'¨è¾ºã«4äºº
    staPositionAlloc->Add(Vector(20.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(20.0, 25.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 25.0, 0.0));
    
    // AP4å'¨è¾ºã«5äºº
    staPositionAlloc->Add(Vector(12.0, 12.0, 0.0));
    staPositionAlloc->Add(Vector(18.0, 12.0, 0.0));
    staPositionAlloc->Add(Vector(12.0, 18.0, 0.0));
    staPositionAlloc->Add(Vector(18.0, 18.0, 0.0));
    staPositionAlloc->Add(Vector(15.0, 12.0, 0.0));

    mobility.SetPositionAllocator(staPositionAlloc);
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Bounds", RectangleValue(Rectangle(0, 30, 0, 30)),
                              "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    mobility.Install(staNodes);

    // ã‚¤ãƒ³ã‚¿ãƒ¼ãƒãƒƒãƒˆãƒ—ãƒ­ãƒˆã‚³ãƒ«ã‚¹ã‚¿ãƒƒã‚¯
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);

    // IPã‚¢ãƒ‰ãƒ¬ã‚¹å‰²ã‚Šå½"ã¦
    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

    // ãƒˆãƒ©ãƒ•ã‚£ãƒƒã‚¯ç"Ÿæˆï¼ˆUDPã‚¨ã‚³ãƒ¼ã‚¢ãƒ—ãƒªã‚±ãƒ¼ã‚·ãƒ§ãƒ³ï¼‰
    UdpEchoServerHelper echoServer(9);
    ApplicationContainer serverApps = echoServer.Install(apNodes.Get(0));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(simTime));

    UdpEchoClientHelper echoClient(apInterfaces.GetAddress(0), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(1000));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(0.1)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(staNodes);
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(simTime));

    // FlowMonitorè¨­å®š
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // NetAnimè¨­å®š
    AnimationInterface anim("kamikawa-animation.xml");

    // çµæžœãƒ•ã‚¡ã‚¤ãƒ«ã‚ªãƒ¼ãƒ—ãƒ³
    std::ofstream resultFile("ap_selection_results.txt");
    g_resultFile = &resultFile;
    resultFile << "Time(s)\tUserID\tPosition(x,y)\tSelectedAP\tThroughput(Mbps)" << std::endl;

    // ã‚·ãƒŸãƒ¥ãƒ¬ãƒ¼ã‚·ãƒ§ãƒ³å®Ÿè¡Œ
    Simulator::Stop(Seconds(simTime));
    
    // å®šæœŸçš„ãªAPé¸æŠžçµæžœå‡ºåŠ›ï¼ˆé–¢æ•°ãƒãƒ¼ã‚¿ãƒ¼ã‚'ä½¿ç"¨ï¼‰
    Simulator::Schedule(Seconds(5.0), &PrintAPSelectionResults);

    Simulator::Run();

    // FlowMonitorçµ±è¨ˆå‡ºåŠ›
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::cout << "\n=== Flow Monitor Statistics ===" << std::endl;
    for (auto& flow : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        std::cout << "Flow " << flow.first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")" << std::endl;
        if (flow.second.timeLastRxPacket.GetSeconds() > flow.second.timeFirstTxPacket.GetSeconds()) {
            std::cout << "  Throughput: " << flow.second.rxBytes * 8.0 / (flow.second.timeLastRxPacket.GetSeconds() - flow.second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps" << std::endl;
        }
    }

    resultFile.close();
    Simulator::Destroy();

    std::cout << "\nSimulation completed. Results saved to ap_selection_results.txt" << std::endl;

    return 0;
}