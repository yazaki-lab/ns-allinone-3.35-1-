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

// APの情報
struct APInfo {
    uint32_t apId;
    Vector position;
    uint32_t connectedUsers;
    double channelUtilization;
    std::vector<double> userRates;
    uint32_t channel;
};

// Userの情報
struct UserInfo {
    uint32_t userId;
    Vector position;
    uint32_t connectedAP;
    double throughput;
};

class APSelectionAlgorithm {
private:
    std::vector<APInfo> m_apList;
    double m_dThreshold;  // APの距離閾値
    double m_thetaThreshold;  // 最低要求スループット
    std::vector<double> m_weights; // 各指標の重み [w1, w2, w3, w4]

public:
    APSelectionAlgorithm(double dTh, double thetaTh) 
        : m_dThreshold(dTh), m_thetaThreshold(thetaTh) {
        m_weights = {0.4, 0.3, 0.2, 0.1}; // 各指標の重み  
    }

    void UpdateAPInfo(const std::vector<APInfo>& apList) {
        m_apList = apList;
    }

    // 距離計算
    double CalculateDistance(const Vector& pos1, const Vector& pos2) {
        return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
    }

    // 伝送レート計算（距離に基づく）
    double CalculateTransmissionRate(const Vector& userPos, const Vector& apPos) {
        double distance = CalculateDistance(userPos, apPos);
        // 802.11nの仕様に基づく伝送レート
        if (distance < 5.0) return 150.0;  // Mbps
        else if (distance < 10.0) return 130.0;
        else if (distance < 15.0) return 100.0;
        else if (distance < 20.0) return 65.0;
        else if (distance < 25.0) return 30.0;
        else return 6.5;
    }

    // スループット計算（幾何平均）
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

    // 最適AP選択アルゴリズム
    std::pair<uint32_t, Vector> SelectOptimalAP(const Vector& userPos) {
        std::vector<uint32_t> candidates;

        // 候補AP選択（距離閾値）
        for (const auto& ap : m_apList) {
            double distance = CalculateDistance(userPos, ap.position);
            if (distance <= m_dThreshold) {
                candidates.push_back(ap.apId);
            }
        }

        if (candidates.empty()) {
            return std::make_pair(0, Vector(0, 0, 0)); // 接続先APなし
        }

        // スコア計算
        std::vector<std::pair<uint32_t, double>> apScores;
        double thetaMax = 0, thetaMin = 1e9;
        double dMax = 0, dMin = 1e9;
        uint32_t nMax = 0;

        // 最大最小値計算
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

        // スコア計算（6-10）  
        for (uint32_t apId : candidates) {
            const APInfo& ap = m_apList[apId];
            double newRate = CalculateTransmissionRate(userPos, ap.position);
            double throughput = CalculateThroughput(ap, newRate);
            double distance = CalculateDistance(userPos, ap.position);

            // スコア計算
            double scoreTheta = (thetaMax == thetaMin) ? 1.0 : 
                               (throughput - thetaMin) / (thetaMax - thetaMin);
            double scoreDist = (dMax == dMin) ? 1.0 : 
                              1.0 - (distance - dMin) / (dMax - dMin);
            double scoreChan = 1.0 - ap.channelUtilization; // チャンネル利用率が低いほど良い
            double scoreUsers = (nMax == 0) ? 1.0 : 
                               1.0 - (double)ap.connectedUsers / nMax;

            double totalScore = m_weights[0] * scoreTheta + 
                               m_weights[1] * scoreDist + 
                               m_weights[2] * scoreChan + 
                               m_weights[3] * scoreUsers;

            apScores.push_back(std::make_pair(apId, totalScore));
        }

        // スコア計算
        std::sort(apScores.begin(), apScores.end(), 
                 [](const auto& a, const auto& b) { return a.second > b.second; });

        // 最適APの選択（移動平均フィルタ適用）
        return std::make_pair(apScores[0].first, Vector(0, 0, 0));
    }
};

// 結果出力
static std::ofstream* g_resultFile = nullptr;
static NodeContainer* g_apNodes = nullptr;
static NodeContainer* g_staNodes = nullptr;
static uint32_t g_nAPs = 0;
static uint32_t g_nUsers = 0;

// AP選択結果出力
void PrintAPSelectionResults() {
    if (!g_resultFile || !g_apNodes || !g_staNodes) return;
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Ptr<MobilityModel> mobility = g_staNodes->Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();

        // AP情報の更新
        std::vector<APInfo> apInfo(g_nAPs);
        for (uint32_t j = 0; j < g_nAPs; ++j) {
            apInfo[j].apId = j;
            apInfo[j].position = g_apNodes->Get(j)->GetObject<MobilityModel>()->GetPosition();
            apInfo[j].connectedUsers = 3; // 仮の値
            apInfo[j].channelUtilization = 0.5; // 仮の値
            apInfo[j].channel = j % 3; // 3つのチャネル
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
    // パラメータ設定
    uint32_t nAPs = 5;
    uint32_t nUsers = 18; // 4+2+3+4+5
    double simTime = 60.0; // 60秒

    // 結果出力用ファイルの設定
    g_nAPs = nAPs;
    g_nUsers = nUsers;

    // コマンドライン引数の処理
    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.Parse(argc, argv);

    // パラメータ設定
    LogComponentEnable("WiFiAPSelection", LOG_LEVEL_INFO);

    // APノードの作成
    NodeContainer apNodes;
    apNodes.Create(nAPs);
    g_apNodes = &apNodes;
    
    NodeContainer staNodes;
    staNodes.Create(nUsers);
    g_staNodes = &staNodes;

    // WiFi設定
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");
    // wifi.SetRemoteStationManager("ns3::MinstrelWifiManager");

    // PHY設定（Default PHYモデルを使用）
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    // MAC設定
    WifiMacHelper mac;
    Ssid ssid = Ssid("wifi-network");

    // AP設定
    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    for (uint32_t i = 0; i < nAPs; ++i) {
        NetDeviceContainer device = wifi.Install(phy, mac, apNodes.Get(i));
        apDevices.Add(device);
    }

    // STA設定
    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    staDevices = wifi.Install(phy, mac, staNodes);

    // 移動端末設定
    MobilityHelper mobility;

    // AP配置（30m×30mの正方形）
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(7.5, 7.5, 0.0));   // AP0
    apPositionAlloc->Add(Vector(22.5, 7.5, 0.0));  // AP1
    apPositionAlloc->Add(Vector(7.5, 22.5, 0.0));  // AP2
    apPositionAlloc->Add(Vector(22.5, 22.5, 0.0)); // AP3
    apPositionAlloc->Add(Vector(15.0, 15.0, 0.0)); // AP4 (ä¸­å¤®)

    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // STA設定
    Ptr<ListPositionAllocator> staPositionAlloc = CreateObject<ListPositionAllocator>();

    // AP0接続の4端末
    staPositionAlloc->Add(Vector(5.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(5.0, 10.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 10.0, 0.0));

    // AP1接続の2端末
    staPositionAlloc->Add(Vector(20.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 5.0, 0.0));

    // AP2接続の3端末
    staPositionAlloc->Add(Vector(5.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(5.0, 25.0, 0.0));

    // AP3接続の4端末
    staPositionAlloc->Add(Vector(20.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(20.0, 25.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 25.0, 0.0));

    // AP4接続の5端末
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

    // アプリケーション設定
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);

    // IPアドレス割り当て
    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

    // UDPエコーサーバー設定
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

    // FlowMonitor設定
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // NetAnim設定
    AnimationInterface anim("kamikawa-animation.xml");

    // 結果ファイル設定
    std::ofstream resultFile("ap_selection_results.txt");
    g_resultFile = &resultFile;
    resultFile << "Time(s)\tUserID\tPosition(x,y)\tSelectedAP\tThroughput(Mbps)" << std::endl;

    // シミュレーション実行
    Simulator::Stop(Seconds(simTime));

    // 定期的なAP選択結果出力（定期的にファイルに書き込む）
    Simulator::Schedule(Seconds(5.0), &PrintAPSelectionResults);

    Simulator::Run();

    // FlowMonitor結果出力
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