// #include "ns3/core-module.h"
// #include "ns3/network-module.h"
// #include "ns3/mobility-module.h"
// #include "ns3/wifi-module.h"
// #include "ns3/internet-module.h"
// #include "ns3/applications-module.h"
// #include "ns3/flow-monitor-module.h"
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <fstream>

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

// AP情報を格納する構造体
struct APInfo {
    uint32_t apId;
    Vector position;
    uint32_t connectedUsers;
    double channelUtilization;
    std::vector<double> userRates;
    uint32_t channel;
};

// ユーザ情報を格納する構造体
struct UserInfo {
    uint32_t userId;
    Vector position;
    uint32_t connectedAP;
    double throughput;
};

class APSelectionAlgorithm {
private:
    std::vector<APInfo> m_apList;
    double m_dThreshold;  // 移動可能距離
    double m_thetaThreshold;  // 最低要求スループット
    std::vector<double> m_weights; // スコア重み [w1, w2, w3, w4]

public:
    APSelectionAlgorithm(double dTh, double thetaTh) 
        : m_dThreshold(dTh), m_thetaThreshold(thetaTh) {
        m_weights = {0.4, 0.3, 0.2, 0.1}; // デフォルト重み
    }

    void UpdateAPInfo(const std::vector<APInfo>& apList) {
        m_apList = apList;
    }

    // 距離計算
    double CalculateDistance(const Vector& pos1, const Vector& pos2) {
        return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
    }

    // 伝送レート計算（距離ベース）
    double CalculateTransmissionRate(const Vector& userPos, const Vector& apPos) {
        double distance = CalculateDistance(userPos, apPos);
        // 802.11nの簡単な距離-レートモデル
        if (distance < 5.0) return 150.0;  // Mbps
        else if (distance < 10.0) return 130.0;
        else if (distance < 15.0) return 100.0;
        else if (distance < 20.0) return 65.0;
        else if (distance < 25.0) return 30.0;
        else return 6.5;
    }

    // スループット計算（調和平均）
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
        
        // 候補AP選択（式1,2）
        for (const auto& ap : m_apList) {
            double distance = CalculateDistance(userPos, ap.position);
            if (distance <= m_dThreshold) {
                candidates.push_back(ap.apId);
            }
        }

        if (candidates.empty()) {
            return std::make_pair(0, Vector(0, 0, 0)); // 接続不可
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

        // 各APのスコア計算（式6-10）
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
            double scoreChan = 1.0 - ap.channelUtilization; // 使用率が低いほど良い
            double scoreUsers = (nMax == 0) ? 1.0 : 
                               1.0 - (double)ap.connectedUsers / nMax;

            double totalScore = m_weights[0] * scoreTheta + 
                               m_weights[1] * scoreDist + 
                               m_weights[2] * scoreChan + 
                               m_weights[3] * scoreUsers;

            apScores.push_back(std::make_pair(apId, totalScore));
        }

        // スコア順でソート
        std::sort(apScores.begin(), apScores.end(), 
                 [](const auto& a, const auto& b) { return a.second > b.second; });

        // 最適APを返す（移動ベクトルは0とする）
        return std::make_pair(apScores[0].first, Vector(0, 0, 0));
    }
};

int main(int argc, char *argv[]) {
    // シミュレーション設定
    uint32_t nAPs = 5;
    uint32_t nUsers = 18; // 4+2+3+4+5
    double simTime = 60.0; // 60秒
    
    // コマンドライン引数
    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.Parse(argc, argv);

    // ログ設定
    LogComponentEnable("WiFiAPSelection", LOG_LEVEL_INFO);

    // ノード作成
    NodeContainer apNodes;
    apNodes.Create(nAPs);
    
    NodeContainer staNodes;
    staNodes.Create(nUsers);

    // WiFi設定
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n_2_4GHZ);
    wifi.SetRemoteStationManager("ns3::MinstrelWifiManager");

    // PHY設定
    YansWifiChannelHelper channel;
    YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    phy.SetChannel(channel.Create());

    // MAC設定
    WifiMacHelper mac;
    Ssid ssid = Ssid("wifi-network");

    // APの設定
    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    for (uint32_t i = 0; i < nAPs; ++i) {
        NetDeviceContainer device = wifi.Install(phy, mac, apNodes.Get(i));
        apDevices.Add(device);
    }

    // STAの設定
    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    staDevices = wifi.Install(phy, mac, staNodes);

    // 移動モデル設定
    MobilityHelper mobility;

    // AP配置（30m×30mエリアに等間隔）
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(7.5, 7.5, 0.0));   // AP0
    apPositionAlloc->Add(Vector(22.5, 7.5, 0.0));  // AP1
    apPositionAlloc->Add(Vector(7.5, 22.5, 0.0));  // AP2
    apPositionAlloc->Add(Vector(22.5, 22.5, 0.0)); // AP3
    apPositionAlloc->Add(Vector(15.0, 15.0, 0.0)); // AP4 (中央)

    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // STA初期配置（変更可能）
    Ptr<ListPositionAllocator> staPositionAlloc = CreateObject<ListPositionAllocator>();
    
    // AP0周辺に4人
    staPositionAlloc->Add(Vector(5.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(5.0, 10.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 10.0, 0.0));
    
    // AP1周辺に2人
    staPositionAlloc->Add(Vector(20.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(25.0,#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WiFiAPSelection");

// AP情報を格納する構造体
struct APInfo {
    uint32_t apId;
    Vector position;
    uint32_t connectedUsers;
    double channelUtilization;
    std::vector<double> userRates;
    uint32_t channel;
};

// ユーザ情報を格納する構造体
struct UserInfo {
    uint32_t userId;
    Vector position;
    uint32_t connectedAP;
    double throughput;
};

class APSelectionAlgorithm {
private:
    std::vector<APInfo> m_apList;
    double m_dThreshold;  // 移動可能距離
    double m_thetaThreshold;  // 最低要求スループット
    std::vector<double> m_weights; // スコア重み [w1, w2, w3, w4]

public:
    APSelectionAlgorithm(double dTh, double thetaTh) 
        : m_dThreshold(dTh), m_thetaThreshold(thetaTh) {
        m_weights = {0.4, 0.3, 0.2, 0.1}; // デフォルト重み
    }

    void UpdateAPInfo(const std::vector<APInfo>& apList) {
        m_apList = apList;
    }

    // 距離計算
    double CalculateDistance(const Vector& pos1, const Vector& pos2) {
        return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
    }

    // 伝送レート計算（距離ベース）
    double CalculateTransmissionRate(const Vector& userPos, const Vector& apPos) {
        double distance = CalculateDistance(userPos, apPos);
        // 802.11nの簡単な距離-レートモデル
        if (distance < 5.0) return 150.0;  // Mbps
        else if (distance < 10.0) return 130.0;
        else if (distance < 15.0) return 100.0;
        else if (distance < 20.0) return 65.0;
        else if (distance < 25.0) return 30.0;
        else return 6.5;
    }

    // スループット計算（調和平均）
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
        
        // 候補AP選択（式1,2）
        for (const auto& ap : m_apList) {
            double distance = CalculateDistance(userPos, ap.position);
            if (distance <= m_dThreshold) {
                candidates.push_back(ap.apId);
            }
        }

        if (candidates.empty()) {
            return std::make_pair(0, Vector(0, 0, 0)); // 接続不可
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

        // 各APのスコア計算（式6-10）
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
            double scoreChan = 1.0 - ap.channelUtilization; // 使用率が低いほど良い
            double scoreUsers = (nMax == 0) ? 1.0 : 
                               1.0 - (double)ap.connectedUsers / nMax;

            double totalScore = m_weights[0] * scoreTheta + 
                               m_weights[1] * scoreDist + 
                               m_weights[2] * scoreChan + 
                               m_weights[3] * scoreUsers;

            apScores.push_back(std::make_pair(apId, totalScore));
        }

        // スコア順でソート
        std::sort(apScores.begin(), apScores.end(), 
                 [](const auto& a, const auto& b) { return a.second > b.second; });

        // 最適APを返す（移動ベクトルは0とする）
        return std::make_pair(apScores[0].first, Vector(0, 0, 0));
    }
};

int main(int argc, char *argv[]) {
    // シミュレーション設定
    uint32_t nAPs = 5;
    uint32_t nUsers = 18; // 4+2+3+4+5
    double simTime = 60.0; // 60秒
    
    // コマンドライン引数
    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.Parse(argc, argv);

    // ログ設定
    LogComponentEnable("WiFiAPSelection", LOG_LEVEL_INFO);

    // ノード作成
    NodeContainer apNodes;
    apNodes.Create(nAPs);
    
    NodeContainer staNodes;
    staNodes.Create(nUsers);

    // WiFi設定
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n_2_4GHZ);
    wifi.SetRemoteStationManager("ns3::MinstrelWifiManager");

    // PHY設定
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    // MAC設定
    WifiMacHelper mac;
    Ssid ssid = Ssid("wifi-network");

    // APの設定
    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    for (uint32_t i = 0; i < nAPs; ++i) {
        NetDeviceContainer device = wifi.Install(phy, mac, apNodes.Get(i));
        apDevices.Add(device);
    }

    // STAの設定
    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    staDevices = wifi.Install(phy, mac, staNodes);

    // 移動モデル設定
    MobilityHelper mobility;

    // AP配置（30m×30mエリアに等間隔）
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(7.5, 7.5, 0.0));   // AP0
    apPositionAlloc->Add(Vector(22.5, 7.5, 0.0));  // AP1
    apPositionAlloc->Add(Vector(7.5, 22.5, 0.0));  // AP2
    apPositionAlloc->Add(Vector(22.5, 22.5, 0.0)); // AP3
    apPositionAlloc->Add(Vector(15.0, 15.0, 0.0)); // AP4 (中央)

    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // STA初期配置（変更可能）
    Ptr<ListPositionAllocator> staPositionAlloc = CreateObject<ListPositionAllocator>();
    
    // AP0周辺に4人
    staPositionAlloc->Add(Vector(5.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(5.0, 10.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 10.0, 0.0));
    
    // AP1周辺に2人
    staPositionAlloc->Add(Vector(20.0, 5.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 5.0, 0.0));
    
    // AP2周辺に3人
    staPositionAlloc->Add(Vector(5.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(5.0, 25.0, 0.0));
    
    // AP3周辺に4人
    staPositionAlloc->Add(Vector(20.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(20.0, 25.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 25.0, 0.0));
    
    // AP4周辺に5人
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

    // インターネットプロトコルスタック
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);

    // IPアドレス割り当て
    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

    // トラフィック生成（UDPエコーアプリケーション）
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

    // AP選択アルゴリズム初期化（移動）
    // APSelectionAlgorithm algorithm(15.0, 10.0); // 15m移動可能、10Mbps最低要求

    // シミュレーション実行
    Simulator::Stop(Seconds(simTime));
    
    // 定期的なAP選択結果出力用のグローバル変数
    std::ofstream* resultFilePtr = new std::ofstream("ap_selection_results.txt");
    *resultFilePtr << "Time(s)\tUserID\tPosition(x,y)\tSelectedAP\tThroughput(Mbps)" << std::endl;

    // コールバック関数の定義
    auto printAPSelectionResults = [=]() {
        // AP選択アルゴリズムを実行し、結果を出力
        for (uint32_t i = 0; i < nUsers; ++i) {
            Ptr<MobilityModel> mobility = staNodes.Get(i)->GetObject<MobilityModel>();
            Vector pos = mobility->GetPosition();
            
            // 簡単なAP情報生成（実際の実装では実際の統計を使用）
            std::vector<APInfo> apInfo(nAPs);
            for (uint32_t j = 0; j < nAPs; ++j) {
                apInfo[j].apId = j;
                apInfo[j].position = apNodes.Get(j)->GetObject<MobilityModel>()->GetPosition();
                apInfo[j].connectedUsers = 3; // 簡略化
                apInfo[j].channelUtilization = 0.5; // 簡略化
                apInfo[j].channel = j % 3; // 3チャネル
            }
            
            APSelectionAlgorithm tempAlgorithm(15.0, 10.0);
            tempAlgorithm.UpdateAPInfo(apInfo);
            auto result = tempAlgorithm.SelectOptimalAP(pos);
            
            *resultFilePtr << "5.0\t" << i << "\t(" << pos.x << "," << pos.y << ")\t" 
                          << result.first << "\t" << "50.0" << std::endl;
        }
        resultFilePtr->flush();
    };

    Simulator::Schedule(Seconds(5.0), printAPSelectionResults);

    Simulator::Run();

    // FlowMonitor統計出力
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::cout << "\n=== Flow Monitor Statistics ===" << std::endl;
    for (auto& flow : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        std::cout << "Flow " << flow.first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")" << std::endl;
        std::cout << "  Throughput: " << flow.second.rxBytes * 8.0 / (flow.second.timeLastRxPacket.GetSeconds() - flow.second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps" << std::endl;
    }

    resultFilePtr->close();
    delete resultFilePtr;
    Simulator::Destroy();

    std::cout << "\nSimulation completed. Results saved to ap_selection_results.txt" << std::endl;

    return 0;
} 5.0, 0.0));
    
    // AP2周辺に3人
    staPositionAlloc->Add(Vector(5.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(10.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(5.0, 25.0, 0.0));
    
    // AP3周辺に4人
    staPositionAlloc->Add(Vector(20.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 20.0, 0.0));
    staPositionAlloc->Add(Vector(20.0, 25.0, 0.0));
    staPositionAlloc->Add(Vector(25.0, 25.0, 0.0));
    
    // AP4周辺に5人
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

    // インターネットプロトコルスタック
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);

    // IPアドレス割り当て
    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

    // トラフィック生成（UDPエコーアプリケーション）
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

    AnimationInterface anim("kamikawa-animation.xml");

    // AP選択アルゴリズム初期化
    APSelectionAlgorithm algorithm(15.0, 10.0); // 15m移動可能、10Mbps最低要求

    // シミュレーション実行
    Simulator::Stop(Seconds(simTime));
    
    // 定期的なAP選択結果出力
    std::ofstream resultFile("ap_selection_results.txt");
    resultFile << "Time(s)\tUserID\tPosition(x,y)\tSelectedAP\tThroughput(Mbps)" << std::endl;

    Simulator::Schedule(Seconds(5.0), [&]() {
        // ここでAP選択アルゴリズムを実行し、結果を出力
        for (uint32_t i = 0; i < nUsers; ++i) {
            Ptr<MobilityModel> mobility = staNodes.Get(i)->GetObject<MobilityModel>();
            Vector pos = mobility->GetPosition();
            
            // 簡単なAP情報生成（実際の実装では実際の統計を使用）
            std::vector<APInfo> apInfo(nAPs);
            for (uint32_t j = 0; j < nAPs; ++j) {
                apInfo[j].apId = j;
                apInfo[j].position = apNodes.Get(j)->GetObject<MobilityModel>()->GetPosition();
                apInfo[j].connectedUsers = 3; // 簡略化
                apInfo[j].channelUtilization = 0.5; // 簡略化
                apInfo[j].channel = j % 3; // 3チャネル
            }
            
            algorithm.UpdateAPInfo(apInfo);
            auto result = algorithm.SelectOptimalAP(pos);
            
            resultFile << "5.0\t" << i << "\t(" << pos.x << "," << pos.y << ")\t" 
                      << result.first << "\t" << "50.0" << std::endl;
        }
    });

    Simulator::Run();

    // FlowMonitor統計出力
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::cout << "\n=== Flow Monitor Statistics ===" << std::endl;
    for (auto& flow : stats) {
        std::cout << "Flow " << flow.first << " (";
        classifier->FindFlow(flow.first).Print(std::cout);
        std::cout << ")" << std::endl;
        std::cout << "  Throughput: " << flow.second.rxBytes * 8.0 / (flow.second.timeLastRxPacket.GetSeconds() - flow.second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps" << std::endl;
    }

    resultFile.close();
    Simulator::Destroy();

    std::cout << "\nSimulation completed. Results saved to ap_selection_results.txt" << std::endl;

    return 0;
}