#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

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
    uint32_t apId; // APのID
    Vector position; // APの位置
    uint32_t connectedUsers; // 接続中のユーザー数
    double channelUtilization; // チャンネル利用率
    std::vector<double> userRates; // ユーザーごとのスループット
    uint32_t channel; // チャンネル番号
};

// Userの情報
struct UserInfo {
    uint32_t userId; // ユーザーID
    Vector position; // ユーザーの位置
    uint32_t connectedAP; // 接続中のAP
    double throughput; // ユーザーのスループット
};

// AP選択結果を格納する構造体
struct APSelectionResult {
    uint32_t userId;
    Vector userPosition;
    uint32_t selectedAP;
    double expectedThroughput;
    double distance;
    double score;
    std::vector<std::pair<uint32_t, double>> allScores; // 全APのスコア
};

class APSelectionAlgorithm { //AP選択アルゴリズム  
private:
    std::vector<APInfo> m_apList;
    double m_dThreshold;  // APの距離閾値
    double m_thetaThreshold;  // 最低要求スループット
    std::vector<double> m_weights; // 各指標の重み [w1, w2, w3, w4]

public:
    APSelectionAlgorithm(double dTh, double thetaTh) 
        : m_dThreshold(dTh), m_thetaThreshold(thetaTh) {
        // デフォルト重み設定（シンプルなケースでは距離とスループットを重視）
        m_weights = {0.5, 0.4, 0.1, 0.0}; // スループット、距離、チャンネル、ユーザー数
    }
    
    const std::vector<double>& getWeights() const { return m_weights; }

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
        // 802.11nの仕様に基づく伝送レート（シンプル化）
        if (distance < 5.0) return 150.0;  // Mbps
        else if (distance < 10.0) return 130.0;
        else if (distance < 15.0) return 100.0;
        else if (distance < 20.0) return 65.0;
        else if (distance < 25.0) return 30.0;
        else return 6.5;
    }

    // スループット計算（1ユーザーなので単純）
    double CalculateThroughput(const APInfo& ap, double newRate) {
        if (ap.userRates.empty()) {
            return newRate;
        }
        
        // 調和平均（複数ユーザーがいる場合）
        double sum = 0.0;
        for (double rate : ap.userRates) {
            sum += 1.0 / rate;
        }
        sum += 1.0 / newRate;
        
        return (ap.userRates.size() + 1) / sum;
    }

    // 最適AP選択アルゴリズム（詳細な結果を返す）
    APSelectionResult SelectOptimalAPDetailed(const Vector& userPos, uint32_t userId) {
        APSelectionResult result;
        result.userId = userId;
        result.userPosition = userPos;
        result.selectedAP = 0;
        result.expectedThroughput = 0.0;
        result.distance = 0.0;
        result.score = 0.0;

        std::vector<uint32_t> candidates;

        // 候補AP選択（距離閾値）
        for (const auto& ap : m_apList) {
            double distance = CalculateDistance(userPos, ap.position);
            if (distance <= m_dThreshold) {
                candidates.push_back(ap.apId);
            }
        }

        if (candidates.empty()) {
            return result; // 接続先APなし
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

        // スコア計算
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
            result.allScores.push_back(std::make_pair(apId, totalScore));
        }

        // スコア順にソート
        std::sort(apScores.begin(), apScores.end(), 
                 [](const auto& a, const auto& b) { return a.second > b.second; });

        // 最適APの選択
        result.selectedAP = apScores[0].first;
        result.score = apScores[0].second;
        result.distance = CalculateDistance(userPos, m_apList[result.selectedAP].position);
        result.expectedThroughput = CalculateThroughput(m_apList[result.selectedAP], 
            CalculateTransmissionRate(userPos, m_apList[result.selectedAP].position));

        return result;
    }

    // 簡単なAP選択（互換性のため）
    std::pair<uint32_t, Vector> SelectOptimalAP(const Vector& userPos) {
        APSelectionResult result = SelectOptimalAPDetailed(userPos, 0);
        return std::make_pair(result.selectedAP, Vector(0, 0, 0));
    }
};

// グローバル変数
static std::ofstream* g_resultFile = nullptr;
static NodeContainer* g_apNodes = nullptr;
static NodeContainer* g_staNodes = nullptr;
static uint32_t g_nAPs = 0;
static uint32_t g_nUsers = 0;
static APSelectionAlgorithm* g_algorithm = nullptr;
static std::vector<APInfo> g_apInfoList;

// 初期状態の表示
void PrintInitialState() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "       シンプル WiFi APセレクションシミュレーション 初期状態" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    // シミュレーション設定
    std::cout << "\n【シミュレーション設定】" << std::endl;
    std::cout << "  AP数: " << g_nAPs << "台" << std::endl;
    std::cout << "  ユーザー数: " << g_nUsers << "台" << std::endl;
    std::cout << "  シミュレーション時間: 60秒" << std::endl;
    std::cout << "  シミュレーション範囲: 30m × 30m" << std::endl;

    // AP配置情報
    std::cout << "\n【AP配置情報】" << std::endl;
    for (uint32_t i = 0; i < g_nAPs; ++i) {
        Vector pos = g_apNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        std::cout << "  AP" << i << ": 位置(" << std::fixed << std::setprecision(1) 
                  << pos.x << "m, " << pos.y << "m)" 
                  << ", 接続ユーザー数:" << g_apInfoList[i].connectedUsers << "台"
                  << ", チャンネル利用率:" << std::setprecision(1) << (g_apInfoList[i].channelUtilization * 100) << "%" << std::endl;
    }

    // ユーザー初期位置
    std::cout << "\n【ユーザー初期配置】" << std::endl;
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Vector pos = g_staNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        std::cout << "  ユーザー" << i << ": 位置(" 
                  << std::fixed << std::setprecision(1)
                  << pos.x << "m, " << pos.y << "m)" << std::endl;
    }

    // アルゴリズム設定
    std::cout << "\n【AP選択アルゴリズム設定】" << std::endl;
    std::cout << "  距離閾値: 25.0m" << std::endl;
    std::cout << "  最低要求スループット: 10.0Mbps" << std::endl;
    
    // 実際の重みの値を表示
    if (g_algorithm) {
        const std::vector<double>& weights = g_algorithm->getWeights();
        std::cout << "  スコア重み: [スループット:" << std::fixed << std::setprecision(1) << weights[0] 
                  << ", 距離:" << weights[1] 
                  << ", チャンネル:" << weights[2] 
                  << ", ユーザー数:" << weights[3] << "]" << std::endl;
    }
    
    std::cout << std::string(80, '=') << std::endl;
}

// AP選択結果の詳細表示
void PrintAPSelectionResults() {
    if (!g_resultFile || !g_apNodes || !g_staNodes || !g_algorithm) return;
    
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "              AP選択アルゴリズム実行結果" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    std::vector<APSelectionResult> results;
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Ptr<MobilityModel> mobility = g_staNodes->Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();

        APSelectionResult result = g_algorithm->SelectOptimalAPDetailed(pos, i);
        results.push_back(result);
        
        // ファイルにも出力（元の形式を維持）
        *g_resultFile << "5.0\t" << i << "\t(" << pos.x << "," << pos.y << ")\t" 
                     << result.selectedAP << "\t" << result.expectedThroughput << std::endl;
    }

    // ユーザーごとの詳細結果表示
    std::cout << "\n【各ユーザーのAP選択結果】" << std::endl;
    for (const auto& result : results) {
        std::cout << "\n--- ユーザー" << result.userId << " ---" << std::endl;
        std::cout << "  現在位置: (" << std::fixed << std::setprecision(1) 
                  << result.userPosition.x << "m, " << result.userPosition.y << "m)" << std::endl;
        std::cout << "  選択されたAP: AP" << result.selectedAP << std::endl;
        std::cout << "  APまでの距離: " << std::setprecision(1) << result.distance << "m" << std::endl;
        std::cout << "  予想スループット: " << std::setprecision(1) << result.expectedThroughput << "Mbps" << std::endl;
        std::cout << "  総合スコア: " << std::setprecision(3) << result.score << std::endl;
        
        // 全APのスコア表示（1APなのでシンプル）
        std::cout << "  APスコア詳細:" << std::endl;
        for (const auto& score : result.allScores) {
            Vector apPos = g_apNodes->Get(score.first)->GetObject<MobilityModel>()->GetPosition();
            double dist = sqrt(pow(result.userPosition.x - apPos.x, 2) + 
                             pow(result.userPosition.y - apPos.y, 2));
            std::cout << "    AP" << score.first << ": スコア=" << std::setprecision(3) << score.second
                      << " (距離:" << std::setprecision(1) << dist << "m)" << std::endl;
        }
    }

    // 統計情報（1APなので簡単）
    std::cout << "\n【AP選択統計】" << std::endl;
    std::cout << "  AP0: " << g_nUsers << "ユーザー選択 (100.0%)" << std::endl;

    // 平均距離とスループット
    double totalDistance = 0.0, totalThroughput = 0.0;
    for (const auto& result : results) {
        totalDistance += result.distance;
        totalThroughput += result.expectedThroughput;
    }
    
    std::cout << "\n【性能指標】" << std::endl;
    std::cout << "  平均AP距離: " << std::setprecision(1) << (totalDistance / g_nUsers) << "m" << std::endl;
    std::cout << "  平均予想スループット: " << std::setprecision(1) << (totalThroughput / g_nUsers) << "Mbps" << std::endl;

    std::cout << std::string(80, '=') << std::endl;
    
    g_resultFile->flush();
}

int main(int argc, char *argv[]) {
    // パラメータ設定（シンプル化）
    uint32_t nAPs = 1;    // AP1台
    uint32_t nUsers = 1;  // ユーザー1台
    double simTime = 60.0; // 60秒

    // グローバル変数設定
    g_nAPs = nAPs;
    g_nUsers = nUsers;

    // コマンドライン引数の処理
    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.Parse(argc, argv);

    // ログ設定
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

    // PHY設定
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    // MAC設定
    WifiMacHelper mac;
    Ssid ssid = Ssid("simple-wifi-network");

    // AP設定
    NetDeviceContainer apDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    apDevices = wifi.Install(phy, mac, apNodes);

    // STA設定
    NetDeviceContainer staDevices;
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    staDevices = wifi.Install(phy, mac, staNodes);

    // 移動端末設定
    MobilityHelper mobility;

    // AP配置（中央に1台）
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(15.0, 15.0, 0.0));   // AP0 (中央)

    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // STA配置（APの近くに1台）
    Ptr<ListPositionAllocator> staPositionAlloc = CreateObject<ListPositionAllocator>();
    staPositionAlloc->Add(Vector(12.0, 12.0, 0.0));  // ユーザー0（APから少し離れた位置）

    mobility.SetPositionAllocator(staPositionAlloc);
    
    // ユーザーにシンプルな移動を設定
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Bounds", RectangleValue(Rectangle(0, 30, 0, 30)),
                              "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
                              
    mobility.Install(staNodes);

    // AP情報の初期化（1APなのでシンプル）
    g_apInfoList.resize(nAPs);
    g_apInfoList[0].apId = 0;
    g_apInfoList[0].position = apNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
    g_apInfoList[0].connectedUsers = 0; // 初期は接続なし
    g_apInfoList[0].channelUtilization = 0.1; // 低利用率
    g_apInfoList[0].channel = 0; // チャンネル0

    // アルゴリズム初期化（距離閾値を広めに）
    APSelectionAlgorithm algorithm(25.0, 10.0);
    algorithm.UpdateAPInfo(g_apInfoList);
    g_algorithm = &algorithm;

    // 初期状態表示
    PrintInitialState();

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
    AnimationInterface anim("simple-kamikawa-animation.xml");

    // 結果ファイル設定
    std::ofstream resultFile("simple_ap_selection_results.txt");
    g_resultFile = &resultFile;
    resultFile << "Time(s)\tUserID\tPosition(x,y)\tSelectedAP\tThroughput(Mbps)" << std::endl;

    // シミュレーション実行
    Simulator::Stop(Seconds(simTime));

    // AP選択結果出力（シミュレーション開始時）
    Simulator::Schedule(Seconds(5.0), &PrintAPSelectionResults);

    Simulator::Run();

    // FlowMonitor結果出力
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "                シミュレーション最終結果" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    std::cout << "\n【実測通信性能】" << std::endl;
    
    double totalRealThroughput = 0.0;
    uint32_t flowCount = 0;
    
    for (auto& flow : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        if (flow.second.timeLastRxPacket.GetSeconds() > flow.second.timeFirstTxPacket.GetSeconds()) {
            double throughput = flow.second.rxBytes * 8.0 / 
                (flow.second.timeLastRxPacket.GetSeconds() - flow.second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024;
            std::cout << "  フロー " << flow.first << " (" << t.sourceAddress << " -> " << t.destinationAddress << "): "
                      << std::fixed << std::setprecision(2) << throughput << " Mbps" << std::endl;
            totalRealThroughput += throughput;
            flowCount++;
        }
    }
    
    if (flowCount > 0) {
        std::cout << "\n  実測スループット: " << std::setprecision(2) << (totalRealThroughput / flowCount) << " Mbps" << std::endl;
    }

    std::cout << "\n【シミュレーション完了】" << std::endl;
    std::cout << "  結果ファイル: simple_ap_selection_results.txt" << std::endl;
    std::cout << "  アニメーションファイル: simple-kamikawa-animation.xml" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    resultFile.close();
    Simulator::Destroy();

    return 0;
}