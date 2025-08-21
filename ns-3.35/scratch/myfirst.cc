#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MiyataAlgorithm");

// --- ヘルパー関数の定義 ---

// 距離計算関数
double CalculateEuclideanDistance(Vector a, Vector b)
{
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

// 距離(m) -> 伝送レート(Mbps)のマッピング (論文のTable 1)
double GetRateFromDistance(double distance)
{
    if (distance <= 5) return 54.0;
    if (distance <= 7) return 48.0;
    if (distance <= 9) return 36.0;
    if (distance <= 20) return 24.0;
    if (distance <= 25) return 18.0;
    if (distance <= 40) return 12.0;
    if (distance <= 50) return 9.0;
    if (distance <= 60) return 6.0;
    return 0.0; // 範囲外
}

// 式(9): 接続後のAPスループットを計算
// θ^after_{a,m} = (n_a + 1) / (n_a/θ^before_a + 1/b_{new,a})
// 論文の式(9)の詳細な実装
double CalculateApThroughputAfter(double theta_before, int n_a, double b_new_a)
{
    if (theta_before <= 0 || b_new_a <= 0 || n_a < 0)
    {
        NS_LOG_WARN("Invalid parameters: theta_before=" << theta_before <<
                   ", n_a=" << n_a << ", b_new_a=" << b_new_a);
        return 0.0;
    }
    
    // 分母の計算: n_a/θ^before_a + 1/b_{new,a}
    double denominator = (double)n_a / theta_before + 1.0 / b_new_a;
    
    // 分子の計算: n_a + 1
    double numerator = (double)(n_a + 1);
    
    double result = numerator / denominator;
    
    NS_LOG_DEBUG("式(9)計算: θ^before=" << theta_before <<
                ", n_a=" << n_a << ", b_new=" << b_new_a <<
                " → θ^after=" << result);
    
    return result;
}

// 式(10): 新規ユーザーのスループットを計算
// θ_{new,a} = θ^after_{a,m} / (n_a + 1)
// 論文の式(10)の詳細な実装
double CalculateNewUserThroughput(double theta_after, int n_a)
{
    if (theta_after <= 0 || n_a < 0)
    {
        NS_LOG_WARN("Invalid parameters: theta_after=" << theta_after << ", n_a=" << n_a);
        return 0.0;
    }
    
    double result = theta_after / (double)(n_a + 1);
    
    NS_LOG_DEBUG("式(10)計算: θ^after=" << theta_after <<
                ", n_a=" << n_a << " → θ_new=" << result);
    
    return result;
}

// ベクトルの正規化
Vector NormalizeVector(Vector v)
{
    double magnitude = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (magnitude == 0) return Vector(0, 0, 0);
    return Vector(v.x / magnitude, v.y / magnitude, v.z / magnitude);
}

// ベクトルのスカラー乗算（修正のために追加）
Vector MultiplyVector(Vector v, double scalar)
{
    return Vector(v.x * scalar, v.y * scalar, v.z * scalar);
}

// 候補構造体
struct Candidate
{
    uint32_t apIndex;
    Vector newPosition;
    double movingDistance;
    double systemThroughput;
    double newUserThroughput;
    double apThroughputAfter;
};

int main(int argc, char *argv[])
{
    // ログレベル設定
    LogComponentEnable("MiyataAlgorithm", LOG_LEVEL_INFO);
    
    // --- 1. 初期設定 ---
    uint32_t nWifi = 3;  // 既存ユーザー数
    uint32_t nAps = 2;   // AP数

    // ノード作成
    NodeContainer staNodes;
    staNodes.Create(nWifi);
    NodeContainer apNodes;
    apNodes.Create(nAps);

    // WiFi設定
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager");

    // SSID設定
    Ssid ssidAp1 = Ssid("ns3-wifi-ap1");
    Ssid ssidAp2 = Ssid("ns3-wifi-ap2");
    
    WifiMacHelper mac;
    NetDeviceContainer staDevices, apDevices;

    // AP設定
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidAp1));
    apDevices.Add(wifi.Install(phy, mac, apNodes.Get(0)));

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssidAp2));
    apDevices.Add(wifi.Install(phy, mac, apNodes.Get(1)));
    
    // 既存ユーザー設定 (AP1に2台、AP2に1台)
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidAp1), "ActiveProbing", BooleanValue(false));
    staDevices.Add(wifi.Install(phy, mac, staNodes.Get(0)));
    staDevices.Add(wifi.Install(phy, mac, staNodes.Get(1)));
    
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssidAp2), "ActiveProbing", BooleanValue(false));
    staDevices.Add(wifi.Install(phy, mac, staNodes.Get(2)));

    // --- 2. モビリティモデル設定 ---
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);
    mobility.Install(staNodes);

    // AP位置設定
    apNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0.0, 0.0, 0.0));
    apNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(60.0, 0.0, 0.0));

    // 既存ユーザー位置設定
    staNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(5.0, 5.0, 0.0));
    staNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(-5.0, 5.0, 0.0));
    staNodes.Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(55.0, 5.0, 0.0));

    // --- 3. インターネットスタック設定 ---
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);
    
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);
    
    // --- 4. 新規ユーザー作成 ---
    NodeContainer newUserNode;
    newUserNode.Create(1);
    mobility.Install(newUserNode);
    
    // 新規ユーザーの初期位置
    Vector initialPosition(20.0, 30.0, 0.0);
    newUserNode.Get(0)->GetObject<MobilityModel>()->SetPosition(initialPosition);
    stack.Install(newUserNode);

    // --- 5. アルゴリズムパラメータ設定 ---
    double d_th = 30.0;   // 許容移動距離 (論文のパラメータ)
    double theta_th = 3.0; // 要求スループット (論文のパラメータ)

    // 各APの現在の情報 (論文のシナリオに基づく)
    std::map<uint32_t, int> usersPerAp;
    usersPerAp[0] = 2;  // AP1に2ユーザー
    usersPerAp[1] = 1;  // AP2に1ユーザー

    // 各APの接続前スループット (論文のシナリオに基づく)
    std::map<uint32_t, double> throughputBefore;
    throughputBefore[0] = 25.0;  // AP1の現在のスループット (Mbps)
    throughputBefore[1] = 17.0;  // AP2の現在のスループット (Mbps)
    
    // パラメータの妥当性チェック
    NS_LOG_INFO("=== アルゴリズムパラメータ ===");
    NS_LOG_INFO("許容移動距離 d_th: " << d_th << "m");
    NS_LOG_INFO("要求スループット θ_th: " << theta_th << "Mbps");
    for (auto const& [apId, users] : usersPerAp)
    {
        NS_LOG_INFO("AP" << apId << ": " << users << "ユーザー, " <<
                   throughputBefore[apId] << "Mbps");
    }

    // --- 6. 論文のアルゴリズム実装 ---
    std::vector<Candidate> candidates;
    
    NS_LOG_INFO("=== 新規ユーザー接続アルゴリズム開始 ===");
    NS_LOG_INFO("初期位置: " << initialPosition);
    NS_LOG_INFO("許容移動距離: " << d_th << "m");
    NS_LOG_INFO("要求スループット: " << theta_th << "Mbps");

    // 論文の「full search」アルゴリズム実装
    // 各APに対して、移動可能領域内での最適位置を探索
    for (uint32_t i = 0; i < apNodes.GetN(); ++i)
    {
        Ptr<Node> ap = apNodes.Get(i);
        Vector apPosition = ap->GetObject<MobilityModel>()->GetPosition();
        double initialDistanceToAp = CalculateEuclideanDistance(initialPosition, apPosition);
        
        NS_LOG_INFO("--- AP" << i << " 評価開始 ---");
        NS_LOG_INFO("AP位置: " << apPosition);
        NS_LOG_INFO("初期距離: " << initialDistanceToAp << "m");
        
        // 論文の式(3)に基づく最適移動位置の計算
        // d_{u,a} = |P_a - (P_u - m)| を最小化する移動ベクトルmを求める
        
        Vector bestPositionForThisAp;
        double bestMovingDistanceForThisAp = d_th + 1.0; // 初期値は制約を超える値
        bool foundValidPosition = false;
        
        // APまでの距離がd_th以下の場合、APに向かって移動
        if (initialDistanceToAp <= d_th)
        {
            // APの位置が移動可能範囲内にある場合
            bestPositionForThisAp = apPosition;
            bestMovingDistanceForThisAp = initialDistanceToAp;
            foundValidPosition = true;
        }
        else
        {
            // APが移動可能範囲外にある場合、APに向かってd_thだけ移動
            Vector direction = NormalizeVector(apPosition - initialPosition);
            bestPositionForThisAp = initialPosition + MultiplyVector(direction, d_th);  // 修正箇所
            bestMovingDistanceForThisAp = d_th;
            foundValidPosition = true;
        }
        
        if (foundValidPosition)
        {
            // 移動後のAPまでの距離を計算（論文の式(3)）
            double newDistanceToAp = CalculateEuclideanDistance(bestPositionForThisAp, apPosition);
            
            NS_LOG_INFO("最適移動距離: " << bestMovingDistanceForThisAp << "m");
            NS_LOG_INFO("最適移動先: " << bestPositionForThisAp);
            NS_LOG_INFO("AP距離(移動後): " << newDistanceToAp << "m");
            
            // 制約条件(6): 移動距離チェック |m| ≤ d_th
            if (bestMovingDistanceForThisAp <= d_th)
            {
                // 伝送レート取得（論文のTable 1）
                double b_new = GetRateFromDistance(newDistanceToAp);
                NS_LOG_INFO("伝送レート: " << b_new << "Mbps");
                
                if (b_new > 0)
                {
                    // 式(9): 接続後のAPスループット計算
                    // θ^after_{a,m} = (n_a + 1) / (n_a/θ^before_a + 1/b_{new,a})
                    double theta_after = CalculateApThroughputAfter(throughputBefore[i], usersPerAp[i], b_new);
                    
                    // 式(10): 新規ユーザーのスループット計算
                    // θ_{new,a} = θ^after_{a,m} / (n_a + 1)
                    double theta_new_user = CalculateNewUserThroughput(theta_after, usersPerAp[i]);
                    
                    NS_LOG_INFO("AP接続後スループット: " << theta_after << "Mbps");
                    NS_LOG_INFO("新規ユーザースループット: " << theta_new_user << "Mbps");
                    
                    // 制約条件(7): スループット要求チェック θ_{new,a} ≥ θ_th
                    if (theta_new_user >= theta_th)
                    {
                        // システム全体のスループット計算
                        // Θ^after_{a,m} = θ^after_{a,m} + Σ_{i≠a∈A} θ^before_i
                        double systemThroughput = 0.0;
                        for (auto const& [key, val] : throughputBefore)
                        {
                            systemThroughput += (key == i) ? theta_after : val;
                        }
                        
                        NS_LOG_INFO("システム全体スループット: " << systemThroughput << "Mbps");
                        
                        // 候補として追加
                        Candidate candidate;
                        candidate.apIndex = i;
                        candidate.newPosition = bestPositionForThisAp;
                        candidate.movingDistance = bestMovingDistanceForThisAp;
                        candidate.systemThroughput = systemThroughput;
                        candidate.newUserThroughput = theta_new_user;
                        candidate.apThroughputAfter = theta_after;
                        
                        candidates.push_back(candidate);
                        
                        NS_LOG_INFO("候補として追加されました");
                    }
                    else
                    {
                        NS_LOG_INFO("スループット要求を満たさないため除外 (" << theta_new_user << " < " << theta_th << ")");
                    }
                }
                else
                {
                    NS_LOG_INFO("伝送レートが0のため除外（距離: " << newDistanceToAp << "m）");
                }
            }
            else
            {
                NS_LOG_INFO("移動距離制約を満たさないため除外 (" << bestMovingDistanceForThisAp << " > " << d_th << ")");
            }
        }
        else
        {
            NS_LOG_INFO("有効な移動位置が見つかりませんでした");
        }
    }

    // --- 7. 最適解決定（論文の式(5)と式(6)に基づく） ---
    Candidate* bestCandidate = nullptr;
    
    if (candidates.empty())
    {
        NS_LOG_INFO("=== 接続拒否 ===");
        std::cout << "No suitable AP found. User connection is rejected." << std::endl;
        std::cout << "制約条件を満たすAP候補が見つかりませんでした。" << std::endl;
        std::cout << "- 移動距離制約: |m| ≤ " << d_th << "m" << std::endl;
        std::cout << "- スループット制約: θ_new ≥ " << theta_th << "Mbps" << std::endl;
    }
    else
    {
        NS_LOG_INFO("=== 候補リスト Φ の作成完了 ===");
        NS_LOG_INFO("候補数: " << candidates.size());
        
        // 論文の式(5): {a*, m*} = arg min_{(a,m)∈Φ} |m|
        // where Φ := arg max_{(a,m)} Θ^after_{a,m} (式(6))
        
        // Step 1: システムスループット最大値を見つける
        double maxSystemThroughput = 0.0;
        for (const auto& candidate : candidates)
        {
            if (candidate.systemThroughput > maxSystemThroughput)
            {
                maxSystemThroughput = candidate.systemThroughput;
            }
        }
        
        NS_LOG_INFO("最大システムスループット: " << maxSystemThroughput << "Mbps");
        
        // Step 2: 最大システムスループットを達成する候補のみを抽出（Φの作成）
        std::vector<Candidate*> optimalCandidates;
        const double epsilon = 1e-6; // 浮動小数点比較の許容誤差
        
        for (auto& candidate : candidates)
        {
            if (std::abs(candidate.systemThroughput - maxSystemThroughput) < epsilon)
            {
                optimalCandidates.push_back(&candidate);
            }
        }
        
        NS_LOG_INFO("最適候補数（Φ）: " << optimalCandidates.size());
        
        // Step 3: 最適候補の中から移動距離が最小のものを選択
        bestCandidate = optimalCandidates[0];
        for (auto* candidate : optimalCandidates)
        {
            NS_LOG_INFO("候補: AP" << candidate->apIndex <<
                       ", 移動距離=" << candidate->movingDistance <<
                       "m, システムスループット=" << candidate->systemThroughput << "Mbps");
            
            if (candidate->movingDistance < bestCandidate->movingDistance)
            {
                bestCandidate = candidate;
            }
        }
        
        NS_LOG_INFO("=== 最適解決定 ===");
        NS_LOG_INFO("最適AP: AP" << bestCandidate->apIndex);
        NS_LOG_INFO("移動先: " << bestCandidate->newPosition);
        NS_LOG_INFO("移動距離: " << bestCandidate->movingDistance << "m");
        NS_LOG_INFO("システムスループット: " << bestCandidate->systemThroughput << "Mbps");
        NS_LOG_INFO("新規ユーザースループット: " << bestCandidate->newUserThroughput << "Mbps");
        
        std::cout << "=== 最適化結果 ===" << std::endl;
        std::cout << "最適AP: AP" << bestCandidate->apIndex + 1 << std::endl;
        std::cout << "初期位置: " << initialPosition << std::endl;
        std::cout << "移動先: " << bestCandidate->newPosition << std::endl;
        std::cout << "移動距離: " << bestCandidate->movingDistance << "m" << std::endl;
        std::cout << "システム全体スループット: " << bestCandidate->systemThroughput << "Mbps" << std::endl;
        std::cout << "新規ユーザースループット: " << bestCandidate->newUserThroughput << "Mbps" << std::endl;

        // 新規ユーザーを最適位置に移動
        newUserNode.Get(0)->GetObject<MobilityModel>()->SetPosition(bestCandidate->newPosition);

        // 最適APに接続
        Ssid bestSsid = (bestCandidate->apIndex == 0) ? ssidAp1 : ssidAp2;
        mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(bestSsid), "ActiveProbing", BooleanValue(false));
        NetDeviceContainer newUserDevice = wifi.Install(phy, mac, newUserNode);
        
        // IPアドレス割り当て
        address.SetBase("10.1.2.0", "255.255.255.0");
        Ipv4InterfaceContainer newUserInterface = address.Assign(newUserDevice);

        // --- 8. 通信アプリケーション設定 ---
        uint16_t port = 9;
        
        // サーバー (最適APに設置)
        PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer serverApp = sink.Install(apNodes.Get(bestCandidate->apIndex));
        serverApp.Start(Seconds(1.0));
        serverApp.Stop(Seconds(10.0));

        // クライアント (新規ユーザー)
        UdpClientHelper client(apInterfaces.GetAddress(bestCandidate->apIndex), port);
        client.SetAttribute("MaxPackets", UintegerValue(1000000));
        client.SetAttribute("Interval", TimeValue(Time("0.001")));
        client.SetAttribute("PacketSize", UintegerValue(1400));

        ApplicationContainer clientApp = client.Install(newUserNode.Get(0));
        clientApp.Start(Seconds(2.0));
        clientApp.Stop(Seconds(10.0));
    }

    // --- 9. FlowMonitor設定 ---
    FlowMonitorHelper flowMonitorHelper;
    Ptr<FlowMonitor> monitor = flowMonitorHelper.InstallAll();

    // --- 10. NetAnim設定 ---
    AnimationInterface anim("miyata-animation.xml");
    
    // ノードの説明設定
    anim.UpdateNodeDescription(apNodes.Get(0), "AP1");
    anim.UpdateNodeDescription(apNodes.Get(1), "AP2");
    anim.UpdateNodeDescription(newUserNode.Get(0), "NewUser");
    
    // ノードの色設定
    anim.UpdateNodeColor(apNodes.Get(0), 255, 0, 0);  // AP1: 赤
    anim.UpdateNodeColor(apNodes.Get(1), 0, 255, 0);  // AP2: 緑
    anim.UpdateNodeColor(newUserNode.Get(0), 0, 0, 255);  // 新規ユーザー: 青

    // --- 11. シミュレーション実行 ---
    Simulator::Stop(Seconds(11.0));
    Simulator::Run();

    // --- 12. 結果表示 ---
    if (bestCandidate != nullptr)
    {
        monitor->CheckForLostPackets();
        Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowMonitorHelper.GetClassifier());
        std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
        
        std::cout << "\n=== 通信統計 ===" << std::endl;
        for (auto const& [flowId, flowStats] : stats)
        {
            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
            std::cout << "Flow " << flowId << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")" << std::endl;
            std::cout << "  送信バイト数: " << flowStats.txBytes << std::endl;
            std::cout << "  受信バイト数: " << flowStats.rxBytes << std::endl;
            
            if (flowStats.timeLastRxPacket.GetSeconds() > flowStats.timeFirstTxPacket.GetSeconds())
            {
                double throughput = flowStats.rxBytes * 8.0 / 
                    (flowStats.timeLastRxPacket.GetSeconds() - flowStats.timeFirstTxPacket.GetSeconds()) / 1024 / 1024;
                std::cout << "  実測スループット: " << throughput << " Mbps" << std::endl;
            }
        }
    }

    Simulator::Destroy();
    return 0;
}