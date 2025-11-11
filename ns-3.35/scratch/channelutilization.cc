/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * ns-3 無線LAN チャネル使用率シミュレーション
 * Heavy/Lightユーザの混在環境でのチャネル使用率測定
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"

#include <fstream>
#include <vector>
#include <ctime>
#include <sstream>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiChannelUtilizationSim");

// グローバル変数でチャネル使用率を記録
uint64_t g_totalBusyTime = 0;
uint64_t g_totalTime = 0;
Time g_startTime;
Time g_endTime;

// 統計情報の構造体
struct SimulationStats {
    uint32_t numHeavyUsers;
    uint32_t numLightUsers;
    double channelUtilization;
    std::vector<double> throughputs;
    double avgDelay;
    double packetLoss;
};

// PHY状態変化のコールバック
void PhyStateChangeCallback(std::string context, Time start, Time duration, WifiPhyState state) {
    if (state == WifiPhyState::TX || state == WifiPhyState::RX || state == WifiPhyState::CCA_BUSY) {
        g_totalBusyTime += duration.GetNanoSeconds();
    }
    g_totalTime += duration.GetNanoSeconds();
}

// チャネル使用率を計算
double CalculateChannelUtilization() {
    if (g_totalTime == 0) {
        return 0.0;
    }
    return (double)g_totalBusyTime / (double)g_totalTime * 100.0;
}

// 出力フォルダ名生成関数（日付_時間(JST)）
std::string GenerateOutputFolder() {
    time_t now = time(0);
    struct tm tstruct;
    char timestamp[100];
    tstruct = *localtime(&now);
    
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tstruct);
    
    std::ostringstream oss;
    oss << "channelutilization_" << timestamp;
    
    return oss.str();
}

// タイムスタンプ生成関数（シミュレーション時間を含む）
std::string GenerateTimestamp(double simTime) {
    time_t now = time(0);
    struct tm tstruct;
    char timestamp[100];
    tstruct = *localtime(&now);
    
    // 日時部分
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tstruct);
    
    // シミュレーション時間を追加
    std::ostringstream oss;
    oss << timestamp << "_t" << std::fixed << std::setprecision(1) << simTime << "s";
    
    return oss.str();
}

int main(int argc, char *argv[]) {
    // パラメータ設定
    uint32_t nStations = 10;           // クライアント数
    uint32_t heavyUserPercentage = 100; // Heavyユーザの割合 (0-100%)
    double simulationTime = 10.0;      // シミュレーション時間(秒)
    uint32_t heavyUserRate = 50;      // Heavy: 50 Mbps
    uint32_t lightUserRate = 20;       // Light: 20 Mbps (実際は≤30)
    uint32_t packetSize = 1500;       // パケットサイズ(バイト)
    std::string outputFile = "channel_utilization_results_H50L20_rad75.csv";
    bool verbose = false;
    bool enableNetAnim = true;

    // 出力フォルダの生成
    std::string outputFolder = "results/" + GenerateOutputFolder();
    std::string csvFolder = "result_csv";

    // コマンドライン引数の処理
    CommandLine cmd;
    cmd.AddValue("nStations", "Number of stations", nStations);
    cmd.AddValue("heavyPercent", "Percentage of heavy users (0-100)", heavyUserPercentage);
    cmd.AddValue("simTime", "Simulation time in seconds", simulationTime);
    cmd.AddValue("heavyRate", "Heavy user data rate in Mbps", heavyUserRate);
    cmd.AddValue("lightRate", "Light user data rate in Mbps", lightUserRate);
    cmd.AddValue("packetSize", "Packet size in bytes", packetSize);
    cmd.AddValue("output", "Output CSV file name", outputFile);
    cmd.AddValue("verbose", "Enable verbose logging", verbose);
    cmd.AddValue("netanim", "Enable NetAnim trace generation", enableNetAnim);
    cmd.Parse(argc, argv);

    if (verbose) {
        LogComponentEnable("WifiChannelUtilizationSim", LOG_LEVEL_INFO);
    }

    // Heavy/Lightユーザ数の計算
    uint32_t nHeavy = (nStations * heavyUserPercentage) / 100;
    uint32_t nLight = nStations - nHeavy;

    NS_LOG_INFO("=== シミュレーションパラメータ ===");
    NS_LOG_INFO("総端末数: " << nStations);
    NS_LOG_INFO("Heavyユーザ数: " << nHeavy << " (" << heavyUserPercentage << "%)");
    NS_LOG_INFO("Lightユーザ数: " << nLight);
    NS_LOG_INFO("シミュレーション時間: " << simulationTime << " 秒");

    // ノードの作成
    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nStations);

    // Wi-Fi チャネルとPHY層の設定
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    // Wi-Fi MAC層の設定(802.11ac)
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ac);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                   "DataMode", StringValue("VhtMcs8"),
                                   "ControlMode", StringValue("VhtMcs0"));

    WifiMacHelper mac;
    Ssid ssid = Ssid("ns3-wifi-sim");

    // AP の設定
    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevice = wifi.Install(phy, mac, wifiApNode);

    // Station の設定
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer staDevices = wifi.Install(phy, mac, wifiStaNodes);

    // モビリティモデル(固定位置)
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    
    // APを中心に配置
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));

    // Stationを円形に配置(半径7.5m)
    double radius = 7.5;
    for (uint32_t i = 0; i < nStations; ++i) {
        double angle = (2.0 * M_PI * i) / nStations;
        positionAlloc->Add(Vector(radius * cos(angle), radius * sin(angle), 0.0));
    }

    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(wifiApNode);
    mobility.Install(wifiStaNodes);

    // インターネットスタックのインストール
    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNodes);

    // IPアドレスの割り当て
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface = address.Assign(apDevice);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

    // トラフィック生成(UDP)
    uint16_t port = 9;
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;

    for (uint32_t i = 0; i < nStations; ++i) {
        // サーバ(AP側)
        UdpServerHelper server(port + i);
        serverApps.Add(server.Install(wifiApNode.Get(0)));

        // クライアント(Station側)
        UdpClientHelper client(apInterface.GetAddress(0), port + i);
        
        // Heavy/Lightの判定
        uint32_t dataRate;
        if (i < nHeavy) {
            dataRate = heavyUserRate; // Heavy user
        } else {
            dataRate = lightUserRate; // Light user
        }

        // データレートとパケット送信間隔の設定
        double interval = (packetSize * 8.0) / (dataRate * 1e6); // 秒単位
        client.SetAttribute("MaxPackets", UintegerValue(4294967295u));
        client.SetAttribute("Interval", TimeValue(Seconds(interval)));
        client.SetAttribute("PacketSize", UintegerValue(packetSize));

        clientApps.Add(client.Install(wifiStaNodes.Get(i)));
    }

    serverApps.Start(Seconds(0.0));
    serverApps.Stop(Seconds(simulationTime));
    clientApps.Start(Seconds(1.0)); // 1秒後に開始
    clientApps.Stop(Seconds(simulationTime));

    // PHY状態変化のトレース接続
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/State/State",
                    MakeCallback(&PhyStateChangeCallback));

    // Flow Monitorのセットアップ
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // NetAnimトレースファイルの生成
    AnimationInterface *anim = nullptr;
    std::string animFile;
    if (enableNetAnim) {
        // 出力フォルダの作成
        std::string mkdirCmd = "mkdir -p " + outputFolder;
        system(mkdirCmd.c_str());
        
        // アニメーションファイル名
        animFile = outputFolder + "/animation_n" + std::to_string(nStations) + 
                   "_h" + std::to_string(heavyUserPercentage) + ".xml";
        
        anim = new AnimationInterface(animFile);
        
        // パケットのトレースを有効化
        anim->EnablePacketMetadata(true);
        anim->EnableWifiMacCounters(Seconds(0), Seconds(simulationTime));
        anim->EnableWifiPhyCounters(Seconds(0), Seconds(simulationTime));
        
        // ノードの説明を追加
        anim->UpdateNodeDescription(wifiApNode.Get(0), "AP");
        for (uint32_t i = 0; i < nStations; ++i) {
            std::string desc = (i < nHeavy) ? "Heavy-" : "Light-";
            desc += std::to_string(i);
            anim->UpdateNodeDescription(wifiStaNodes.Get(i), desc);
        }
        
        // ノードの色を設定(APは青、Heavyは赤、Lightは緑)
        anim->UpdateNodeColor(wifiApNode.Get(0), 0, 0, 255); // 青
        for (uint32_t i = 0; i < nStations; ++i) {
            if (i < nHeavy) {
                anim->UpdateNodeColor(wifiStaNodes.Get(i), 255, 0, 0); // 赤(Heavy)
            } else {
                anim->UpdateNodeColor(wifiStaNodes.Get(i), 0, 255, 0); // 緑(Light)
            }
        }
        
        // ノードサイズの設定
        anim->UpdateNodeSize(wifiApNode.Get(0)->GetId(), 2.0, 2.0);
        for (uint32_t i = 0; i < nStations; ++i) {
            anim->UpdateNodeSize(wifiStaNodes.Get(i)->GetId(), 1.0, 1.0);
        }
        
        NS_LOG_INFO("NetAnimトレースファイルの保存先: " << animFile);
    }

    // シミュレーション時間の記録
    g_startTime = Seconds(1.0);
    g_endTime = Seconds(simulationTime);

    // シミュレーション実行
    NS_LOG_INFO("シミュレーション開始...");
    Simulator::Stop(Seconds(simulationTime + 0.1));
    Simulator::Run();

    // 統計情報の収集
    double channelUtil = CalculateChannelUtilization();
    
    NS_LOG_INFO("=== シミュレーション結果 ===");
    NS_LOG_INFO("チャネル使用率: " << channelUtil << "%");

    // Flow Monitorの統計
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    double totalThroughput = 0.0;
    double totalDelay = 0.0;
    uint64_t totalRxPackets = 0;
    uint64_t totalTxPackets = 0;
    std::vector<double> throughputs;

    for (auto const &flow : stats) {
        double throughput = flow.second.rxBytes * 8.0 / (simulationTime - 1.0) / 1e6; // Mbps
        throughputs.push_back(throughput);
        totalThroughput += throughput;
        totalDelay += flow.second.delaySum.GetSeconds();
        totalRxPackets += flow.second.rxPackets;
        totalTxPackets += flow.second.txPackets;
    }

    double avgThroughput = totalThroughput / nStations;
    double avgDelay = (totalRxPackets > 0) ? (totalDelay / totalRxPackets) * 1000.0 : 0.0; // ms
    double packetLoss = (totalTxPackets > 0) ? 
                        (1.0 - (double)totalRxPackets / totalTxPackets) * 100.0 : 0.0;

    NS_LOG_INFO("平均スループット: " << avgThroughput << " Mbps");
    NS_LOG_INFO("平均遅延: " << avgDelay << " ms");
    NS_LOG_INFO("パケット損失率: " << packetLoss << "%");

    // タイムスタンプ付きファイル名の生成（シミュレーション時間を含む）
    std::string timestamp = GenerateTimestamp(simulationTime);
    
    std::string txtFile = outputFolder + "/results_n" + std::to_string(nStations) + 
                          "_h" + std::to_string(heavyUserPercentage) + ".txt";

    // テキストファイルへの出力
    std::ofstream txtOut(txtFile);
    txtOut << "========================================" << std::endl;
    txtOut << "ns-3 無線LANチャネル使用率シミュレーション結果" << std::endl;
    txtOut << "========================================" << std::endl;
    txtOut << std::endl;
    
    txtOut << "[シミュレーションパラメータ]" << std::endl;
    txtOut << "総端末数: " << nStations << std::endl;
    txtOut << "Heavyユーザ数: " << nHeavy << " (" << heavyUserPercentage << "%)" << std::endl;
    txtOut << "Lightユーザ数: " << nLight << " (" << (100 - heavyUserPercentage) << "%)" << std::endl;
    txtOut << "Heavyユーザデータレート: " << heavyUserRate << " Mbps" << std::endl;
    txtOut << "Lightユーザデータレート: " << lightUserRate << " Mbps" << std::endl;
    txtOut << "パケットサイズ: " << packetSize << " バイト" << std::endl;
    txtOut << "シミュレーション時間: " << simulationTime << " 秒" << std::endl;
    txtOut << "実行日時: " << timestamp << std::endl;
    txtOut << std::endl;
    
    txtOut << "[チャネル使用率]" << std::endl;
    txtOut << "チャネル使用率: " << channelUtil << " %" << std::endl;
    txtOut << "総ビジー時間: " << g_totalBusyTime / 1e9 << " 秒" << std::endl;
    txtOut << "総測定時間: " << g_totalTime / 1e9 << " 秒" << std::endl;
    txtOut << std::endl;
    
    txtOut << "[性能指標]" << std::endl;
    txtOut << "平均スループット: " << avgThroughput << " Mbps" << std::endl;
    txtOut << "総スループット: " << totalThroughput << " Mbps" << std::endl;
    txtOut << "平均遅延: " << avgDelay << " ms" << std::endl;
    txtOut << "パケット損失率: " << packetLoss << " %" << std::endl;
    txtOut << "総送信パケット数: " << totalTxPackets << std::endl;
    txtOut << "総受信パケット数: " << totalRxPackets << std::endl;
    txtOut << std::endl;
    
    txtOut << "[フロー別スループット]" << std::endl;
    uint32_t flowIdx = 0;
    for (auto const &flow : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        double throughput = flow.second.rxBytes * 8.0 / (simulationTime - 1.0) / 1e6;
        std::string userType = (flowIdx < nHeavy) ? "Heavy" : "Light";
        txtOut << "フロー " << flowIdx << " (" << userType << "): " 
               << throughput << " Mbps" << std::endl;
        flowIdx++;
    }
    txtOut << std::endl;
    
    txtOut << "[フロー詳細統計]" << std::endl;
    flowIdx = 0;
    for (auto const &flow : stats) {
        txtOut << "フロー " << flowIdx << ":" << std::endl;
        txtOut << "  送信パケット数: " << flow.second.txPackets << std::endl;
        txtOut << "  受信パケット数: " << flow.second.rxPackets << std::endl;
        txtOut << "  送信バイト数: " << flow.second.txBytes << std::endl;
        txtOut << "  受信バイト数: " << flow.second.rxBytes << std::endl;
        txtOut << "  損失パケット数: " << flow.second.lostPackets << std::endl;
        if (flow.second.rxPackets > 0) {
            txtOut << "  平均遅延: " << (flow.second.delaySum.GetSeconds() / flow.second.rxPackets) * 1000.0 << " ms" << std::endl;
        }
        txtOut << std::endl;
        flowIdx++;
    }
    txtOut.close();
    
    NS_LOG_INFO("テキスト形式の結果を出力しました: " << txtFile);

    // 結果をCSVファイルに出力(累積用)
    // CSVフォルダの作成
    std::string mkdirCsvCmd = "mkdir -p " + csvFolder;
    system(mkdirCsvCmd.c_str());
    
    std::string csvFile = csvFolder + "/" + outputFile;
    std::ofstream outFile;
    bool fileExists = std::ifstream(csvFile).good();
    outFile.open(csvFile, std::ios::app);

    if (!fileExists) {
        // ヘッダー行
        outFile << "クライアント数,重ユーザ数,軽ユーザ数,重ユーザ割合,シミュレーション時間,チャネル使用率,";
        outFile << "平均スループット,平均遅延,パケット損失率,タイムスタンプ" << std::endl;
    }

    outFile << nStations << "," << nHeavy << "," << nLight << "," 
            << heavyUserPercentage << "," << simulationTime << "," << channelUtil << ","
            << avgThroughput << "," << avgDelay << "," << packetLoss << ","
            << timestamp << std::endl;
    outFile.close();

    NS_LOG_INFO("CSV形式の結果を出力しました: " << csvFile);

    // NetAnimリソースの解放
    if (anim) {
        delete anim;
        NS_LOG_INFO("NetAnimトレースを出力しました: " << animFile);
    }

    Simulator::Destroy();
    return 0;
}