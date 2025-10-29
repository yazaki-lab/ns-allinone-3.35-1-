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

int main(int argc, char *argv[]) {
    // パラメータ設定
    uint32_t nStations = 10;           // クライアント数
    uint32_t heavyUserPercentage = 50; // Heavyユーザの割合 (0-100%)
    double simulationTime = 10.0;       // シミュレーション時間（秒）
    uint32_t heavyUserRate = 100;      // Heavy: 100 Mbps
    uint32_t lightUserRate = 50;       // Light: 50 Mbps (実際は≤50)
    uint32_t packetSize = 1024;        // パケットサイズ（バイト）
    std::string outputFile = "channel_utilization_results.csv";
    std::string resultsDir = "results";
    bool verbose = false;
    bool enableNetAnim = true;

    // コマンドライン引数の処理
    CommandLine cmd;
    cmd.AddValue("nStations", "Number of stations", nStations);
    cmd.AddValue("heavyPercent", "Percentage of heavy users (0-100)", heavyUserPercentage);
    cmd.AddValue("simTime", "Simulation time in seconds", simulationTime);
    cmd.AddValue("heavyRate", "Heavy user data rate in Mbps", heavyUserRate);
    cmd.AddValue("lightRate", "Light user data rate in Mbps", lightUserRate);
    cmd.AddValue("packetSize", "Packet size in bytes", packetSize);
    cmd.AddValue("output", "Output CSV file name", outputFile);
    cmd.AddValue("resultsDir", "Results directory path", resultsDir);
    cmd.AddValue("verbose", "Enable verbose logging", verbose);
    cmd.AddValue("netanim", "Enable NetAnim trace generation", enableNetAnim);
    cmd.Parse(argc, argv);

    if (verbose) {
        LogComponentEnable("WifiChannelUtilizationSim", LOG_LEVEL_INFO);
    }

    // Heavy/Lightユーザ数の計算
    uint32_t nHeavy = (nStations * heavyUserPercentage) / 100;
    uint32_t nLight = nStations - nHeavy;

    NS_LOG_INFO("=== Simulation Parameters ===");
    NS_LOG_INFO("Total Stations: " << nStations);
    NS_LOG_INFO("Heavy Users: " << nHeavy << " (" << heavyUserPercentage << "%)");
    NS_LOG_INFO("Light Users: " << nLight);
    NS_LOG_INFO("Simulation Time: " << simulationTime << " seconds");

    // ノードの作成
    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nStations);

    // Wi-Fi チャネルとPHY層の設定
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    // Wi-Fi MAC層の設定（802.11ac）
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

    // モビリティモデル（固定位置）
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    
    // APを中心に配置
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    
    // Stationを円形に配置（半径5m）
    double radius = 5.0;
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

    // トラフィック生成（UDP）
    uint16_t port = 9;
    ApplicationContainer serverApps;
    ApplicationContainer clientApps;

    for (uint32_t i = 0; i < nStations; ++i) {
        // サーバ（AP側）
        UdpServerHelper server(port + i);
        serverApps.Add(server.Install(wifiApNode.Get(0)));

        // クライアント（Station側）
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
        // 結果ディレクトリの作成
        std::string mkdirCmd = "mkdir -p " + resultsDir;
        system(mkdirCmd.c_str());
        
        // タイムスタンプ付きファイル名
        time_t now = time(0);
        struct tm tstruct;
        char timestamp[80];
        tstruct = *localtime(&now);
        strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tstruct);
        
        animFile = resultsDir + "/animation_n" + std::to_string(nStations) + 
                   "_h" + std::to_string(heavyUserPercentage) + 
                   "_" + std::string(timestamp) + ".xml";
        
        anim = new AnimationInterface(animFile);
        
        // ノードの説明を追加
        anim->UpdateNodeDescription(wifiApNode.Get(0), "AP");
        for (uint32_t i = 0; i < nStations; ++i) {
            std::string desc = (i < nHeavy) ? "Heavy-" : "Light-";
            desc += std::to_string(i);
            anim->UpdateNodeDescription(wifiStaNodes.Get(i), desc);
        }
        
        // ノードの色を設定（APは青、Heavyは赤、Lightは緑）
        anim->UpdateNodeColor(wifiApNode.Get(0), 0, 0, 255); // 青
        for (uint32_t i = 0; i < nStations; ++i) {
            if (i < nHeavy) {
                anim->UpdateNodeColor(wifiStaNodes.Get(i), 255, 0, 0); // 赤（Heavy）
            } else {
                anim->UpdateNodeColor(wifiStaNodes.Get(i), 0, 255, 0); // 緑（Light）
            }
        }
        
        // ノードサイズの設定
        anim->UpdateNodeSize(wifiApNode.Get(0)->GetId(), 2.0, 2.0);
        for (uint32_t i = 0; i < nStations; ++i) {
            anim->UpdateNodeSize(wifiStaNodes.Get(i)->GetId(), 1.0, 1.0);
        }
        
        NS_LOG_INFO("NetAnim trace file will be saved to: " << animFile);
    }

    // シミュレーション時間の記録
    g_startTime = Seconds(1.0);
    g_endTime = Seconds(simulationTime);

    // シミュレーション実行
    NS_LOG_INFO("Starting simulation...");
    Simulator::Stop(Seconds(simulationTime + 0.1));
    Simulator::Run();

    // 統計情報の収集
    double channelUtil = CalculateChannelUtilization();
    
    NS_LOG_INFO("=== Simulation Results ===");
    NS_LOG_INFO("Channel Utilization: " << channelUtil << "%");

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

    NS_LOG_INFO("Average Throughput: " << avgThroughput << " Mbps");
    NS_LOG_INFO("Average Delay: " << avgDelay << " ms");
    NS_LOG_INFO("Packet Loss Rate: " << packetLoss << "%");

    // 結果ディレクトリの作成
    std::string mkdirCmd = "mkdir -p " + resultsDir;
    system(mkdirCmd.c_str());

    // タイムスタンプ付きファイル名の生成
    time_t now = time(0);
    struct tm tstruct;
    char timestamp[80];
    tstruct = *localtime(&now);
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tstruct);
    
    std::string baseName = resultsDir + "/sim_n" + std::to_string(nStations) + 
                          "_h" + std::to_string(heavyUserPercentage) + 
                          "_" + std::string(timestamp);
    
    std::string txtFile = baseName + ".txt";
    std::string xmlFile = baseName + ".xml";

    // テキストファイルへの出力
    std::ofstream txtOut(txtFile);
    txtOut << "========================================" << std::endl;
    txtOut << "ns-3 Wi-Fi Channel Utilization Simulation Results" << std::endl;
    txtOut << "========================================" << std::endl;
    txtOut << std::endl;
    
    txtOut << "[Simulation Parameters]" << std::endl;
    txtOut << "Total Stations: " << nStations << std::endl;
    txtOut << "Heavy Users: " << nHeavy << " (" << heavyUserPercentage << "%)" << std::endl;
    txtOut << "Light Users: " << nLight << " (" << (100 - heavyUserPercentage) << "%)" << std::endl;
    txtOut << "Heavy User Rate: " << heavyUserRate << " Mbps" << std::endl;
    txtOut << "Light User Rate: " << lightUserRate << " Mbps" << std::endl;
    txtOut << "Packet Size: " << packetSize << " bytes" << std::endl;
    txtOut << "Simulation Time: " << simulationTime << " seconds" << std::endl;
    txtOut << std::endl;
    
    txtOut << "[Channel Utilization]" << std::endl;
    txtOut << "Channel Utilization: " << channelUtil << " %" << std::endl;
    txtOut << "Total Busy Time: " << g_totalBusyTime / 1e9 << " seconds" << std::endl;
    txtOut << "Total Time: " << g_totalTime / 1e9 << " seconds" << std::endl;
    txtOut << std::endl;
    
    txtOut << "[Performance Metrics]" << std::endl;
    txtOut << "Average Throughput: " << avgThroughput << " Mbps" << std::endl;
    txtOut << "Total Throughput: " << totalThroughput << " Mbps" << std::endl;
    txtOut << "Average Delay: " << avgDelay << " ms" << std::endl;
    txtOut << "Packet Loss Rate: " << packetLoss << " %" << std::endl;
    txtOut << "Total Tx Packets: " << totalTxPackets << std::endl;
    txtOut << "Total Rx Packets: " << totalRxPackets << std::endl;
    txtOut << std::endl;
    
    txtOut << "[Per-Flow Throughput]" << std::endl;
    uint32_t flowIdx = 0;
    for (auto const &flow : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        double throughput = flow.second.rxBytes * 8.0 / (simulationTime - 1.0) / 1e6;
        std::string userType = (flowIdx < nHeavy) ? "Heavy" : "Light";
        txtOut << "Flow " << flowIdx << " (" << userType << "): " 
               << throughput << " Mbps" << std::endl;
        flowIdx++;
    }
    txtOut << std::endl;
    
    txtOut << "[Flow Monitor Statistics]" << std::endl;
    flowIdx = 0;
    for (auto const &flow : stats) {
        txtOut << "Flow " << flowIdx << ":" << std::endl;
        txtOut << "  Tx Packets: " << flow.second.txPackets << std::endl;
        txtOut << "  Rx Packets: " << flow.second.rxPackets << std::endl;
        txtOut << "  Tx Bytes: " << flow.second.txBytes << std::endl;
        txtOut << "  Rx Bytes: " << flow.second.rxBytes << std::endl;
        txtOut << "  Lost Packets: " << flow.second.lostPackets << std::endl;
        if (flow.second.rxPackets > 0) {
            txtOut << "  Average Delay: " << (flow.second.delaySum.GetSeconds() / flow.second.rxPackets) * 1000.0 << " ms" << std::endl;
        }
        txtOut << std::endl;
        flowIdx++;
    }
    txtOut.close();
    
    NS_LOG_INFO("Text results written to " << txtFile);

    // XMLファイルへの出力
    std::ofstream xmlOut(xmlFile);
    xmlOut << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
    xmlOut << "<simulation>" << std::endl;
    xmlOut << "  <parameters>" << std::endl;
    xmlOut << "    <totalStations>" << nStations << "</totalStations>" << std::endl;
    xmlOut << "    <heavyUsers>" << nHeavy << "</heavyUsers>" << std::endl;
    xmlOut << "    <lightUsers>" << nLight << "</lightUsers>" << std::endl;
    xmlOut << "    <heavyUserPercentage>" << heavyUserPercentage << "</heavyUserPercentage>" << std::endl;
    xmlOut << "    <heavyUserRate unit=\"Mbps\">" << heavyUserRate << "</heavyUserRate>" << std::endl;
    xmlOut << "    <lightUserRate unit=\"Mbps\">" << lightUserRate << "</lightUserRate>" << std::endl;
    xmlOut << "    <packetSize unit=\"bytes\">" << packetSize << "</packetSize>" << std::endl;
    xmlOut << "    <simulationTime unit=\"seconds\">" << simulationTime << "</simulationTime>" << std::endl;
    xmlOut << "    <timestamp>" << timestamp << "</timestamp>" << std::endl;
    xmlOut << "  </parameters>" << std::endl;
    xmlOut << std::endl;
    
    xmlOut << "  <channelUtilization>" << std::endl;
    xmlOut << "    <utilization unit=\"percent\">" << channelUtil << "</utilization>" << std::endl;
    xmlOut << "    <totalBusyTime unit=\"seconds\">" << g_totalBusyTime / 1e9 << "</totalBusyTime>" << std::endl;
    xmlOut << "    <totalTime unit=\"seconds\">" << g_totalTime / 1e9 << "</totalTime>" << std::endl;
    xmlOut << "  </channelUtilization>" << std::endl;
    xmlOut << std::endl;
    
    xmlOut << "  <performanceMetrics>" << std::endl;
    xmlOut << "    <averageThroughput unit=\"Mbps\">" << avgThroughput << "</averageThroughput>" << std::endl;
    xmlOut << "    <totalThroughput unit=\"Mbps\">" << totalThroughput << "</totalThroughput>" << std::endl;
    xmlOut << "    <averageDelay unit=\"ms\">" << avgDelay << "</averageDelay>" << std::endl;
    xmlOut << "    <packetLossRate unit=\"percent\">" << packetLoss << "</packetLossRate>" << std::endl;
    xmlOut << "    <totalTxPackets>" << totalTxPackets << "</totalTxPackets>" << std::endl;
    xmlOut << "    <totalRxPackets>" << totalRxPackets << "</totalRxPackets>" << std::endl;
    xmlOut << "  </performanceMetrics>" << std::endl;
    xmlOut << std::endl;
    
    xmlOut << "  <flows>" << std::endl;
    flowIdx = 0;
    for (auto const &flow : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        double throughput = flow.second.rxBytes * 8.0 / (simulationTime - 1.0) / 1e6;
        std::string userType = (flowIdx < nHeavy) ? "Heavy" : "Light";
        double flowDelay = (flow.second.rxPackets > 0) ? 
                          (flow.second.delaySum.GetSeconds() / flow.second.rxPackets) * 1000.0 : 0.0;
        
        xmlOut << "    <flow id=\"" << flowIdx << "\" type=\"" << userType << "\">" << std::endl;
        xmlOut << "      <sourceIP>" << t.sourceAddress << "</sourceIP>" << std::endl;
        xmlOut << "      <destinationIP>" << t.destinationAddress << "</destinationIP>" << std::endl;
        xmlOut << "      <sourcePort>" << t.sourcePort << "</sourcePort>" << std::endl;
        xmlOut << "      <destinationPort>" << t.destinationPort << "</destinationPort>" << std::endl;
        xmlOut << "      <throughput unit=\"Mbps\">" << throughput << "</throughput>" << std::endl;
        xmlOut << "      <txPackets>" << flow.second.txPackets << "</txPackets>" << std::endl;
        xmlOut << "      <rxPackets>" << flow.second.rxPackets << "</rxPackets>" << std::endl;
        xmlOut << "      <txBytes>" << flow.second.txBytes << "</txBytes>" << std::endl;
        xmlOut << "      <rxBytes>" << flow.second.rxBytes << "</rxBytes>" << std::endl;
        xmlOut << "      <lostPackets>" << flow.second.lostPackets << "</lostPackets>" << std::endl;
        xmlOut << "      <averageDelay unit=\"ms\">" << flowDelay << "</averageDelay>" << std::endl;
        xmlOut << "    </flow>" << std::endl;
        flowIdx++;
    }
    xmlOut << "  </flows>" << std::endl;
    xmlOut << "</simulation>" << std::endl;
    xmlOut.close();
    
    NS_LOG_INFO("XML results written to " << xmlFile);

    // 結果をCSVファイルに出力（累積用）
    std::string csvFile = resultsDir + "/" + outputFile;
    std::ofstream outFile;
    bool fileExists = std::ifstream(csvFile).good();
    outFile.open(csvFile, std::ios::app);

    if (!fileExists) {
        // ヘッダー行
        outFile << "nStations,nHeavy,nLight,heavyPercent,channelUtilization,";
        outFile << "avgThroughput,avgDelay,packetLoss" << std::endl;
    }

    outFile << nStations << "," << nHeavy << "," << nLight << "," 
            << heavyUserPercentage << "," << channelUtil << ","
            << avgThroughput << "," << avgDelay << "," << packetLoss << std::endl;
    outFile.close();

    NS_LOG_INFO("CSV results written to " << csvFile);

    // NetAnimリソースの解放
    if (anim) {
        delete anim;
        NS_LOG_INFO("NetAnim trace written to " << animFile);
    }

    Simulator::Destroy();
    return 0;
}