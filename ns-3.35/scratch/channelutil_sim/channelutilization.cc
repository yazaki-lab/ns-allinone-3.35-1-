/*
 * ns-3 無線LAN チャネル使用率シミュレーション
 * Heavy/Lightユーザの混在環境でのチャネル使用率測定
 * YAML設定ファイル対応版
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
#include <yaml-cpp/yaml.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiChannelUtilizationSim");

// グローバル変数でチャネル使用率を記録
uint64_t g_totalBusyTime = 0;
uint64_t g_totalTime = 0;
Time g_startTime;
Time g_endTime;

// シミュレーションパラメータ構造体
struct SimulationConfig {
    uint32_t nStations;           // 総ユーザ数
    uint32_t nHeavyUsers;         // 重ユーザ数
    uint32_t nLightUsers;         // 軽ユーザ数
    uint32_t heavyUserPercentage; // 重ユーザ割合(%)
    double radius;                // 配置半径(m)
    std::string outputFile;       // 出力CSVファイル名
    uint32_t heavyUserRate;       // 重ユーザレート(Mbps)
    uint32_t lightUserRate;       // 軽ユーザレート(Mbps)
    uint32_t packetSize;          // パケットサイズ(バイト)
    double simulationTime;        // シミュレーション時間(秒)
    bool enableTxtOutput;         // TXT出力有効化
    bool enableNetAnim;           // NetAnim有効化
    bool verbose;                 // 詳細ログ
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

// YAML設定ファイルの読み込み
SimulationConfig LoadConfigFromYAML(const std::string& configFile) {
    SimulationConfig config;
    
    try {
        YAML::Node yamlConfig = YAML::LoadFile(configFile);
        
        // 必須パラメータの読み込み
        config.nStations = yamlConfig["nStations"].as<uint32_t>();
        config.nHeavyUsers = yamlConfig["nHeavyUsers"].as<uint32_t>();
        config.nLightUsers = yamlConfig["nLightUsers"].as<uint32_t>();
        config.heavyUserPercentage = yamlConfig["heavyUserPercentage"].as<uint32_t>();
        config.radius = yamlConfig["radius"].as<double>();
        config.outputFile = yamlConfig["outputFile"].as<std::string>();
        config.heavyUserRate = yamlConfig["heavyUserRate"].as<uint32_t>();
        config.lightUserRate = yamlConfig["lightUserRate"].as<uint32_t>();
        config.packetSize = yamlConfig["packetSize"].as<uint32_t>();
        
        // オプションパラメータ（デフォルト値あり）
        config.simulationTime = yamlConfig["simulationTime"].as<double>(10.0);
        config.enableTxtOutput = yamlConfig["enableTxtOutput"].as<bool>(true);
        config.enableNetAnim = yamlConfig["enableNetAnim"].as<bool>(true);
        config.verbose = yamlConfig["verbose"].as<bool>(false);
        
        // 整合性チェック
        if (config.nStations != config.nHeavyUsers + config.nLightUsers) {
            std::cerr << "警告: nStations != nHeavyUsers + nLightUsers" << std::endl;
            std::cerr << "nStations を " << (config.nHeavyUsers + config.nLightUsers) << " に自動調整します" << std::endl;
            config.nStations = config.nHeavyUsers + config.nLightUsers;
        }
        
        uint32_t calculatedPercentage = (config.nHeavyUsers * 100) / config.nStations;
        if (calculatedPercentage != config.heavyUserPercentage) {
            std::cerr << "警告: heavyUserPercentage の不整合を検出" << std::endl;
            std::cerr << "計算値: " << calculatedPercentage << "% → 設定値: " << config.heavyUserPercentage << "%" << std::endl;
        }
        
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML読み込みエラー: " << e.what() << std::endl;
        exit(1);
    }
    
    return config;
}

// デフォルト設定ファイルの生成
void GenerateDefaultConfig(const std::string& filename) {
    std::ofstream out(filename);
    out << "# ns-3 無線LANチャネル使用率シミュレーション 設定ファイル" << std::endl;
    out << std::endl;
    out << "# 総ユーザ数" << std::endl;
    out << "nStations: 10" << std::endl;
    out << std::endl;
    out << "# 重ユーザ数" << std::endl;
    out << "nHeavyUsers: 10" << std::endl;
    out << std::endl;
    out << "# 軽ユーザ数" << std::endl;
    out << "nLightUsers: 0" << std::endl;
    out << std::endl;
    out << "# 重ユーザ割合(%) ※nHeavyUsers/nStations * 100 と一致させること" << std::endl;
    out << "heavyUserPercentage: 100" << std::endl;
    out << std::endl;
    out << "# 実行環境の配置半径(m)" << std::endl;
    out << "radius: 7.5" << std::endl;
    out << std::endl;
    out << "# 出力CSVファイル名" << std::endl;
    out << "outputFile: \"channel_utilization_results.csv\"" << std::endl;
    out << std::endl;
    out << "# 重ユーザのデータレート(Mbps)" << std::endl;
    out << "heavyUserRate: 50" << std::endl;
    out << std::endl;
    out << "# 軽ユーザのデータレート(Mbps)" << std::endl;
    out << "lightUserRate: 20" << std::endl;
    out << std::endl;
    out << "# パケットサイズ(バイト)" << std::endl;
    out << "packetSize: 1500" << std::endl;
    out << std::endl;
    out << "# シミュレーション時間(秒)" << std::endl;
    out << "simulationTime: 10.0" << std::endl;
    out << std::endl;
    out << "# TXT形式の詳細結果出力を有効化" << std::endl;
    out << "enableTxtOutput: true" << std::endl;
    out << std::endl;
    out << "# NetAnimトレース生成を有効化" << std::endl;
    out << "enableNetAnim: true" << std::endl;
    out << std::endl;
    out << "# 詳細ログ出力" << std::endl;
    out << "verbose: false" << std::endl;
    out.close();
    
    std::cout << "デフォルト設定ファイルを生成しました: " << filename << std::endl;
}

int main(int argc, char *argv[]) {
    // デフォルト設定
    std::string configFile = "config.yaml";
    bool generateConfig = false;

    // コマンドライン引数の処理
    CommandLine cmd;
    cmd.AddValue("config", "YAML configuration file", configFile);
    cmd.AddValue("generate-config", "Generate default configuration file", generateConfig);
    cmd.Parse(argc, argv);

    // デフォルト設定ファイル生成モード
    if (generateConfig) {
        GenerateDefaultConfig(configFile);
        return 0;
    }

    // 設定ファイルの読み込み
    SimulationConfig config = LoadConfigFromYAML(configFile);

    if (config.verbose) {
        LogComponentEnable("WifiChannelUtilizationSim", LOG_LEVEL_INFO);
    }

    // 出力フォルダの生成
    std::string outputFolder = "results/" + GenerateOutputFolder();
    std::string csvFolder = "result_csv";

    NS_LOG_INFO("=== シミュレーションパラメータ ===");
    NS_LOG_INFO("設定ファイル: " << configFile);
    NS_LOG_INFO("総端末数: " << config.nStations);
    NS_LOG_INFO("Heavyユーザ数: " << config.nHeavyUsers << " (" << config.heavyUserPercentage << "%)");
    NS_LOG_INFO("Lightユーザ数: " << config.nLightUsers);
    NS_LOG_INFO("配置半径: " << config.radius << " m");
    NS_LOG_INFO("シミュレーション時間: " << config.simulationTime << " 秒");

    // ノードの作成
    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(config.nStations);

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

    // Stationを円形に配置
    for (uint32_t i = 0; i < config.nStations; ++i) {
        double angle = (2.0 * M_PI * i) / config.nStations;
        positionAlloc->Add(Vector(config.radius * cos(angle), config.radius * sin(angle), 0.0));
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

    for (uint32_t i = 0; i < config.nStations; ++i) {
        // サーバ(AP側)
        UdpServerHelper server(port + i);
        serverApps.Add(server.Install(wifiApNode.Get(0)));

        // クライアント(Station側)
        UdpClientHelper client(apInterface.GetAddress(0), port + i);
        
        // Heavy/Lightの判定
        uint32_t dataRate;
        if (i < config.nHeavyUsers) {
            dataRate = config.heavyUserRate; // Heavy user
        } else {
            dataRate = config.lightUserRate; // Light user
        }

        // データレートとパケット送信間隔の設定
        double interval = (config.packetSize * 8.0) / (dataRate * 1e6); // 秒単位
        client.SetAttribute("MaxPackets", UintegerValue(4294967295u));
        client.SetAttribute("Interval", TimeValue(Seconds(interval)));
        client.SetAttribute("PacketSize", UintegerValue(config.packetSize));

        clientApps.Add(client.Install(wifiStaNodes.Get(i)));
    }

    serverApps.Start(Seconds(0.0));
    serverApps.Stop(Seconds(config.simulationTime));
    clientApps.Start(Seconds(1.0)); // 1秒後に開始
    clientApps.Stop(Seconds(config.simulationTime));

    // PHY状態変化のトレース接続
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/State/State",
                    MakeCallback(&PhyStateChangeCallback));

    // Flow Monitorのセットアップ
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // NetAnimトレースファイルの生成
    AnimationInterface *anim = nullptr;
    std::string animFile;
    if (config.enableNetAnim) {
        // 出力フォルダの作成
        std::string mkdirCmd = "mkdir -p " + outputFolder;
        system(mkdirCmd.c_str());
        
        // アニメーションファイル名
        animFile = outputFolder + "/animation_n" + std::to_string(config.nStations) + 
                   "_h" + std::to_string(config.heavyUserPercentage) + ".xml";
        
        anim = new AnimationInterface(animFile);
        
        // パケットのトレースを有効化
        anim->EnablePacketMetadata(true);
        anim->EnableWifiMacCounters(Seconds(0), Seconds(config.simulationTime));
        anim->EnableWifiPhyCounters(Seconds(0), Seconds(config.simulationTime));
        
        // ノードの説明を追加
        anim->UpdateNodeDescription(wifiApNode.Get(0), "AP");
        for (uint32_t i = 0; i < config.nStations; ++i) {
            std::string desc = (i < config.nHeavyUsers) ? "Heavy-" : "Light-";
            desc += std::to_string(i);
            anim->UpdateNodeDescription(wifiStaNodes.Get(i), desc);
        }
        
        // ノードの色を設定(APは青、Heavyは赤、Lightは緑)
        anim->UpdateNodeColor(wifiApNode.Get(0), 0, 0, 255); // 青
        for (uint32_t i = 0; i < config.nStations; ++i) {
            if (i < config.nHeavyUsers) {
                anim->UpdateNodeColor(wifiStaNodes.Get(i), 255, 0, 0); // 赤(Heavy)
            } else {
                anim->UpdateNodeColor(wifiStaNodes.Get(i), 0, 255, 0); // 緑(Light)
            }
        }
        
        // ノードサイズの設定
        anim->UpdateNodeSize(wifiApNode.Get(0)->GetId(), 2.0, 2.0);
        for (uint32_t i = 0; i < config.nStations; ++i) {
            anim->UpdateNodeSize(wifiStaNodes.Get(i)->GetId(), 1.0, 1.0);
        }
        
        NS_LOG_INFO("NetAnimトレースファイルの保存先: " << animFile);
    }

    // シミュレーション時間の記録
    g_startTime = Seconds(1.0);
    g_endTime = Seconds(config.simulationTime);

    // シミュレーション実行
    NS_LOG_INFO("シミュレーション開始...");
    Simulator::Stop(Seconds(config.simulationTime + 0.1));
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
        double throughput = flow.second.rxBytes * 8.0 / (config.simulationTime - 1.0) / 1e6; // Mbps
        throughputs.push_back(throughput);
        totalThroughput += throughput;
        totalDelay += flow.second.delaySum.GetSeconds();
        totalRxPackets += flow.second.rxPackets;
        totalTxPackets += flow.second.txPackets;
    }

    double avgThroughput = totalThroughput / config.nStations;
    double avgDelay = (totalRxPackets > 0) ? (totalDelay / totalRxPackets) * 1000.0 : 0.0; // ms
    double packetLoss = (totalTxPackets > 0) ? 
                        (1.0 - (double)totalRxPackets / totalTxPackets) * 100.0 : 0.0;

    NS_LOG_INFO("平均スループット: " << avgThroughput << " Mbps");
    NS_LOG_INFO("平均遅延: " << avgDelay << " ms");
    NS_LOG_INFO("パケット損失率: " << packetLoss << "%");

    // タイムスタンプ付きファイル名の生成（シミュレーション時間を含む）
    std::string timestamp = GenerateTimestamp(config.simulationTime);

    // テキストファイルへの出力
    if (config.enableTxtOutput) {
        std::string txtFile = outputFolder + "/results_n" + std::to_string(config.nStations) + 
                              "_h" + std::to_string(config.heavyUserPercentage) + ".txt";

        std::ofstream txtOut(txtFile);
        txtOut << "========================================" << std::endl;
        txtOut << "ns-3 無線LANチャネル使用率シミュレーション結果" << std::endl;
        txtOut << "========================================" << std::endl;
        txtOut << std::endl;
        
        txtOut << "[シミュレーションパラメータ]" << std::endl;
        txtOut << "総端末数: " << config.nStations << std::endl;
        txtOut << "Heavyユーザ数: " << config.nHeavyUsers << " (" << config.heavyUserPercentage << "%)" << std::endl;
        txtOut << "Lightユーザ数: " << config.nLightUsers << " (" << (100 - config.heavyUserPercentage) << "%)" << std::endl;
        txtOut << "配置半径: " << config.radius << " m" << std::endl;
        txtOut << "Heavyユーザデータレート: " << config.heavyUserRate << " Mbps" << std::endl;
        txtOut << "Lightユーザデータレート: " << config.lightUserRate << " Mbps" << std::endl;
        txtOut << "パケットサイズ: " << config.packetSize << " バイト" << std::endl;
        txtOut << "シミュレーション時間: " << config.simulationTime << " 秒" << std::endl;
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
            double throughput = flow.second.rxBytes * 8.0 / (config.simulationTime - 1.0) / 1e6;
            std::string userType = (flowIdx < config.nHeavyUsers) ? "Heavy" : "Light";
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
    }

    // 結果をCSVファイルに出力(累積用)
    // CSVフォルダの作成
    std::string mkdirCsvCmd = "mkdir -p " + csvFolder;
    system(mkdirCsvCmd.c_str());
    
    std::string csvFile = csvFolder + "/" + config.outputFile;
    std::ofstream outFile;
    bool fileExists = std::ifstream(csvFile).good();
    outFile.open(csvFile, std::ios::app);

    if (!fileExists) {
        // ヘッダー行
        outFile << "クライアント数,重ユーザ数,軽ユーザ数,重ユーザ割合,配置半径,シミュレーション時間,チャネル使用率,";
        outFile << "平均スループット,平均遅延,パケット損失率,タイムスタンプ" << std::endl;
    }

    outFile << config.nStations << "," << config.nHeavyUsers << "," << config.nLightUsers << "," 
            << config.heavyUserPercentage << "," << config.radius << "," << config.simulationTime << "," << channelUtil << ","
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