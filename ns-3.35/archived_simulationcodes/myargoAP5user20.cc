#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

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
    bool isNew; // 新規ユーザーかどうか
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
    bool isNewUser; // 新規ユーザーかどうか
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
        // デフォルト重み設定
        m_weights = {0.4, 0.3, 0.2, 0.1}; // 各指標の重み  
    
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
        // 802.11nの仕様に基づく伝送レート
        // 標準設定
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
        
        // 標準設定（調和平均）
        double sum = 0.0;
        for (double rate : ap.userRates) {
            sum += 1.0 / rate;
        }
        sum += 1.0 / newRate;
        
        return (ap.userRates.size() + 1) / sum;
    }

    // 最適AP選択アルゴリズム（詳細な結果を返す）
    APSelectionResult SelectOptimalAPDetailed(const Vector& userPos, uint32_t userId, bool isNewUser = true) {
        APSelectionResult result;
        result.userId = userId;
        result.userPosition = userPos;
        result.selectedAP = 0;
        result.expectedThroughput = 0.0;
        result.distance = 0.0;
        result.score = 0.0;
        result.isNewUser = isNewUser;

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

        // スコア計算（式6-10）  
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
        APSelectionResult result = SelectOptimalAPDetailed(userPos, 0, true);
        return std::make_pair(result.selectedAP, Vector(0, 0, 0));
    }
};

// グローバル変数
static std::ofstream* g_resultFile = nullptr;
static std::ofstream* g_configFile = nullptr;
static NodeContainer* g_apNodes = nullptr;
static NodeContainer* g_staNodes = nullptr;
static uint32_t g_nAPs = 0;
static uint32_t g_nUsers = 0;
static uint32_t g_nExistingUsers = 0; // 既存接続済みユーザー数
static uint32_t g_nNewUsers = 0; // 新規ユーザー数
static double g_simTime = 0.0;
static APSelectionAlgorithm* g_algorithm = nullptr;
static std::vector<APInfo> g_apInfoList;
static std::vector<UserInfo> g_userInfoList; // ユーザー情報リスト
static std::string g_outputDir = "";
static std::string g_timestamp = "";

// タイムスタンプ文字列を生成する関数
std::string GetTimestamp() {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
    return std::string(buffer);
}

// ディレクトリを作成する関数
bool CreateDirectory(const std::string& path) {
#ifdef _WIN32
    return _mkdir(path.c_str()) == 0 || errno == EEXIST;
#else
    return mkdir(path.c_str(), 0777) == 0 || errno == EEXIST;
#endif
}

// 設定情報をファイルに出力する関数
void WriteConfigFile() {
    if (!g_configFile) return;
    
    *g_configFile << "# WiFi AP Selection Simulation Configuration" << std::endl;
    *g_configFile << "# Generated on: " << g_timestamp << std::endl;
    *g_configFile << std::endl;
    
    *g_configFile << "[Simulation Parameters]" << std::endl;
    *g_configFile << "Number_of_APs=" << g_nAPs << std::endl;
    *g_configFile << "Number_of_Total_Users=" << g_nUsers << std::endl;
    *g_configFile << "Number_of_Existing_Users=" << g_nExistingUsers << std::endl;
    *g_configFile << "Number_of_New_Users=" << g_nNewUsers << std::endl;
    *g_configFile << "Simulation_Time_sec=" << std::fixed << std::setprecision(1) << g_simTime << std::endl;
    *g_configFile << "Simulation_Area=30x30m" << std::endl;
    *g_configFile << "WiFi_Standard=802.11a" << std::endl;
    *g_configFile << std::endl;
    
    *g_configFile << "[AP Selection Algorithm Parameters]" << std::endl;
    *g_configFile << "Distance_Threshold_m=25.0" << std::endl;
    *g_configFile << "Min_Required_Throughput_Mbps=10.0" << std::endl;
    
    if (g_algorithm) {
        const std::vector<double>& weights = g_algorithm->getWeights();
        *g_configFile << "Weight_Throughput=" << std::fixed << std::setprecision(1) << weights[0] << std::endl;
        *g_configFile << "Weight_Distance=" << weights[1] << std::endl;
        *g_configFile << "Weight_Channel=" << weights[2] << std::endl;
        *g_configFile << "Weight_Users=" << weights[3] << std::endl;
    }
    *g_configFile << std::endl;
    
    *g_configFile << "[AP Configuration]" << std::endl;
    for (uint32_t i = 0; i < g_nAPs; ++i) {
        Vector pos = g_apNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        *g_configFile << "AP" << i << "_Position_x=" << std::fixed << std::setprecision(1) << pos.x << std::endl;
        *g_configFile << "AP" << i << "_Position_y=" << pos.y << std::endl;
        *g_configFile << "AP" << i << "_Connected_Users=" << g_apInfoList[i].connectedUsers << std::endl;
        *g_configFile << "AP" << i << "_Channel_Utilization=" << std::setprecision(3) << g_apInfoList[i].channelUtilization << std::endl;
        *g_configFile << "AP" << i << "_Channel=" << g_apInfoList[i].channel << std::endl;
    }
    *g_configFile << std::endl;
    
    *g_configFile << "[User Initial Configuration]" << std::endl;
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Vector pos = g_staNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        *g_configFile << "User" << i << "_Initial_Position_x=" << std::fixed << std::setprecision(1) << pos.x << std::endl;
        *g_configFile << "User" << i << "_Initial_Position_y=" << pos.y << std::endl;
        *g_configFile << "User" << i << "_Type=" << (g_userInfoList[i].isNew ? "New" : "Existing") << std::endl;
        if (!g_userInfoList[i].isNew) {
            *g_configFile << "User" << i << "_Initial_Connected_AP=" << g_userInfoList[i].connectedAP << std::endl;
        }
    }
    
    g_configFile->flush();
}

// 初期状態の表示
void PrintInitialState() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "         WiFi APセレクションシミュレーション 初期状態" << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    // シミュレーション設定
    std::cout << "\n【シミュレーション設定】" << std::endl;
    std::cout << "  AP数: " << g_nAPs << "台" << std::endl;
    std::cout << "  総ユーザー数: " << g_nUsers << "台" << std::endl;
    std::cout << "    - 既存接続済みユーザー: " << g_nExistingUsers << "台" << std::endl;
    std::cout << "    - 新規未接続ユーザー: " << g_nNewUsers << "台" << std::endl;
    std::cout << "  シミュレーション時間: " << std::fixed << std::setprecision(0) << g_simTime << "秒" << std::endl;
    std::cout << "  シミュレーション範囲: 30m × 30m" << std::endl;
    std::cout << "  出力ディレクトリ: " << g_outputDir << std::endl;

    // AP配置情報
    std::cout << "\n【AP配置情報】" << std::endl;
    for (uint32_t i = 0; i < g_nAPs; ++i) {
        Vector pos = g_apNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        std::cout << "  AP" << i << ": 位置(" << std::fixed << std::setprecision(1) 
                  << pos.x << "m, " << pos.y << "m)" 
                  << ", 接続ユーザー数:" << g_apInfoList[i].connectedUsers << "台"
                  << ", チャンネル利用率:" << std::setprecision(1) << (g_apInfoList[i].channelUtilization * 100) << "%" << std::endl;
    }

    // ユーザー分類表示
    std::cout << "\n【ユーザー配置情報】" << std::endl;
    
    // 既存ユーザーの表示
    std::cout << "\n  ＜既存接続済みユーザー＞" << std::endl;
    std::vector<std::vector<uint32_t>> apUsers(g_nAPs);
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (!g_userInfoList[i].isNew) {
            apUsers[g_userInfoList[i].connectedAP].push_back(i);
        }
    }

    for (uint32_t apId = 0; apId < g_nAPs; ++apId) {
        if (!apUsers[apId].empty()) {
            std::cout << "    AP" << apId << "接続ユーザー:" << std::endl;
            for (uint32_t userId : apUsers[apId]) {
                Vector pos = g_staNodes->Get(userId)->GetObject<MobilityModel>()->GetPosition();
                std::cout << "      ユーザー" << userId << ": 位置(" 
                          << std::fixed << std::setprecision(1)
                          << pos.x << "m, " << pos.y << "m)" << std::endl;
            }
        }
    }
    
    // 新規ユーザーの表示
    std::cout << "\n  ＜新規未接続ユーザー（AP選択対象）＞" << std::endl;
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (g_userInfoList[i].isNew) {
            Vector pos = g_staNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
            std::cout << "    ユーザー" << i << ": 位置(" 
                      << std::fixed << std::setprecision(1)
                      << pos.x << "m, " << pos.y << "m) [未接続]" << std::endl;
        }
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
    
    // 新規ユーザーのみAP選択を実行
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Ptr<MobilityModel> mobility = g_staNodes->Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();

        if (g_userInfoList[i].isNew) {
            // 新規ユーザーのAP選択
            APSelectionResult result = g_algorithm->SelectOptimalAPDetailed(pos, i, true);
            results.push_back(result);
            
            // ファイルにも出力
            *g_resultFile << "5.0\t" << i << "\t(" << pos.x << "," << pos.y << ")\t" 
                         << result.selectedAP << "\t" << result.expectedThroughput << "\tNew" << std::endl;
        } else {
            // 既存ユーザーは現在の接続を維持
            APSelectionResult result;
            result.userId = i;
            result.userPosition = pos;
            result.selectedAP = g_userInfoList[i].connectedAP;
            result.isNewUser = false;
            
            // 既存ユーザーの期待スループットも計算
            Vector apPos = g_apNodes->Get(result.selectedAP)->GetObject<MobilityModel>()->GetPosition();
            result.distance = g_algorithm->CalculateDistance(pos, apPos);
            double rate = g_algorithm->CalculateTransmissionRate(pos, apPos);
            result.expectedThroughput = g_algorithm->CalculateThroughput(g_apInfoList[result.selectedAP], rate);
            
            *g_resultFile << "5.0\t" << i << "\t(" << pos.x << "," << pos.y << ")\t" 
                         << result.selectedAP << "\t" << result.expectedThroughput << "\tExisting" << std::endl;
        }
    }

    // 新規ユーザーの詳細結果表示
    std::cout << "\n【新規ユーザーのAP選択結果】" << std::endl;
    for (const auto& result : results) {
        if (result.isNewUser) {
            std::cout << "\n--- ユーザー" << result.userId << " （新規）---" << std::endl;
            std::cout << "  現在位置: (" << std::fixed << std::setprecision(1) 
                      << result.userPosition.x << "m, " << result.userPosition.y << "m)" << std::endl;
            std::cout << "  選択されたAP: AP" << result.selectedAP << std::endl;
            std::cout << "  APまでの距離: " << std::setprecision(1) << result.distance << "m" << std::endl;
            std::cout << "  予想スループット: " << std::setprecision(1) << result.expectedThroughput << "Mbps" << std::endl;
            std::cout << "  総合スコア: " << std::setprecision(3) << result.score << std::endl;
            
            // 全APのスコア表示
            std::cout << "  各APスコア詳細:" << std::endl;
            for (const auto& score : result.allScores) {
                Vector apPos = g_apNodes->Get(score.first)->GetObject<MobilityModel>()->GetPosition();
                double dist = sqrt(pow(result.userPosition.x - apPos.x, 2) + 
                                 pow(result.userPosition.y - apPos.y, 2));
                std::cout << "    AP" << score.first << ": スコア=" << std::setprecision(3) << score.second
                          << " (距離:" << std::setprecision(1) << dist << "m)" << std::endl;
            }
        }
    }

    // 統計情報
    std::cout << "\n【AP選択統計】" << std::endl;
    std::map<uint32_t, uint32_t> apSelectionCount;
    std::map<uint32_t, uint32_t> apNewUserCount;
    
    for (uint32_t i = 0; i < g_nAPs; ++i) {
        apSelectionCount[i] = 0;
        apNewUserCount[i] = 0;
    }
    
    // 既存ユーザーと新規ユーザーの分布をカウント
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (g_userInfoList[i].isNew) {
            // 新規ユーザーは選択されたAPをカウント
            for (const auto& result : results) {
                if (result.userId == i) {
                    apSelectionCount[result.selectedAP]++;
                    apNewUserCount[result.selectedAP]++;
                    break;
                }
            }
        } else {
            // 既存ユーザーは元の接続APをカウント
            apSelectionCount[g_userInfoList[i].connectedAP]++;
        }
    }

    for (uint32_t i = 0; i < g_nAPs; ++i) {
        double percentage = (double)apSelectionCount[i] / g_nUsers * 100.0;
        std::cout << "  AP" << i << ": " << apSelectionCount[i] << "ユーザー合計 ("
                  << std::fixed << std::setprecision(1) << percentage << "%) "
                  << "[既存:" << (apSelectionCount[i] - apNewUserCount[i]) 
                  << ", 新規:" << apNewUserCount[i] << "]" << std::endl;
    }

    // 平均距離とスループット（新規ユーザーのみ）
    double totalDistance = 0.0, totalThroughput = 0.0;
    uint32_t newUserCount = 0;
    
    for (const auto& result : results) {
        if (result.isNewUser) {
            totalDistance += result.distance;
            totalThroughput += result.expectedThroughput;
            newUserCount++;
        }
    }
    
    std::cout << "\n【新規ユーザー性能指標】" << std::endl;
    if (newUserCount > 0) {
        std::cout << "  平均AP距離: " << std::setprecision(1) << (totalDistance / newUserCount) << "m" << std::endl;
        std::cout << "  平均予想スループット: " << std::setprecision(1) << (totalThroughput / newUserCount) << "Mbps" << std::endl;
    } else {
        std::cout << "  新規ユーザーが存在しません" << std::endl;
    }

    std::cout << std::string(80, '=') << std::endl;
    
    g_resultFile->flush();
}

int main(int argc, char *argv[]) {
    // パラメータ設定
    uint32_t nAPs = 5;
    uint32_t nExistingUsers = 13; // 既存接続済みユーザー (4+2+3+4+0)
    uint32_t nNewUsers = 5; // 新規未接続ユーザー
    uint32_t nUsers = nExistingUsers + nNewUsers; // 総ユーザー数
    double simTime = 60.0; // 60秒
    
    // グローバル変数設定
    g_nAPs = nAPs;
    g_nUsers = nUsers;
    g_nExistingUsers = nExistingUsers;
    g_nNewUsers = nNewUsers;
    g_simTime = simTime;

    // タイムスタンプ生成
    g_timestamp = GetTimestamp();

    // 結果出力ディレクトリ作成
    g_outputDir = "results";
    if (!CreateDirectory(g_outputDir)) {
        std::cerr << "Failed to create output directory: " << g_outputDir << std::endl;
        return 1;
    }

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
    apPositionAlloc->Add(Vector(15.0, 15.0, 0.0)); // AP4 (中央)

    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // STA配置
    Ptr<ListPositionAllocator> staPositionAlloc = CreateObject<ListPositionAllocator>();

    // 既存ユーザーの配置（各APに接続済み）
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

    // 新規ユーザーの配置（まだどこにも接続していない）
    // シミュレーション領域内にランダムまたは特定位置に配置
    staPositionAlloc->Add(Vector(12.0, 8.0, 0.0));   // 新規ユーザー1
    staPositionAlloc->Add(Vector(18.0, 12.0, 0.0));  // 新規ユーザー2
    staPositionAlloc->Add(Vector(8.0, 18.0, 0.0));   // 新規ユーザー3
    staPositionAlloc->Add(Vector(22.0, 15.0, 0.0));  // 新規ユーザー4
    staPositionAlloc->Add(Vector(15.0, 25.0, 0.0));  // 新規ユーザー5

    mobility.SetPositionAllocator(staPositionAlloc);
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Bounds", RectangleValue(Rectangle(0, 30, 0, 30)),
                              "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    mobility.Install(staNodes);

    // ユーザー情報の初期化
    g_userInfoList.resize(nUsers);
    
    // 既存ユーザーの設定
    uint32_t userIndex = 0;
    // AP0接続ユーザー（4名）
    for (uint32_t i = 0; i < 4; ++i, ++userIndex) {
        g_userInfoList[userIndex].userId = userIndex;
        g_userInfoList[userIndex].connectedAP = 0;
        g_userInfoList[userIndex].isNew = false;
        g_userInfoList[userIndex].position = staNodes.Get(userIndex)->GetObject<MobilityModel>()->GetPosition();
    }
    
    // AP1接続ユーザー（2名）
    for (uint32_t i = 0; i < 2; ++i, ++userIndex) {
        g_userInfoList[userIndex].userId = userIndex;
        g_userInfoList[userIndex].connectedAP = 1;
        g_userInfoList[userIndex].isNew = false;
        g_userInfoList[userIndex].position = staNodes.Get(userIndex)->GetObject<MobilityModel>()->GetPosition();
    }
    
    // AP2接続ユーザー（3名）
    for (uint32_t i = 0; i < 3; ++i, ++userIndex) {
        g_userInfoList[userIndex].userId = userIndex;
        g_userInfoList[userIndex].connectedAP = 2;
        g_userInfoList[userIndex].isNew = false;
        g_userInfoList[userIndex].position = staNodes.Get(userIndex)->GetObject<MobilityModel>()->GetPosition();
    }
    
    // AP3接続ユーザー（4名）
    for (uint32_t i = 0; i < 4; ++i, ++userIndex) {
        g_userInfoList[userIndex].userId = userIndex;
        g_userInfoList[userIndex].connectedAP = 3;
        g_userInfoList[userIndex].isNew = false;
        g_userInfoList[userIndex].position = staNodes.Get(userIndex)->GetObject<MobilityModel>()->GetPosition();
    }
    
    // 新規ユーザーの設定（AP4には最初誰も接続していない）
    for (uint32_t i = 0; i < nNewUsers; ++i, ++userIndex) {
        g_userInfoList[userIndex].userId = userIndex;
        g_userInfoList[userIndex].connectedAP = 0; // 初期値、実際は未接続
        g_userInfoList[userIndex].isNew = true;
        g_userInfoList[userIndex].position = staNodes.Get(userIndex)->GetObject<MobilityModel>()->GetPosition();
    }

    // AP情報の初期化
    g_apInfoList.resize(nAPs);
    
    // 既存ユーザーの負荷を反映したAP情報設定
    uint32_t userCounts[] = {4, 2, 3, 4, 0}; // 各APの既存接続ユーザー数
    double utilizations[] = {0.6, 0.3, 0.45, 0.8, 0.0}; // チャンネル利用率
    
    for (uint32_t i = 0; i < nAPs; ++i) {
        g_apInfoList[i].apId = i;
        g_apInfoList[i].position = apNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        g_apInfoList[i].connectedUsers = userCounts[i];
        g_apInfoList[i].channelUtilization = utilizations[i];
        g_apInfoList[i].channel = i % 3; // 3つのチャンネル
    }

    // アルゴリズム初期化
    APSelectionAlgorithm algorithm(25.0, 10.0);    // 距離閾値25m, 最低スループット10Mbps
    algorithm.UpdateAPInfo(g_apInfoList);
    g_algorithm = &algorithm;

    // 結果ファイル設定
    std::string resultFileName = g_outputDir + "/ap_selection_results_" + g_timestamp + ".txt";
    std::string configFileName = g_outputDir + "/simulation_config_" + g_timestamp + ".txt";
    std::string animFileName = g_outputDir + "/animation_" + g_timestamp + ".xml";

    std::ofstream resultFile(resultFileName);
    std::ofstream configFile(configFileName);
    g_resultFile = &resultFile;
    g_configFile = &configFile;

    // 設定ファイル書き込み
    WriteConfigFile();
    configFile.close();

    resultFile << "Time(s)\tUserID\tPosition(x,y)\tSelectedAP\tThroughput(Mbps)\tUserType" << std::endl;

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
    AnimationInterface anim(animFileName);
    // APを緑色で表示
    for (uint32_t i = 0; i < nAPs; ++i) {
        anim.UpdateNodeColor(apNodes.Get(i), 0, 255, 0);
    }
    
    // 既存ユーザーを青色、新規ユーザーを赤色で表示
    for (uint32_t i = 0; i < nUsers; ++i) {
        if (g_userInfoList[i].isNew) {
            anim.UpdateNodeColor(staNodes.Get(i), 255, 0, 0); // 新規ユーザー：赤
        } else {
            anim.UpdateNodeColor(staNodes.Get(i), 0, 0, 255); // 既存ユーザー：青
        }
    }

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
    std::cout << "  出力ディレクトリ: " << g_outputDir << std::endl;
    std::cout << "  結果ファイル: " << resultFileName << std::endl;
    std::cout << "  設定ファイル: " << configFileName << std::endl;
    std::cout << "  アニメーションファイル: " << animFileName << std::endl;
    std::cout << std::string(80, '=') << std::endl;

    resultFile.close();
    Simulator::Destroy();

    return 0;
}

//コマンドラインオプションで
//シミュレーション時間を指定
// CommandLine cmd;
// cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
// cmd.Parse(argc, argv);