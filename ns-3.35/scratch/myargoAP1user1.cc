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
    std::vector<double> userRates; // ユーザー毎のスループット
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

// カスタム移動モデル：APの方向に向かって移動
class APDirectedMobilityModel : public MobilityModel {
private:
    Vector m_currentPosition;
    Vector m_targetPosition; // APの位置
    double m_speed; // 移動速度 (m/s)
    EventId m_moveEvent;
    Time m_moveInterval;
    bool m_isMoving;
    
public:
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("APDirectedMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<APDirectedMobilityModel>()
            .AddAttribute("Speed", "移動速度 (m/s)",
                         DoubleValue(1.0),
                         MakeDoubleAccessor(&APDirectedMobilityModel::m_speed),
                         MakeDoubleChecker<double>())
            .AddAttribute("MoveInterval", "移動更新間隔",
                         TimeValue(Seconds(0.1)),
                         MakeTimeAccessor(&APDirectedMobilityModel::m_moveInterval),
                         MakeTimeChecker());
        return tid;
    }
    
    APDirectedMobilityModel() : m_speed(1.0), m_moveInterval(Seconds(0.1)), m_isMoving(false) {}
    
    void SetTargetPosition(const Vector& target) {
        m_targetPosition = target;
        if (!m_isMoving) {
            StartMoving();
        }
    }
    
    void StartMoving() {
        m_isMoving = true;
        ScheduleMove();
    }
    
    void StopMoving() {
        m_isMoving = false;
        if (m_moveEvent.IsRunning()) {
            Simulator::Cancel(m_moveEvent);
        }
    }
    
private:
    void ScheduleMove() {
        if (!m_isMoving) return;
        m_moveEvent = Simulator::Schedule(m_moveInterval, &APDirectedMobilityModel::DoMove, this);
    }
    
    void DoMove() {
        // 現在位置から目標位置への方向ベクトルを計算
        Vector direction = m_targetPosition - m_currentPosition;
        double distance = sqrt(direction.x * direction.x + direction.y * direction.y);
        
        if (distance > 0.5) { // 目標に十分近づいていない場合
            // 正規化された方向ベクトル
            direction.x /= distance;
            direction.y /= distance;
            direction.z = 0;
            
            // 移動距離を計算
            double moveDistance = m_speed * m_moveInterval.GetSeconds();
            
            // 新しい位置を計算
            Vector newPosition = m_currentPosition;
            newPosition.x += direction.x * moveDistance;
            newPosition.y += direction.y * moveDistance;
            
            // 境界チェック（0-30mの範囲内に制限）
            newPosition.x = std::max(0.0, std::min(30.0, newPosition.x));
            newPosition.y = std::max(0.0, std::min(30.0, newPosition.y));
            
            SetPosition(newPosition);
            
            // 次の移動をスケジュール
            ScheduleMove();
        } else {
            // 目標に到達したら移動を停止
            m_isMoving = false;
        }
    }
    
    virtual Vector DoGetPosition() const {
        return m_currentPosition;
    }
    
    virtual void DoSetPosition(const Vector& position) {
        m_currentPosition = position;
        NotifyCourseChange();
    }
    
    virtual Vector DoGetVelocity() const {
        if (!m_isMoving) return Vector(0, 0, 0);
        
        Vector direction = m_targetPosition - m_currentPosition;
        double distance = sqrt(direction.x * direction.x + direction.y * direction.y);
        
        if (distance > 0) {
            direction.x = (direction.x / distance) * m_speed;
            direction.y = (direction.y / distance) * m_speed;
            direction.z = 0;
        }
        
        return direction;
    }
};

class APSelectionAlgorithm {
private:
    std::vector<APInfo> m_apList;
    double m_dThreshold;
    double m_thetaThreshold;
    std::vector<double> m_weights;

public:
    APSelectionAlgorithm(double dTh, double thetaTh) 
        : m_dThreshold(dTh), m_thetaThreshold(thetaTh) {
        m_weights = {0.5, 0.4, 0.1, 0.0};
    }
    
    const std::vector<double>& getWeights() const { return m_weights; }

    void UpdateAPInfo(const std::vector<APInfo>& apList) {
        m_apList = apList;
    }

    double CalculateDistance(const Vector& pos1, const Vector& pos2) {
        return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
    }

    double CalculateTransmissionRate(const Vector& userPos, const Vector& apPos) {
        double distance = CalculateDistance(userPos, apPos);
        if (distance < 5.0) return 150.0;
        else if (distance < 10.0) return 130.0;
        else if (distance < 15.0) return 100.0;
        else if (distance < 20.0) return 65.0;
        else if (distance < 25.0) return 30.0;
        else return 6.5;
    }

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

    APSelectionResult SelectOptimalAPDetailed(const Vector& userPos, uint32_t userId) {
        APSelectionResult result;
        result.userId = userId;
        result.userPosition = userPos;
        result.selectedAP = 0;
        result.expectedThroughput = 0.0;
        result.distance = 0.0;
        result.score = 0.0;

        std::vector<uint32_t> candidates;

        for (const auto& ap : m_apList) {
            double distance = CalculateDistance(userPos, ap.position);
            if (distance <= m_dThreshold) {
                candidates.push_back(ap.apId);
            }
        }

        if (candidates.empty()) {
            return result;
        }

        std::vector<std::pair<uint32_t, double>> apScores;
        double thetaMax = 0, thetaMin = 1e9;
        double dMax = 0, dMin = 1e9;
        uint32_t nMax = 0;

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

        for (uint32_t apId : candidates) {
            const APInfo& ap = m_apList[apId];
            double newRate = CalculateTransmissionRate(userPos, ap.position);
            double throughput = CalculateThroughput(ap, newRate);
            double distance = CalculateDistance(userPos, ap.position);

            double scoreTheta = (thetaMax == thetaMin) ? 1.0 : 
                               (throughput - thetaMin) / (thetaMax - thetaMin);
            double scoreDist = (dMax == dMin) ? 1.0 : 
                              1.0 - (distance - dMin) / (dMax - dMin);
            double scoreChan = 1.0 - ap.channelUtilization;
            double scoreUsers = (nMax == 0) ? 1.0 : 
                               1.0 - (double)ap.connectedUsers / nMax;

            double totalScore = m_weights[0] * scoreTheta + 
                               m_weights[1] * scoreDist + 
                               m_weights[2] * scoreChan + 
                               m_weights[3] * scoreUsers;

            apScores.push_back(std::make_pair(apId, totalScore));
            result.allScores.push_back(std::make_pair(apId, totalScore));
        }

        std::sort(apScores.begin(), apScores.end(), 
                 [](const auto& a, const auto& b) { return a.second > b.second; });

        result.selectedAP = apScores[0].first;
        result.score = apScores[0].second;
        result.distance = CalculateDistance(userPos, m_apList[result.selectedAP].position);
        result.expectedThroughput = CalculateThroughput(m_apList[result.selectedAP], 
            CalculateTransmissionRate(userPos, m_apList[result.selectedAP].position));

        return result;
    }

    std::pair<uint32_t, Vector> SelectOptimalAP(const Vector& userPos) {
        APSelectionResult result = SelectOptimalAPDetailed(userPos, 0);
        return std::make_pair(result.selectedAP, Vector(0, 0, 0));
    }
};

// グローバル変数
static std::ofstream* g_resultFile = nullptr;
static std::ofstream* g_configFile = nullptr;
static std::ofstream* g_terminalOutputFile = nullptr; // ターミナル出力用ファイル
static NodeContainer* g_apNodes = nullptr;
static NodeContainer* g_staNodes = nullptr;
static uint32_t g_nAPs = 0;
static uint32_t g_nUsers = 0;
static double g_simTime = 0.0;
static APSelectionAlgorithm* g_algorithm = nullptr;
static std::vector<APInfo> g_apInfoList;
static std::string g_outputDir = "";
static std::string g_timestamp = "";
static std::vector<Ptr<APDirectedMobilityModel>> g_userMobilityModels;

// 画面とファイルの両方に出力する関数
void PrintToScreenAndFile(const std::string& message) {
    std::cout << message;
    if (g_terminalOutputFile) {
        *g_terminalOutputFile << message;
        g_terminalOutputFile->flush();
    }
}

// std::coutの代わりに使用するマクロ
#define DUAL_COUT(x) do { \
    std::ostringstream oss; \
    oss << x; \
    PrintToScreenAndFile(oss.str()); \
} while(0)

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
    *g_configFile << "Number_of_Users=" << g_nUsers << std::endl;
    *g_configFile << "Simulation_Time_sec=" << std::fixed << std::setprecision(1) << g_simTime << std::endl;
    *g_configFile << "Simulation_Area=30x30m" << std::endl;
    *g_configFile << "WiFi_Standard=802.11a" << std::endl;
    *g_configFile << "User_Mobility=AP_Directed_Movement" << std::endl;
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
    }
    
    g_configFile->flush();
}

// 初期状態の表示
void PrintInitialState() {
    DUAL_COUT("\n" << std::string(80, '=') << std::endl);
    DUAL_COUT("       シンプル WiFi APセレクションシミュレーション 初期状態" << std::endl);
    DUAL_COUT(std::string(80, '=') << std::endl);

    // シミュレーション設定
    DUAL_COUT("\n【シミュレーション設定】" << std::endl);
    DUAL_COUT("  AP数: " << g_nAPs << "台" << std::endl);
    DUAL_COUT("  ユーザー数: " << g_nUsers << "台" << std::endl);
    DUAL_COUT("  シミュレーション時間: " << std::fixed << std::setprecision(0) << g_simTime << "秒" << std::endl);
    DUAL_COUT("  シミュレーション範囲: 30m × 30m" << std::endl);
    DUAL_COUT("  移動モデル: APに向かって移動" << std::endl);
    DUAL_COUT("  出力ディレクトリ: " << g_outputDir << std::endl);

    // AP配置情報
    DUAL_COUT("\n【AP配置情報】" << std::endl);
    for (uint32_t i = 0; i < g_nAPs; ++i) {
        Vector pos = g_apNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        DUAL_COUT("  AP" << i << ": 位置(" << std::fixed << std::setprecision(1) 
                  << pos.x << "m, " << pos.y << "m)" 
                  << ", 接続ユーザー数:" << g_apInfoList[i].connectedUsers << "台"
                  << ", チャンネル利用率:" << std::setprecision(1) << (g_apInfoList[i].channelUtilization * 100) << "%" << std::endl);
    }

    // ユーザー初期位置
    DUAL_COUT("\n【ユーザー初期配置】" << std::endl);
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Vector pos = g_staNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        DUAL_COUT("  ユーザー" << i << ": 位置(" 
                  << std::fixed << std::setprecision(1)
                  << pos.x << "m, " << pos.y << "m)" << std::endl);
    }

    // アルゴリズム設定
    DUAL_COUT("\n【AP選択アルゴリズム設定】" << std::endl);
    DUAL_COUT("  距離閾値: 25.0m" << std::endl);
    DUAL_COUT("  最低要求スループット: 10.0Mbps" << std::endl);
    
    if (g_algorithm) {
        const std::vector<double>& weights = g_algorithm->getWeights();
        DUAL_COUT("  スコア重み: [スループット:" << std::fixed << std::setprecision(1) << weights[0] 
                  << ", 距離:" << weights[1] 
                  << ", チャンネル:" << weights[2] 
                  << ", ユーザー数:" << weights[3] << "]" << std::endl);
    }
    
    DUAL_COUT(std::string(80, '=') << std::endl);
}

// ユーザーをAPの近くに移動させる関数
void StartUserMovement() {
    DUAL_COUT("\n【ユーザー移動開始】" << std::endl);
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (i < g_userMobilityModels.size()) {
            Vector apPosition = g_apNodes->Get(0)->GetObject<MobilityModel>()->GetPosition();
            
            Vector targetPosition = apPosition;
            targetPosition.x += 3.0;
            targetPosition.y += 3.0;
            
            targetPosition.x = std::max(0.0, std::min(30.0, targetPosition.x));
            targetPosition.y = std::max(0.0, std::min(30.0, targetPosition.y));
            
            g_userMobilityModels[i]->SetTargetPosition(targetPosition);
            
            Vector currentPos = g_staNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
            DUAL_COUT("  ユーザー" << i << ": (" << std::setprecision(1) << currentPos.x 
                      << "m, " << currentPos.y << "m) -> (" << targetPosition.x 
                      << "m, " << targetPosition.y << "m)に移動開始" << std::endl);
        }
    }
}

// AP選択結果の詳細表示
void PrintAPSelectionResults() {
    if (!g_resultFile || !g_apNodes || !g_staNodes || !g_algorithm) return;
    
    DUAL_COUT("\n" << std::string(80, '=') << std::endl);
    DUAL_COUT("              AP選択アルゴリズム実行結果" << std::endl);
    DUAL_COUT(std::string(80, '=') << std::endl);

    std::vector<APSelectionResult> results;
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Ptr<MobilityModel> mobility = g_staNodes->Get(i)->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();

        APSelectionResult result = g_algorithm->SelectOptimalAPDetailed(pos, i);
        results.push_back(result);
        
        double currentTime = Simulator::Now().GetSeconds();
        *g_resultFile << currentTime << "\t" << i << "\t(" << pos.x << "," << pos.y << ")\t" 
                     << result.selectedAP << "\t" << result.expectedThroughput << std::endl;
    }

    // ユーザー毎の詳細結果表示
    DUAL_COUT("\n【各ユーザーのAP選択結果】" << std::endl);
    for (const auto& result : results) {
        DUAL_COUT("\n--- ユーザー" << result.userId << " ---" << std::endl);
        DUAL_COUT("  現在位置: (" << std::fixed << std::setprecision(1) 
                  << result.userPosition.x << "m, " << result.userPosition.y << "m)" << std::endl);
        DUAL_COUT("  選択されたAP: AP" << result.selectedAP << std::endl);
        DUAL_COUT("  APまでの距離: " << std::setprecision(1) << result.distance << "m" << std::endl);
        DUAL_COUT("  予想スループット: " << std::setprecision(1) << result.expectedThroughput << "Mbps" << std::endl);
        DUAL_COUT("  総合スコア: " << std::setprecision(3) << result.score << std::endl);
        
        DUAL_COUT("  APスコア詳細:" << std::endl);
        for (const auto& score : result.allScores) {
            Vector apPos = g_apNodes->Get(score.first)->GetObject<MobilityModel>()->GetPosition();
            double dist = sqrt(pow(result.userPosition.x - apPos.x, 2) + 
                             pow(result.userPosition.y - apPos.y, 2));
            DUAL_COUT("    AP" << score.first << ": スコア=" << std::setprecision(3) << score.second
                      << " (距離:" << std::setprecision(1) << dist << "m)" << std::endl);
        }
    }

    // 統計情報
    DUAL_COUT("\n【AP選択統計】" << std::endl);
    DUAL_COUT("  AP0: " << g_nUsers << "ユーザー選択 (100.0%)" << std::endl);

    // 平均距離とスループット
    double totalDistance = 0.0, totalThroughput = 0.0;
    for (const auto& result : results) {
        totalDistance += result.distance;
        totalThroughput += result.expectedThroughput;
    }
    
    DUAL_COUT("\n【性能指標】" << std::endl);
    DUAL_COUT("  平均AP距離: " << std::setprecision(1) << (totalDistance / g_nUsers) << "m" << std::endl);
    DUAL_COUT("  平均予想スループット: " << std::setprecision(1) << (totalThroughput / g_nUsers) << "Mbps" << std::endl);

    DUAL_COUT(std::string(80, '=') << std::endl);
    
    g_resultFile->flush();
}

// 定期的な位置と結果の出力
void PeriodicOutput() {
    DUAL_COUT("\n【時刻 " << std::fixed << std::setprecision(1) 
              << Simulator::Now().GetSeconds() << "秒 - 位置更新】" << std::endl);
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Vector pos = g_staNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        DUAL_COUT("  ユーザー" << i << ": (" << std::setprecision(1) 
                  << pos.x << "m, " << pos.y << "m)" << std::endl);
    }
    
    PrintAPSelectionResults();
    
    if (Simulator::Now().GetSeconds() < g_simTime - 5.0) {
        Simulator::Schedule(Seconds(5.0), &PeriodicOutput);
    }
}

int main(int argc, char *argv[]) {
    // パラメータ設定
    uint32_t nAPs = 2;
    uint32_t nUsers = 1;
    double simTime = 60.0;

    // グローバル変数設定
    g_nAPs = nAPs;
    g_nUsers = nUsers;
    g_simTime = simTime;

    // タイムスタンプ生成
    g_timestamp = GetTimestamp();

    // 結果出力ディレクトリ作成
    g_outputDir = "results";
    if (!CreateDirectory(g_outputDir)) {
        std::cerr << "Failed to create output directory: " << g_outputDir << std::endl;
        return 1;
    }

    // ファイル名設定
    std::string resultFileName = g_outputDir + "/AP1user1_" + g_timestamp + ".txt";
    std::string configFileName = g_outputDir + "/AP1user1_simulation_config_" + g_timestamp + ".txt";
    std::string terminalOutputFileName = g_outputDir + "/AP1user1_terminal_output_" + g_timestamp + ".txt";
    std::string animFileName = g_outputDir + "/AP1user1_animation_" + g_timestamp + ".xml";

    // ファイル開く
    std::ofstream resultFile(resultFileName);
    std::ofstream configFile(configFileName);
    std::ofstream terminalOutputFile(terminalOutputFileName);
    g_resultFile = &resultFile;
    g_configFile = &configFile;
    g_terminalOutputFile = &terminalOutputFile;

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
    apPositionAlloc->Add(Vector(5.0, 5.0, 0.0));   // AP1 (中央)

    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // STA配置（APから離れた位置に1台）- カスタム移動モデルを使用
    Ptr<ListPositionAllocator> staPositionAlloc = CreateObject<ListPositionAllocator>();
    staPositionAlloc->Add(Vector(9.0, 9.0, 0.0));  // ユーザー0（APから離れた位置）

    mobility.SetPositionAllocator(staPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(staNodes);

    // カスタム移動モデルに置き換え
    g_userMobilityModels.resize(nUsers);
    for (uint32_t i = 0; i < nUsers; ++i) {
        Ptr<APDirectedMobilityModel> customMobility = CreateObject<APDirectedMobilityModel>();
        // 初期位置を設定
        Vector initialPos = staNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        customMobility->SetPosition(initialPos);
        
        // 移動速度を設定（2 m/s）
        customMobility->SetAttribute("Speed", DoubleValue(2.0));
        customMobility->SetAttribute("MoveInterval", TimeValue(Seconds(0.1)));
        
        staNodes.Get(i)->AggregateObject(customMobility);
        g_userMobilityModels[i] = customMobility;
    }

    // AP情報の初期化
    g_apInfoList.resize(nAPs);
    g_apInfoList[0].apId = 0;
    g_apInfoList[0].position = apNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
    g_apInfoList[0].connectedUsers = 0;
    g_apInfoList[0].channelUtilization = 0.1;
    g_apInfoList[0].channel = 0;

    // アルゴリズム初期化
    APSelectionAlgorithm algorithm(25.0, 10.0);
    algorithm.UpdateAPInfo(g_apInfoList);
    g_algorithm = &algorithm;

    // 設定ファイル書き込み
    WriteConfigFile();
    configFile.close();

    resultFile << "Time(s)\tUserID\tPosition(x,y)\tSelectedAP\tThroughput(Mbps)" << std::endl;

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
    anim.UpdateNodeColor(apNodes.Get(0), 0, 255, 0);  // APを緑色
    anim.UpdateNodeColor(staNodes.Get(0), 255, 0, 0); // ユーザーを赤色

    // シミュレーション実行
    Simulator::Stop(Seconds(simTime));

    // 初期AP選択結果出力
    Simulator::Schedule(Seconds(1.0), &PrintAPSelectionResults);

    // ユーザー移動開始（2秒後）
    Simulator::Schedule(Seconds(2.0), &StartUserMovement);

    // 定期的な位置と結果の出力（2秒後から5秒間隔）
    Simulator::Schedule(Seconds(2.0), &PeriodicOutput);

    Simulator::Run();

    // FlowMonitor結果出力
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    DUAL_COUT("\n" << std::string(80, '=') << std::endl);
    DUAL_COUT("                シミュレーション最終結果" << std::endl);
    DUAL_COUT(std::string(80, '=') << std::endl);
    DUAL_COUT("\n【実測通信性能】" << std::endl);
    
    double totalRealThroughput = 0.0;
    uint32_t flowCount = 0;
    
    for (auto& flow : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        if (flow.second.timeLastRxPacket.GetSeconds() > flow.second.timeFirstTxPacket.GetSeconds()) {
            double throughput = flow.second.rxBytes * 8.0 / 
                (flow.second.timeLastRxPacket.GetSeconds() - flow.second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024;
            DUAL_COUT("  フロー " << flow.first << " (" << t.sourceAddress << " -> " << t.destinationAddress << "): "
                      << std::fixed << std::setprecision(2) << throughput << " Mbps" << std::endl);
            totalRealThroughput += throughput;
            flowCount++;
        }
    }
    
    if (flowCount > 0) {
        DUAL_COUT("\n  実測スループット: " << std::setprecision(2) << (totalRealThroughput / flowCount) << " Mbps" << std::endl);
    }

    // 最終位置表示
    DUAL_COUT("\n【最終位置】" << std::endl);
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Vector finalPos = g_staNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        Vector apPos = g_apNodes->Get(0)->GetObject<MobilityModel>()->GetPosition();
        double finalDistance = sqrt(pow(finalPos.x - apPos.x, 2) + pow(finalPos.y - apPos.y, 2));
        
        DUAL_COUT("  ユーザー" << i << ": (" << std::fixed << std::setprecision(1) 
                  << finalPos.x << "m, " << finalPos.y << "m)" 
                  << ", APまでの距離: " << finalDistance << "m" << std::endl);
    }

    DUAL_COUT("\n【シミュレーション完了】" << std::endl);
    DUAL_COUT("  出力ディレクトリ: " << g_outputDir << std::endl);
    DUAL_COUT("  結果ファイル: " << resultFileName << std::endl);
    DUAL_COUT("  設定ファイル: " << configFileName << std::endl);
    DUAL_COUT("  ターミナル出力ファイル: " << terminalOutputFileName << std::endl);
    DUAL_COUT("  アニメーションファイル: " << animFileName << std::endl);
    DUAL_COUT(std::string(80, '=') << std::endl);

    // ファイルを閉じる
    resultFile.close();
    terminalOutputFile.close();
    Simulator::Destroy();

    return 0;
}