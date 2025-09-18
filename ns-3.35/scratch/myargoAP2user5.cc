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
    uint32_t apId;
    Vector position;
    uint32_t connectedUsers;
    double channelUtilization;
    std::vector<double> userRates;
    uint32_t channel;
    double totalThroughput; 
};

// Userの情報（新規/既存の区別を追加）
struct UserInfo {
    uint32_t userId;
    Vector position;
    Vector initialPosition; 
    uint32_t connectedAP;
    double throughput;
    double throughputThreshold;
    bool hasReachedThreshold;
    bool isNewUser;  // 新規ユーザかどうか
};

// APごとのチャンネル使用率設定
struct APChannelConfig {
    uint32_t apId;
    double channelUtilization;
};

// AP選択結果を格納する構造体
struct APSelectionResult {
    uint32_t userId;
    Vector userPosition;
    uint32_t selectedAP;
    double expectedThroughput;
    double distance;
    double score;
    std::vector<std::pair<uint32_t, double>> allScores;
};

// カスタム移動モデル：APの方向に向かって移動
class APDirectedMobilityModel : public MobilityModel {
private:
    Vector m_currentPosition;
    Vector m_targetPosition;
    double m_speed;
    EventId m_moveEvent;
    Time m_moveInterval;
    bool m_isMoving;
    bool m_targetReached;
    
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
    
    APDirectedMobilityModel() : m_speed(1.0), m_moveInterval(Seconds(0.1)), m_isMoving(false), m_targetReached(false) {}
    
    void SetTargetPosition(const Vector& target) {
        m_targetPosition = target;
        m_targetReached = false;
        if (!m_isMoving) {
            StartMoving();
        }
    }
    
    void StartMoving() {
        m_isMoving = true;
        m_targetReached = false;
        ScheduleMove();
    }
    
    void StopMoving() {
        m_isMoving = false;
        if (m_moveEvent.IsRunning()) {
            Simulator::Cancel(m_moveEvent);
        }
    }
    
    bool HasReachedTarget() const {
        return m_targetReached;
    }
    
    bool IsMoving() const {
        return m_isMoving;
    }
    
private:
    void ScheduleMove() {
        if (!m_isMoving) return;
        m_moveEvent = Simulator::Schedule(m_moveInterval, &APDirectedMobilityModel::DoMove, this);
    }
    
    void DoMove() {
        Vector direction = m_targetPosition - m_currentPosition;
        double distance = sqrt(direction.x * direction.x + direction.y * direction.y);
        
        if (distance > 1.0) { // 目標に十分近づいていない場合
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
            m_targetReached = true;
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
            return newRate * (1.0 - ap.channelUtilization);
        }
        
        double sum = 0.0;
        for (double rate : ap.userRates) {
            sum += 1.0 / rate;
        }
        sum += 1.0 / newRate;
        
        double fairThroughput = (ap.userRates.size() + 1) / sum;
        return fairThroughput * (1.0 - ap.channelUtilization);
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
            // 候補がない場合は最も近いAPを選択
            double minDistance = 1e9;
            for (const auto& ap : m_apList) {
                double distance = CalculateDistance(userPos, ap.position);
                if (distance < minDistance) {
                    minDistance = distance;
                    result.selectedAP = ap.apId;
                    result.distance = distance;
                    result.expectedThroughput = CalculateThroughput(ap, CalculateTransmissionRate(userPos, ap.position));
                }
            }
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
};

// グローバル変数
static std::ofstream* g_resultFile = nullptr;
static std::ofstream* g_configFile = nullptr;
static std::ofstream* g_terminalOutputFile = nullptr;
static NodeContainer* g_apNodes = nullptr;
static NodeContainer* g_staNodes = nullptr;
static uint32_t g_nAPs = 0;
static uint32_t g_nUsers = 0;
static uint32_t g_nNewUsers = 0;  // 新規ユーザ数
static uint32_t g_nExistingUsers = 0;  // 既存ユーザ数
static double g_simTime = 0.0;
static APSelectionAlgorithm* g_algorithm = nullptr;
static std::vector<APInfo> g_apInfoList;
static std::vector<UserInfo> g_userInfoList;
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

#define DUAL_COUT(x) do { \
    std::ostringstream oss; \
    oss << x; \
    PrintToScreenAndFile(oss.str()); \
} while(0)

// システム全体のスループットを計算する関数（調和平均）
double CalculateSystemThroughput() {
    if (g_apInfoList.empty()) return 0.0;
    
    // 各APの総スループットを計算
    for (uint32_t apId = 0; apId < g_nAPs; ++apId) {
        double apTotalThroughput = 0.0;
        int connectedUsers = 0;
        
        // このAPに接続されているユーザーのスループットを合計
        for (uint32_t userId = 0; userId < g_nUsers; ++userId) {
            if (g_userInfoList[userId].connectedAP == apId) {
                apTotalThroughput += g_userInfoList[userId].throughput;
                connectedUsers++;
            }
        }
        
        g_apInfoList[apId].totalThroughput = apTotalThroughput;
        g_apInfoList[apId].connectedUsers = connectedUsers;
    }
    
    // 調和平均を計算
    double harmonicSum = 0.0;
    int activeAPs = 0;
    
    for (const auto& ap : g_apInfoList) {
        if (ap.totalThroughput > 0.0) {
            harmonicSum += 1.0 / ap.totalThroughput;
            activeAPs++;
        }
    }
    
    if (harmonicSum > 0.0 && activeAPs > 0) {
        return activeAPs / harmonicSum;
    }
    
    return 0.0;
}

// 移動ベクトルを計算する関数
Vector CalculateMovementVector(uint32_t userId) {
    if (userId >= g_nUsers) return Vector(0, 0, 0);
    
    Vector currentPos = g_userInfoList[userId].position;
    Vector initialPos = g_userInfoList[userId].initialPosition;
    
    return Vector(currentPos.x - initialPos.x, 
                  currentPos.y - initialPos.y, 
                  currentPos.z - initialPos.z);
}

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

// 現在のスループットを計算する関数（修正版）
double CalculateCurrentThroughput(uint32_t userId, uint32_t apId) {
    // ユーザIDとAPIDの有効性チェック
    if (userId >= g_nUsers || apId >= g_nAPs) {
        return 0.0;
    }
    
    Vector userPos;
    
    // 新規ユーザはMobilityModelから位置を取得、既存ユーザは固定位置を使用
    if (g_userInfoList[userId].isNewUser && g_userMobilityModels[userId]) {
        userPos = g_userMobilityModels[userId]->GetPosition();
    } else {
        userPos = g_userInfoList[userId].position;
    }
    
    // APの位置を取得（nullチェック付き）
    Ptr<MobilityModel> apMobility = g_apNodes->Get(apId)->GetObject<MobilityModel>();
    if (!apMobility) {
        return 0.0;
    }
    Vector apPos = apMobility->GetPosition();
    
    double distance = sqrt(pow(userPos.x - apPos.x, 2) + pow(userPos.y - apPos.y, 2));
    
    // 距離によるスループット計算
    double baseThroughput;
    if (distance < 5.0) baseThroughput = 150.0;
    else if (distance < 10.0) baseThroughput = 130.0;
    else if (distance < 15.0) baseThroughput = 100.0;
    else if (distance < 20.0) baseThroughput = 65.0;
    else if (distance < 25.0) baseThroughput = 30.0;
    else baseThroughput = 6.5;
    
    // チャンネル使用率による影響を考慮
    double channelEffect = 1.0 - g_apInfoList[apId].channelUtilization;
    
    return baseThroughput * channelEffect;
}

// UpdateUserMovement関数の修正（新規ユーザのみ移動）
void UpdateUserMovement() {
    // 各種チェックを追加
    if (!g_algorithm || g_userMobilityModels.empty() || !g_apNodes) {
        return;
    }
    
    bool anyNewUserMoving = false;
    
    DUAL_COUT("\n--- 時刻 " << std::fixed << std::setprecision(1) 
              << Simulator::Now().GetSeconds() << "秒 - 新規ユーザ移動更新 ---" << std::endl);
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        // 既存ユーザはスキップ
        if (!g_userInfoList[i].isNewUser) {
            continue;
        }
        
        // nullチェック
        if (!g_userMobilityModels[i]) {
            continue;
        }
        
        if (!g_userInfoList[i].hasReachedThreshold) {
            // カスタム移動モデルから現在位置を取得
            Vector userPos = g_userMobilityModels[i]->GetPosition();
            g_userInfoList[i].position = userPos;
            
            // 最適なAPを選択
            APSelectionResult result = g_algorithm->SelectOptimalAPDetailed(userPos, i);
            
            // 現在のスループットを計算
            double currentThroughput = CalculateCurrentThroughput(i, result.selectedAP);
            g_userInfoList[i].throughput = currentThroughput;
            
            DUAL_COUT("  新規ユーザ" << i << " - 現在位置: (" 
                      << std::setprecision(1) << userPos.x << "m, " << userPos.y << "m)" << std::endl);
            DUAL_COUT("  新規ユーザ" << i << " - 現在のスループット: " 
                      << std::setprecision(1) << currentThroughput << " Mbps"
                      << " (閾値: " << g_userInfoList[i].throughputThreshold << " Mbps)" << std::endl);
            
            if (currentThroughput >= g_userInfoList[i].throughputThreshold) {
                // スループット閾値に到達したら移動停止
                g_userInfoList[i].hasReachedThreshold = true;
                g_userMobilityModels[i]->StopMoving();
                DUAL_COUT("  新規ユーザ" << i << " - スループット閾値に到達！移動を停止します。" << std::endl);
            } else {
                // まだ閾値に到達していないなら最適なAPに向かって移動を続ける
                Ptr<MobilityModel> targetAPMobility = g_apNodes->Get(result.selectedAP)->GetObject<MobilityModel>();
                if (!targetAPMobility) {
                    continue;
                }
                Vector targetAP = targetAPMobility->GetPosition();
                
                // APの近く（2m以内）を目標にする
                Vector targetPosition = targetAP;
                targetPosition.x += (i % 2 == 0 ? 2.0 : -2.0);
                targetPosition.y += (i / 2 % 2 == 0 ? 2.0 : -2.0);
                
                // 境界チェック
                targetPosition.x = std::max(0.0, std::min(30.0, targetPosition.x));
                targetPosition.y = std::max(0.0, std::min(30.0, targetPosition.y));
                
                // 移動開始または継続
                if (!g_userMobilityModels[i]->IsMoving()) {
                    g_userMobilityModels[i]->SetTargetPosition(targetPosition);
                    g_userMobilityModels[i]->StartMoving();
                } else {
                    g_userMobilityModels[i]->SetTargetPosition(targetPosition);
                }
                
                anyNewUserMoving = true;
                
                g_userInfoList[i].connectedAP = result.selectedAP;
                DUAL_COUT("  新規ユーザ" << i << " - AP" << result.selectedAP 
                          << "に向かって移動継続中..." << std::endl);
                DUAL_COUT("  目標位置: (" << std::setprecision(1) 
                          << targetPosition.x << "m, " << targetPosition.y << "m)" << std::endl);
            }
        }
    }
    
    // まだ移動中の新規ユーザがいる場合、次回の更新をスケジュール
    if (anyNewUserMoving && Simulator::Now().GetSeconds() < g_simTime - 1.0) {
        Simulator::Schedule(Seconds(1.0), &UpdateUserMovement);
    }
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
    *g_configFile << "Number_of_New_Users=" << g_nNewUsers << std::endl;
    *g_configFile << "Number_of_Existing_Users=" << g_nExistingUsers << std::endl;
    *g_configFile << "Simulation_Time_sec=" << std::fixed << std::setprecision(1) << g_simTime << std::endl;
    *g_configFile << "Simulation_Area=30x30m" << std::endl;
    *g_configFile << "WiFi_Standard=802.11a" << std::endl;
    *g_configFile << "User_Mobility=AP_Directed_Movement_for_New_Users_Only" << std::endl;
    *g_configFile << std::endl;
    
    *g_configFile << "[AP Configuration]" << std::endl;
    for (uint32_t i = 0; i < g_nAPs; ++i) {
        Vector pos = g_apNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        *g_configFile << "AP" << i << "_Position_x=" << std::fixed << std::setprecision(1) << pos.x << std::endl;
        *g_configFile << "AP" << i << "_Position_y=" << pos.y << std::endl;
        *g_configFile << "AP" << i << "_Channel_Utilization=" << std::setprecision(3) << g_apInfoList[i].channelUtilization << std::endl;
    }
    
    g_configFile->flush();
}

// 初期状態の表示（修正版）
void PrintInitialState() {
    DUAL_COUT("\n" << std::string(80, '=') << std::endl);
    DUAL_COUT("       WiFi APセレクションシミュレーション 初期状態" << std::endl);
    DUAL_COUT(std::string(80, '=') << std::endl);

    DUAL_COUT("\n【シミュレーション設定】" << std::endl);
    DUAL_COUT("  AP数: " << g_nAPs << "台" << std::endl);
    DUAL_COUT("  総ユーザ数: " << g_nUsers << "台" << std::endl);
    DUAL_COUT("  新規ユーザ数: " << g_nNewUsers << "台 (移動)" << std::endl);
    DUAL_COUT("  既存ユーザ数: " << g_nExistingUsers << "台 (固定)" << std::endl);
    DUAL_COUT("  シミュレーション時間: " << std::fixed << std::setprecision(0) << g_simTime << "秒" << std::endl);

    DUAL_COUT("\n【AP配置情報】" << std::endl);
    for (uint32_t i = 0; i < g_nAPs; ++i) {
        Vector pos = g_apNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        DUAL_COUT("  AP" << i << ": 位置(" << std::fixed << std::setprecision(1) 
                  << pos.x << "m, " << pos.y << "m)" 
                  << ", チャンネル利用率:" << std::setprecision(1) << (g_apInfoList[i].channelUtilization * 100) << "%" << std::endl);
    }

    DUAL_COUT("\n【新規ユーザ初期配置】" << std::endl);
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (g_userInfoList[i].isNewUser) {
            Vector pos = g_userMobilityModels[i]->GetPosition();
            DUAL_COUT("  新規ユーザ" << i << ": 位置(" 
                      << std::fixed << std::setprecision(1)
                      << pos.x << "m, " << pos.y << "m)"
                      << ", スループット閾値:" << g_userInfoList[i].throughputThreshold << " Mbps" << std::endl);
        }
    }
    
    DUAL_COUT("\n【既存ユーザ配置】" << std::endl);
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (!g_userInfoList[i].isNewUser) {
            Vector pos = g_userInfoList[i].position;
            DUAL_COUT("  既存ユーザ" << i << ": 位置(" 
                      << std::fixed << std::setprecision(1)
                      << pos.x << "m, " << pos.y << "m)"
                      << ", 接続AP: AP" << g_userInfoList[i].connectedAP << std::endl);
        }
    }
    
    // 実験前のシステム全体のスループット
    double initialSystemThroughput = CalculateSystemThroughput();
    DUAL_COUT("\n【実験前システム全体スループット（調和平均）】: " 
              << std::fixed << std::setprecision(2) << initialSystemThroughput << " Mbps" << std::endl);
    
    DUAL_COUT(std::string(80, '=') << std::endl);
}

// ユーザを最適APの近くに移動させる関数（新規ユーザのみ）
void StartUserMovement() {
    DUAL_COUT("\n【新規ユーザ移動開始】" << std::endl);
    DUAL_COUT("  新規ユーザのみがスループット閾値に到達するまで最適APに向かって移動します。" << std::endl);
    DUAL_COUT("  既存ユーザは固定位置に留まります。" << std::endl);
    
    // 初回の移動制御を開始
    UpdateUserMovement();
}

// 定期的な位置と結果の出力（修正版）
void PeriodicOutput() {
    DUAL_COUT("\n【時刻 " << std::fixed << std::setprecision(1) 
              << Simulator::Now().GetSeconds() << "秒 - 定期レポート】" << std::endl);
    
    DUAL_COUT("  --- 新規ユーザ状況 ---" << std::endl);
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (g_userInfoList[i].isNewUser) {
            Vector pos = g_userMobilityModels[i]->GetPosition();
            DUAL_COUT("  新規ユーザ" << i << ": (" << std::setprecision(1) 
                      << pos.x << "m, " << pos.y << "m)"
                      << " - 現在スループット: " << g_userInfoList[i].throughput << "Mbps"
                      << " (" << (g_userInfoList[i].hasReachedThreshold ? "到達済み" : "移動中") << ")" << std::endl);
        }
    }
    
    DUAL_COUT("  --- 既存ユーザ状況 ---" << std::endl);
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (!g_userInfoList[i].isNewUser) {
            Vector pos = g_userInfoList[i].position;
            DUAL_COUT("  既存ユーザ" << i << ": (" << std::setprecision(1) 
                      << pos.x << "m, " << pos.y << "m)"
                      << " - 現在スループット: " << g_userInfoList[i].throughput << "Mbps (固定)" << std::endl);
        }
    }
    
    if (Simulator::Now().GetSeconds() < g_simTime - 3.0) {
        Simulator::Schedule(Seconds(3.0), &PeriodicOutput);
    }
}

int main(int argc, char *argv[]) {
    // パラメータ設定
    uint32_t nAPs = 2;
    uint32_t nNewUsers = 2;      // 新規ユーザ数
    uint32_t nExistingUsers = 3; // 既存ユーザ数
    uint32_t nUsers = nNewUsers + nExistingUsers;
    double simTime = 30.0;

    // グローバル変数設定
    g_nAPs = nAPs;
    g_nUsers = nUsers;
    g_nNewUsers = nNewUsers;
    g_nExistingUsers = nExistingUsers;
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
    std::string resultFileName = g_outputDir + "/AP2User_NewExisting_" + g_timestamp + ".txt";
    std::string configFileName = g_outputDir + "/AP2User_NewExisting_config_" + g_timestamp + ".txt";
    std::string terminalOutputFileName = g_outputDir + "/AP2User_NewExisting_output_" + g_timestamp + ".txt";
    std::string animFileName = g_outputDir + "/AP2User_NewExisting_animation_" + g_timestamp + ".xml";

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
    Ssid ssid = Ssid("enhanced-wifi-network");

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

    // AP配置（2台のAP）
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(15.0, 15.0, 0.0));   // AP0 (中央)
    apPositionAlloc->Add(Vector(5.0, 5.0, 0.0));     // AP1 (左上)

    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // STAノードの設定（新規ユーザと既存ユーザを分ける）
    g_userMobilityModels.resize(nUsers);
    
    // 新規ユーザの設定（移動可能）
    for (uint32_t i = 0; i < nNewUsers; ++i) {
        Ptr<APDirectedMobilityModel> customMobility = CreateObject<APDirectedMobilityModel>();
        
        // 初期位置を設定（APから離れた位置）
        Vector initialPos;
        if (i == 0) {
            initialPos = Vector(25.0, 0.0, 0.0);  // 新規ユーザ0
        } else if (i == 1) {
            initialPos = Vector(0.0, 25.0, 0.0);  // 新規ユーザ1
        }
        
        customMobility->SetPosition(initialPos);
        
        // 移動速度を設定（3 m/s）
        customMobility->SetAttribute("Speed", DoubleValue(3.0));
        customMobility->SetAttribute("MoveInterval", TimeValue(Seconds(0.1)));
        
        // STAノードにカスタム MobilityModelを直接アタッチ
        staNodes.Get(i)->AggregateObject(customMobility);
        g_userMobilityModels[i] = customMobility;
    }
    
    // 既存ユーザの設定（固定位置）
    for (uint32_t i = nNewUsers; i < nUsers; ++i) {
        // 固定位置用のMobilityModel設定
        Ptr<ConstantPositionMobilityModel> constantMobility = CreateObject<ConstantPositionMobilityModel>();
        
        // 固定位置を設定
        Vector fixedPos;
        if (i == 2) {
            fixedPos = Vector(12.0, 8.0, 0.0);   // 既存ユーザ2（AP0寄り）
        } else if (i == 3) {
            fixedPos = Vector(8.0, 12.0, 0.0);   // 既存ユーザ3（AP0寄り）
        } else if (i == 4) {
            fixedPos = Vector(7.0, 7.0, 0.0);    // 既存ユーザ4（AP1寄り）
        }
        
        constantMobility->SetPosition(fixedPos);
        staNodes.Get(i)->AggregateObject(constantMobility);
        g_userMobilityModels[i] = nullptr; // 既存ユーザはカスタムモデル不要
    }

    // APの情報を初期化
    g_apInfoList.resize(nAPs);
    
    // AP0の設定
    g_apInfoList[0].apId = 0;
    g_apInfoList[0].position = apNodes.Get(0)->GetObject<MobilityModel>()->GetPosition();
    g_apInfoList[0].connectedUsers = 0;
    g_apInfoList[0].channelUtilization = 0.2; // 20%
    g_apInfoList[0].channel = 0;
    g_apInfoList[0].totalThroughput = 0.0;

    // AP1の設定
    g_apInfoList[1].apId = 1;
    g_apInfoList[1].position = apNodes.Get(1)->GetObject<MobilityModel>()->GetPosition();
    g_apInfoList[1].connectedUsers = 0;
    g_apInfoList[1].channelUtilization = 0.6; // 60%
    g_apInfoList[1].channel = 1;
    g_apInfoList[1].totalThroughput = 0.0;

    // ユーザ情報の初期化
    g_userInfoList.resize(nUsers);
    
    // 新規ユーザの設定
    for (uint32_t i = 0; i < nNewUsers; ++i) {
        g_userInfoList[i].userId = i;
        g_userInfoList[i].position = g_userMobilityModels[i]->GetPosition();
        g_userInfoList[i].initialPosition = g_userMobilityModels[i]->GetPosition(); // 初期位置を保存
        g_userInfoList[i].connectedAP = 0;
        g_userInfoList[i].throughput = 0.0;
        g_userInfoList[i].throughputThreshold = 60.0; // 60Mbpsの閾値
        g_userInfoList[i].hasReachedThreshold = false;
        g_userInfoList[i].isNewUser = true;  // 新規ユーザ
    }
    
    // 既存ユーザの設定
    for (uint32_t i = nNewUsers; i < nUsers; ++i) {
        g_userInfoList[i].userId = i;
        g_userInfoList[i].position = staNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        g_userInfoList[i].initialPosition = g_userInfoList[i].position; // 初期位置を保存
        // 既存ユーザのAP接続を決定（近いAPに接続）
        double distToAP0 = sqrt(pow(g_userInfoList[i].position.x - g_apInfoList[0].position.x, 2) +
                               pow(g_userInfoList[i].position.y - g_apInfoList[0].position.y, 2));
        double distToAP1 = sqrt(pow(g_userInfoList[i].position.x - g_apInfoList[1].position.x, 2) +
                               pow(g_userInfoList[i].position.y - g_apInfoList[1].position.y, 2));
        g_userInfoList[i].connectedAP = (distToAP0 < distToAP1) ? 0 : 1;
        g_userInfoList[i].throughput = CalculateCurrentThroughput(i, g_userInfoList[i].connectedAP);
        g_userInfoList[i].throughputThreshold = 0.0; // 既存ユーザは閾値なし
        g_userInfoList[i].hasReachedThreshold = true; // 既存ユーザは移動しない
        g_userInfoList[i].isNewUser = false; // 既存ユーザ
    }

    // アルゴリズム初期化
    APSelectionAlgorithm algorithm(25.0, 10.0);
    algorithm.UpdateAPInfo(g_apInfoList);
    g_algorithm = &algorithm;

    // 設定ファイル書き込み
    WriteConfigFile();
    configFile.close();

    resultFile << "Time(s)\tUserID\tUserType\tPosition(x,y)\tSelectedAP\tThroughput(Mbps)\tStatus" << std::endl;

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

    // NetAnim設定（色分け）
    AnimationInterface anim(animFileName);
    anim.UpdateNodeColor(apNodes.Get(0), 0, 255, 0);    // AP0を緑色
    anim.UpdateNodeColor(apNodes.Get(1), 0, 255, 0);  // AP1を緑色

    // ユーザーを色分け
    for (uint32_t i = 0; i < nUsers; ++i) {
        if (g_userInfoList[i].isNewUser) {
            anim.UpdateNodeColor(staNodes.Get(i), 255, 0, 0);   // 新規ユーザを赤色
        } else {
            anim.UpdateNodeColor(staNodes.Get(i), 0, 0, 255);   // 既存ユーザを青色
        }
    }

    // シミュレーション実行
    Simulator::Stop(Seconds(simTime));

    // ユーザー移動開始（3秒後）
    Simulator::Schedule(Seconds(3.0), &StartUserMovement);

    // 定期的な位置と結果の出力（3秒後から3秒間隔）
    Simulator::Schedule(Seconds(3.0), &PeriodicOutput);

    DUAL_COUT("\n【シミュレーション開始】" << std::endl);
    DUAL_COUT("  開始時刻: " << g_timestamp << std::endl);
    DUAL_COUT("  実行時間: " << simTime << "秒" << std::endl);
    
    Simulator::Run();

    // 最終結果の出力
    DUAL_COUT("\n" << std::string(80, '=') << std::endl);
    DUAL_COUT("                シミュレーション最終結果" << std::endl);
    DUAL_COUT(std::string(80, '=') << std::endl);
    
    // 最終位置表示
    DUAL_COUT("\n【最終位置と状態】" << std::endl);
    
    DUAL_COUT("  --- 新規ユーザ ---" << std::endl);
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (g_userInfoList[i].isNewUser) {
            Vector finalPos = g_userMobilityModels[i]->GetPosition();
            uint32_t connectedAP = g_userInfoList[i].connectedAP;
            Vector apPos = g_apNodes->Get(connectedAP)->GetObject<MobilityModel>()->GetPosition();
            double finalDistance = sqrt(pow(finalPos.x - apPos.x, 2) + pow(finalPos.y - apPos.y, 2));
            
            DUAL_COUT("  新規ユーザ" << i << ": (" << std::fixed << std::setprecision(1) 
                      << finalPos.x << "m, " << finalPos.y << "m)" 
                      << ", 接続AP: AP" << connectedAP
                      << ", APまでの距離: " << finalDistance << "m"
                      << ", 最終スループット: " << g_userInfoList[i].throughput << "Mbps"
                      << ", 閾値到達: " << (g_userInfoList[i].hasReachedThreshold ? "はい" : "いいえ") << std::endl);
        }
    }
    
    DUAL_COUT("  --- 既存ユーザ ---" << std::endl);
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (!g_userInfoList[i].isNewUser) {
            Vector finalPos = g_userInfoList[i].position;
            uint32_t connectedAP = g_userInfoList[i].connectedAP;
            Vector apPos = g_apNodes->Get(connectedAP)->GetObject<MobilityModel>()->GetPosition();
            double finalDistance = sqrt(pow(finalPos.x - apPos.x, 2) + pow(finalPos.y - apPos.y, 2));
            
            DUAL_COUT("  既存ユーザ" << i << ": (" << std::fixed << std::setprecision(1) 
                      << finalPos.x << "m, " << finalPos.y << "m)" 
                      << ", 接続AP: AP" << connectedAP
                      << ", APまでの距離: " << finalDistance << "m"
                      << ", 最終スループット: " << g_userInfoList[i].throughput << "Mbps (固定)" << std::endl);
        }
    }

    // 移動ベクトルの表示（新規ユーザのみ）
    DUAL_COUT("\n【新規ユーザ移動ベクトル情報】" << std::endl);
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (g_userInfoList[i].isNewUser) {
            Vector movementVector = CalculateMovementVector(i);
            double movementDistance = sqrt(movementVector.x * movementVector.x + 
                                         movementVector.y * movementVector.y);
            
            DUAL_COUT("  新規ユーザ" << i << " - 移動ベクトル: (" 
                      << std::fixed << std::setprecision(1) 
                      << movementVector.x << "m, " << movementVector.y << "m)"
                      << ", 移動距離: " << movementDistance << "m" << std::endl);
            DUAL_COUT("    初期位置: (" 
                      << g_userInfoList[i].initialPosition.x << "m, " 
                      << g_userInfoList[i].initialPosition.y << "m)"
                      << " → 最終位置: (" 
                      << g_userInfoList[i].position.x << "m, " 
                      << g_userInfoList[i].position.y << "m)" << std::endl);
        }
    }

    // APの最終状態
    DUAL_COUT("\n【APの最終状態】" << std::endl);
    for (uint32_t i = 0; i < g_nAPs; ++i) {
        Vector apPos = g_apNodes->Get(i)->GetObject<MobilityModel>()->GetPosition();
        
        // このAPに接続されているユーザー数を数える
        uint32_t newUsersConnected = 0;
        uint32_t existingUsersConnected = 0;
        for (uint32_t j = 0; j < g_nUsers; ++j) {
            if (g_userInfoList[j].connectedAP == i) {
                if (g_userInfoList[j].isNewUser) {
                    newUsersConnected++;
                } else {
                    existingUsersConnected++;
                }
            }
        }
        
        DUAL_COUT("  AP" << i << ": 位置(" << std::setprecision(1) << apPos.x 
                  << "m, " << apPos.y << "m)"
                  << ", チャンネル利用率: " << std::setprecision(1) << (g_apInfoList[i].channelUtilization * 100) << "%"
                  << ", 接続ユーザー数: " << (newUsersConnected + existingUsersConnected)
                  << " (新規:" << newUsersConnected << ", 既存:" << existingUsersConnected << ")"
                  << ", AP総スループット: " << std::setprecision(2) << g_apInfoList[i].totalThroughput << "Mbps" << std::endl);
    }

    // 実験後のシステム全体のスループット
    double finalSystemThroughput = CalculateSystemThroughput();
    DUAL_COUT("\n【実験後システム全体スループット（調和平均）】: " 
              << std::fixed << std::setprecision(2) << finalSystemThroughput << " Mbps" << std::endl);

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