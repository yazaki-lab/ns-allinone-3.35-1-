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
#include <random>

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

struct APInfo {
    uint32_t apId;
    Vector position;
    uint32_t connectedUsers;
    double channelUtilization;
    std::vector<double> userRates;
    uint32_t channel;
    double totalThroughput; 
};

struct UserInfo {
    uint32_t userId;
    Vector position;
    Vector initialPosition; 
    uint32_t connectedAP;
    double throughput;
    double throughputThreshold;
    bool hasReachedThreshold;
    bool isNewUser;
    double movementDistance;
    Vector targetPosition;  // 目標位置を記録
};

struct APSelectionResult {
    uint32_t userId;
    Vector userPosition;
    uint32_t selectedAP;
    double expectedThroughput;
    double distance;
    double score;
    std::vector<std::pair<uint32_t, double>> allScores;
};

// 改善されたカスタム移動モデル
class APDirectedMobilityModel : public MobilityModel {
private:
    Vector m_currentPosition;
    Vector m_targetPosition;
    double m_speed;
    EventId m_moveEvent;
    Time m_moveInterval;
    bool m_isMoving;
    bool m_targetReached;
    double m_tolerance;  // 目標到達の許容誤差
    
public:
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("APDirectedMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<APDirectedMobilityModel>()
            .AddAttribute("Speed", "移動速度 (m/s)",
                         DoubleValue(3.0),  // 速度を3m/sに設定
                         MakeDoubleAccessor(&APDirectedMobilityModel::m_speed),
                         MakeDoubleChecker<double>())
            .AddAttribute("MoveInterval", "移動更新間隔",
                         TimeValue(Seconds(0.1)),
                         MakeTimeAccessor(&APDirectedMobilityModel::m_moveInterval),
                         MakeTimeChecker())
            .AddAttribute("Tolerance", "目標到達許容誤差 (m)",
                         DoubleValue(0.5),  // 許容誤差を0.5mに設定
                         MakeDoubleAccessor(&APDirectedMobilityModel::m_tolerance),
                         MakeDoubleChecker<double>());
        return tid;
    }
    
    APDirectedMobilityModel() : m_speed(3.0), m_moveInterval(Seconds(0.1)), 
                                m_isMoving(false), m_targetReached(false), m_tolerance(0.5) {}
    
    void SetTargetPosition(const Vector& target) {
        m_targetPosition = target;
        m_targetReached = false;
        // 新しい目標が設定された場合は常に移動を開始
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
    
    bool HasReachedTarget() const { return m_targetReached; }
    bool IsMoving() const { return m_isMoving; }
    double GetDistanceToTarget() const {
        Vector direction = m_targetPosition - m_currentPosition;
        return sqrt(direction.x * direction.x + direction.y * direction.y);
    }
    
private:
    void ScheduleMove() {
        if (!m_isMoving) return;
        m_moveEvent = Simulator::Schedule(m_moveInterval, &APDirectedMobilityModel::DoMove, this);
    }
    
    void DoMove() {
        Vector direction = m_targetPosition - m_currentPosition;
        double distance = sqrt(direction.x * direction.x + direction.y * direction.y);
        
        // 許容誤差内に到達したかチェック
        if (distance <= m_tolerance) {
            SetPosition(m_targetPosition);  // 正確に目標位置に設定
            m_targetReached = true;
            m_isMoving = false;
            return;
        }
        
        // 方向ベクトルを正規化
        direction.x /= distance;
        direction.y /= distance;
        direction.z = 0;
        
        double moveDistance = m_speed * m_moveInterval.GetSeconds();
        
        // オーバーシュートを防ぐ
        if (moveDistance >= distance) {
            SetPosition(m_targetPosition);
            m_targetReached = true;
            m_isMoving = false;
            return;
        }
        
        Vector newPosition = m_currentPosition;
        newPosition.x += direction.x * moveDistance;
        newPosition.y += direction.y * moveDistance;
        
        // 境界チェック（0-125mの範囲内に制限）
        newPosition.x = std::max(0.0, std::min(125.0, newPosition.x));
        newPosition.y = std::max(0.0, std::min(125.0, newPosition.y));
        
        SetPosition(newPosition);
        ScheduleMove();
    }
    
    virtual Vector DoGetPosition() const { return m_currentPosition; }
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
    APSelectionAlgorithm(double dTh = 25.0, double thetaTh = 10.0) 
        : m_dThreshold(dTh), m_thetaThreshold(thetaTh) {
        m_weights = {0.25, 0.25, 0.25, 0.25};
    }
    
    const std::vector<double>& getWeights() const { return m_weights; }
    double getDThreshold() const { return m_dThreshold; }

    void UpdateAPInfo(const std::vector<APInfo>& apList) { m_apList = apList; }

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

    uint32_t SelectOptimalAP(const Vector& userPos) {
        APSelectionResult result = SelectOptimalAPDetailed(userPos, 0);
        return result.selectedAP;
    }
    
    uint32_t SelectNearestAP(const Vector& userPos) {
        double minDistance = 1e9;
        uint32_t nearestAP = 0;
        
        for (const auto& ap : m_apList) {
            double distance = CalculateDistance(userPos, ap.position);
            if (distance < minDistance) {
                minDistance = distance;
                nearestAP = ap.apId;
            }
        }
        
        return nearestAP;
    }
};

// グローバル変数
static std::ofstream* g_outputFile = nullptr;
static NodeContainer* g_apNodes = nullptr;
static NodeContainer* g_staNodes = nullptr;
static uint32_t g_nAPs = 0;
static uint32_t g_nUsers = 0;
static uint32_t g_nNewUsers = 0;
static uint32_t g_nExistingUsers = 0;
static double g_simTime = 0.0;
static double g_movementRadius = 0.0;
static double g_requiredThroughput = 0.0;
static APSelectionAlgorithm* g_algorithm = nullptr;
static std::vector<APInfo> g_apInfoList;
static std::vector<UserInfo> g_userInfoList;
static std::vector<Ptr<APDirectedMobilityModel>> g_userMobilityModels;
static std::string g_sessionDir = "";

// ランダム生成器
static std::mt19937 g_randomEngine;
static std::uniform_real_distribution<double> g_positionDistribution(0.0, 125.0);

void PrintMessage(const std::string& message) {
    std::cout << message;
    if (g_outputFile) {
        *g_outputFile << message;
        g_outputFile->flush();
    }
}

#define OUTPUT(x) do { \
    std::ostringstream oss; \
    oss << x; \
    PrintMessage(oss.str()); \
} while(0)

std::string GetTimestamp() {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
    return std::string(buffer);
}

bool CreateDirectory(const std::string& path) {
#ifdef _WIN32
    return _mkdir(path.c_str()) == 0 || errno == EEXIST;
#else
    return mkdir(path.c_str(), 0777) == 0 || errno == EEXIST;
#endif
}

Vector GenerateRandomPosition() {
    double x = g_positionDistribution(g_randomEngine);
    double y = g_positionDistribution(g_randomEngine);
    return Vector(x, y, 0.0);
}

std::pair<double, double> CalculatePositionVariance() {
    if (g_nUsers == 0) return {0.0, 0.0};
    
    std::vector<Vector> positions;
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Vector pos;
        if (g_userInfoList[i].isNewUser && g_userMobilityModels[i]) {
            pos = g_userMobilityModels[i]->GetPosition();
        } else {
            pos = g_userInfoList[i].position;
        }
        positions.push_back(pos);
    }
    
    double meanX = 0.0, meanY = 0.0;
    for (const auto& pos : positions) {
        meanX += pos.x;
        meanY += pos.y;
    }
    meanX /= g_nUsers;
    meanY /= g_nUsers;
    
    double varX = 0.0, varY = 0.0;
    for (const auto& pos : positions) {
        varX += pow(pos.x - meanX, 2);
        varY += pow(pos.y - meanY, 2);
    }
    varX /= g_nUsers;
    varY /= g_nUsers;
    
    return {varX, varY};
}

void OutputResultsToCSV(double finalSystemThroughput, double initialSystemThroughput) {
    std::string csvDir = "results_csv";
    CreateDirectory(csvDir);
    
    std::string csvFile = csvDir + "/myargo_AP4user16_09231649.csv";
    
    bool fileExists = false;
    std::ifstream checkFile(csvFile);
    if (checkFile.good()) {
        fileExists = true;
    }
    checkFile.close();
    
    std::ofstream csvOutput(csvFile, std::ios::app);
    
    if (!fileExists) {
        csvOutput << "Method,NewUserName,MovementRadius,RequiredThroughput,MovementDistance,ConnectedAP,FinalSystemThroughput,InitialSystemThroughput,ImprovementRate,PositionVarianceX,PositionVarianceY,TotalPositionVariance" << std::endl;
    }
    
    auto variance = CalculatePositionVariance();
    double totalVariance = variance.first + variance.second;
    
    for (uint32_t i = 0; i < g_nNewUsers; ++i) {
        if (g_userInfoList[i].isNewUser) {
            double improvementRate = (initialSystemThroughput > 0.0) ? 
                ((finalSystemThroughput - initialSystemThroughput) / initialSystemThroughput) * 100.0 : 0.0;
            
            csvOutput << std::fixed << std::setprecision(2);
            csvOutput << "myargo," 
                     << "User" << i << ","
                     << g_movementRadius << ","
                     << g_requiredThroughput << ","
                     << g_userInfoList[i].movementDistance << ","
                     << "AP" << g_userInfoList[i].connectedAP << ","
                     << finalSystemThroughput << ","
                     << initialSystemThroughput << ","
                     << improvementRate << ","
                     << variance.first << ","
                     << variance.second << ","
                     << totalVariance << std::endl;
        }
    }
    
    csvOutput.close();
    OUTPUT("CSVファイルに結果を出力しました: " << csvFile << "\n");
}

double CalculateSystemThroughput() {
    if (g_apInfoList.empty()) return 0.0;
    
    for (uint32_t apId = 0; apId < g_nAPs; ++apId) {
        double apTotalThroughput = 0.0;
        int connectedUsers = 0;
        
        for (uint32_t userId = 0; userId < g_nUsers; ++userId) {
            if (g_userInfoList[userId].connectedAP == apId) {
                apTotalThroughput += g_userInfoList[userId].throughput;
                connectedUsers++;
            }
        }
        
        g_apInfoList[apId].totalThroughput = apTotalThroughput;
        g_apInfoList[apId].connectedUsers = connectedUsers;
    }
    
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

double CalculateCurrentThroughput(uint32_t userId, uint32_t apId) {
    if (userId >= g_nUsers || apId >= g_nAPs) return 0.0;
    
    Vector userPos;
    
    if (g_userInfoList[userId].isNewUser && g_userMobilityModels[userId]) {
        userPos = g_userMobilityModels[userId]->GetPosition();
    } else {
        userPos = g_userInfoList[userId].position;
    }
    
    Ptr<MobilityModel> apMobility = g_apNodes->Get(apId)->GetObject<MobilityModel>();
    if (!apMobility) return 0.0;
    
    Vector apPos = apMobility->GetPosition();
    double distance = sqrt(pow(userPos.x - apPos.x, 2) + pow(userPos.y - apPos.y, 2));
    
    double baseThroughput;
    if (distance < 5.0) baseThroughput = 150.0;
    else if (distance < 10.0) baseThroughput = 130.0;
    else if (distance < 15.0) baseThroughput = 100.0;
    else if (distance < 20.0) baseThroughput = 65.0;
    else if (distance < 25.0) baseThroughput = 30.0;
    else baseThroughput = 6.5;
    
    return baseThroughput * (1.0 - g_apInfoList[apId].channelUtilization);
}

// 改善された目標位置計算関数
Vector CalculateOptimalTargetPosition(const Vector& currentPos, const Vector& apPos, uint32_t userId) {
    // 目標距離を4mに設定（5m未満で150Mbpsを得るため）
    double targetDistance = 4.0;
    
    // APとユーザの現在位置の方向ベクトルを計算
    Vector direction = currentPos - apPos;
    double currentDistance = sqrt(direction.x * direction.x + direction.y * direction.y);
    
    Vector targetPosition;
    
    if (currentDistance > 0.1) {
        // 方向ベクトルを正規化
        direction.x /= currentDistance;
        direction.y /= currentDistance;
        
        // AP中心から目標距離の位置を計算
        targetPosition.x = apPos.x + direction.x * targetDistance;
        targetPosition.y = apPos.y + direction.y * targetDistance;
    } else {
        // ほぼ同じ位置にいる場合は、ユーザIDベースの角度でオフセット
        double angle = (userId * 60.0) * M_PI / 180.0; // 60度刻み
        targetPosition.x = apPos.x + targetDistance * cos(angle);
        targetPosition.y = apPos.y + targetDistance * sin(angle);
    }
    
    targetPosition.z = 0.0;
    
    // 境界チェック（0-125mの範囲内に制限）
    targetPosition.x = std::max(0.0, std::min(125.0, targetPosition.x));
    targetPosition.y = std::max(0.0, std::min(125.0, targetPosition.y));
    
    return targetPosition;
}

// 改善されたユーザ移動更新関数
void UpdateUserMovement() {
    if (!g_algorithm || g_userMobilityModels.empty()) return;
    
    bool anyMoving = false;
    double currentTime = Simulator::Now().GetSeconds();
    
    OUTPUT("[時刻 " << std::fixed << std::setprecision(1) << currentTime << "s] ユーザ移動更新\n");
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (!g_userInfoList[i].isNewUser || !g_userMobilityModels[i] || 
            g_userInfoList[i].hasReachedThreshold) continue;
        
        Vector userPos = g_userMobilityModels[i]->GetPosition();
        g_userInfoList[i].position = userPos;
        
        // 移動距離を計算
        Vector initialPos = g_userInfoList[i].initialPosition;
        g_userInfoList[i].movementDistance = sqrt(
            pow(userPos.x - initialPos.x, 2) + pow(userPos.y - initialPos.y, 2));
        
        // 移動許容距離チェック
        if (g_userInfoList[i].movementDistance >= g_movementRadius) {
            OUTPUT("新規ユーザ" << i << ": 移動許容距離に到達 (" 
                   << std::setprecision(2) << g_userInfoList[i].movementDistance 
                   << "m >= " << g_movementRadius << "m), 移動停止\n");
            g_userInfoList[i].hasReachedThreshold = true;
            g_userMobilityModels[i]->StopMoving();
            continue;
        }
        
        // AP選択
        APSelectionResult result = g_algorithm->SelectOptimalAPDetailed(userPos, i);
        double currentThroughput = CalculateCurrentThroughput(i, result.selectedAP);
        g_userInfoList[i].throughput = currentThroughput;
        
        // スループット閾値チェック
        if (currentThroughput >= g_userInfoList[i].throughputThreshold) {
            g_userInfoList[i].hasReachedThreshold = true;
            g_userMobilityModels[i]->StopMoving();
            
            OUTPUT("新規ユーザ" << i << ": 要求スループット達成 位置(" 
                   << std::fixed << std::setprecision(1) << userPos.x << ", " << userPos.y 
                   << ") -> AP" << result.selectedAP << " 接続, スループット: " 
                   << std::setprecision(2) << currentThroughput << "Mbps\n");
        } else {
            // 目標位置を設定して移動継続
            Ptr<MobilityModel> targetAPMobility = g_apNodes->Get(result.selectedAP)->GetObject<MobilityModel>();
            if (targetAPMobility) {
                Vector apPos = targetAPMobility->GetPosition();
                Vector targetPosition = CalculateOptimalTargetPosition(userPos, apPos, i);
                
                // 目標位置が変更された場合のみ更新
                Vector currentTarget = g_userInfoList[i].targetPosition;
                double targetDistance = sqrt(pow(targetPosition.x - currentTarget.x, 2) + 
                                           pow(targetPosition.y - currentTarget.y, 2));
                
                if (targetDistance > 1.0) {  // 1m以上の変更がある場合のみ更新
                    g_userMobilityModels[i]->SetTargetPosition(targetPosition);
                    g_userInfoList[i].targetPosition = targetPosition;
                    g_userInfoList[i].connectedAP = result.selectedAP;
                    
                    OUTPUT("新規ユーザ" << i << ": 新目標設定 現在位置(" 
                           << std::setprecision(1) << userPos.x << ", " << userPos.y 
                           << ") -> 目標位置(" << targetPosition.x << ", " << targetPosition.y 
                           << ") AP" << result.selectedAP << " スループット: " 
                           << std::setprecision(2) << currentThroughput << "Mbps\n");
                }
                
                anyMoving = true;
            }
        }
    }
    
    // 移動中のユーザがいて、シミュレーション時間内であれば継続
    if (anyMoving && currentTime < g_simTime - 1.0) {
        Simulator::Schedule(Seconds(1.0), &UpdateUserMovement);
    } else {
        OUTPUT("全ての新規ユーザの移動が完了しました。\n");
    }
}

void PrintInitialState() {
    OUTPUT("\n=== WiFi APセレクションシミュレーション ===\n");
    OUTPUT("AP数: " << g_nAPs << ", ユーザ数: " << g_nUsers);
    OUTPUT(" (新規:" << g_nNewUsers << ", 既存:" << g_nExistingUsers << ")\n");
    OUTPUT("シミュレーション時間: " << g_simTime << "秒\n");
    OUTPUT("移動許容距離: " << g_movementRadius << "m\n");
    OUTPUT("要求スループット: " << g_requiredThroughput << "Mbps\n");
    OUTPUT("環境サイズ: 125m×125m\n");
    
    for (uint32_t i = 0; i < g_nNewUsers; ++i) {
        Vector pos = g_userInfoList[i].initialPosition;
        OUTPUT("新規ユーザ" << i << " 初期位置: (" << std::fixed << std::setprecision(1) 
               << pos.x << ", " << pos.y << ") -> 最寄りAP" << g_userInfoList[i].connectedAP << " 接続\n");
    }
    
    for (uint32_t i = g_nNewUsers; i < g_nUsers; ++i) {
        Vector pos = g_userInfoList[i].position;
        OUTPUT("既存ユーザ" << i << " 位置: (" << std::fixed << std::setprecision(1) 
               << pos.x << ", " << pos.y << ") -> 最寄りAP" << g_userInfoList[i].connectedAP << " 接続\n");
    }
    
    double initialSystemThroughput = CalculateSystemThroughput();
    OUTPUT("実験前システム全体スループット（調和平均）: " 
           << std::fixed << std::setprecision(2) << initialSystemThroughput << " Mbps\n");
    
    auto initialVariance = CalculatePositionVariance();
    OUTPUT("初期状態の位置分散 - X軸: " << std::setprecision(2) << initialVariance.first 
           << ", Y軸: " << initialVariance.second 
           << ", 合計: " << initialVariance.first + initialVariance.second << "\n");
}

void StartUserMovement() {
    OUTPUT("\n新規ユーザの移動を開始します。\n");
    UpdateUserMovement();
}

void PrintFinalResults() {
    OUTPUT("\n=== シミュレーション完了 ===\n");
    OUTPUT("結果ディレクトリ: " << g_sessionDir << "\n");
    OUTPUT("移動許容距離: " << g_movementRadius << "m\n");
    OUTPUT("要求スループット: " << g_requiredThroughput << "Mbps\n");
    OUTPUT("環境サイズ: 125m×125m\n");
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (g_userInfoList[i].isNewUser) {
            Vector pos = g_userMobilityModels[i]->GetPosition();
            OUTPUT("新規ユーザ" << i << ": 最終位置(" << std::fixed << std::setprecision(1) 
                   << pos.x << ", " << pos.y << "), 移動距離: " 
                   << std::setprecision(2) << g_userInfoList[i].movementDistance 
                   << "m, 接続AP: " << g_userInfoList[i].connectedAP
                   << ", スループット: " << std::setprecision(2) 
                   << g_userInfoList[i].throughput << "Mbps\n");
        }
    }
    
    auto finalVariance = CalculatePositionVariance();
    OUTPUT("最終状態の位置分散 - X軸: " << std::fixed << std::setprecision(2) << finalVariance.first 
           << ", Y軸: " << finalVariance.second 
           << ", 合計: " << finalVariance.first + finalVariance.second << "\n");
    
    double finalSystemThroughput = CalculateSystemThroughput();
    
    // 初期スループット計算
    for (uint32_t apId = 0; apId < g_nAPs; ++apId) {
        g_apInfoList[apId].totalThroughput = 0.0;
        g_apInfoList[apId].connectedUsers = 0;
    }
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Vector initialPos = g_userInfoList[i].initialPosition;
        uint32_t initialConnectedAP;
        
        if (g_userInfoList[i].isNewUser) {
            APSelectionResult initialResult = g_algorithm->SelectOptimalAPDetailed(initialPos, i);
            initialConnectedAP = initialResult.selectedAP;
        } else {
            initialConnectedAP = g_userInfoList[i].connectedAP;
        }
        
        Vector apPos = g_apNodes->Get(initialConnectedAP)->GetObject<MobilityModel>()->GetPosition();
        double distance = sqrt(pow(initialPos.x - apPos.x, 2) + pow(initialPos.y - apPos.y, 2));
        
        double baseThroughput;
        if (distance < 5.0) baseThroughput = 150.0;
        else if (distance < 10.0) baseThroughput = 130.0;
        else if (distance < 15.0) baseThroughput = 100.0;
        else if (distance < 20.0) baseThroughput = 65.0;
        else if (distance < 25.0) baseThroughput = 30.0;
        else baseThroughput = 6.5;
        
        double channelEffect = 1.0 - g_apInfoList[initialConnectedAP].channelUtilization;
        double initialThroughput = baseThroughput * channelEffect;
        
        g_apInfoList[initialConnectedAP].totalThroughput += initialThroughput;
        g_apInfoList[initialConnectedAP].connectedUsers++;
    }
    
    double harmonicSum = 0.0;
    int activeAPs = 0;
    
    for (const auto& ap : g_apInfoList) {
        if (ap.totalThroughput > 0.0) {
            harmonicSum += 1.0 / ap.totalThroughput;
            activeAPs++;
        }
    }
    
    double initialSystemThroughput = 0.0;
    if (harmonicSum > 0.0 && activeAPs > 0) {
        initialSystemThroughput = activeAPs / harmonicSum;
    }

    double improvement = finalSystemThroughput - initialSystemThroughput;
    double improvementPercent = (initialSystemThroughput > 0.0) ? 
        (improvement / initialSystemThroughput) * 100.0 : 0.0;
    
    OUTPUT("システム全体スループット（調和平均）: " 
           << std::fixed << std::setprecision(2) << finalSystemThroughput << " Mbps\n");
    OUTPUT("初期システムスループット: " << std::setprecision(2) << initialSystemThroughput << " Mbps\n");
    OUTPUT("スループット改善量: " << std::setprecision(2) << improvement << " Mbps\n");
    OUTPUT("改善率: " << std::setprecision(1) << improvementPercent << "%\n");
    
    OutputResultsToCSV(finalSystemThroughput, initialSystemThroughput);
}

int main(int argc, char *argv[]) {
    uint32_t nAPs = 4;
    uint32_t nNewUsers = 4;
    uint32_t nExistingUsers = 12;
    uint32_t nUsers = nNewUsers + nExistingUsers;
    double simTime = 30.0;
    double movementRadius = 15.0;
    double requiredThroughput = 30.0;

    CommandLine cmd;
    cmd.AddValue("movementRadius", "移動許容距離 (m)", movementRadius);
    cmd.AddValue("requiredThroughput", "要求スループット (Mbps)", requiredThroughput);
    cmd.AddValue("simTime", "シミュレーション時間 (秒)", simTime);
    cmd.Parse(argc, argv);

    g_nAPs = nAPs;
    g_nUsers = nUsers;
    g_nNewUsers = nNewUsers;
    g_nExistingUsers = nExistingUsers;
    g_simTime = simTime;
    g_movementRadius = movementRadius;
    g_requiredThroughput = requiredThroughput;

    g_randomEngine.seed(static_cast<unsigned int>(time(nullptr)));

    std::string outputDir = "results";
    CreateDirectory(outputDir);
    g_sessionDir = outputDir + "/AP4User16_myargo_" + GetTimestamp();
    CreateDirectory(g_sessionDir);

    std::ofstream outputFile(g_sessionDir + "/output.txt");
    g_outputFile = &outputFile;

    NodeContainer apNodes, staNodes;
    apNodes.Create(nAPs);
    staNodes.Create(nUsers);
    g_apNodes = &apNodes;
    g_staNodes = &staNodes;

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax_2_4GHZ);
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    Ssid ssid = Ssid("wifi-network");

    NetDeviceContainer apDevices, staDevices;
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    apDevices = wifi.Install(phy, mac, apNodes);
    
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
    staDevices = wifi.Install(phy, mac, staNodes);

    MobilityHelper mobility;
    
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(31.25, 31.25, 0.0));
    apPositionAlloc->Add(Vector(93.75, 31.25, 0.0));
    apPositionAlloc->Add(Vector(31.25, 93.75, 0.0));
    apPositionAlloc->Add(Vector(93.75, 93.75, 0.0));
    
    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    g_userMobilityModels.resize(nUsers);
    
    for (uint32_t i = 0; i < nNewUsers; ++i) {
        Ptr<APDirectedMobilityModel> customMobility = CreateObject<APDirectedMobilityModel>();
        Vector initialPos = GenerateRandomPosition();
        
        customMobility->SetPosition(initialPos);
        customMobility->SetAttribute("Speed", DoubleValue(3.0));
        customMobility->SetAttribute("Tolerance", DoubleValue(0.5));
        staNodes.Get(i)->AggregateObject(customMobility);
        g_userMobilityModels[i] = customMobility;
    }
    
    for (uint32_t i = nNewUsers; i < nUsers; ++i) {
        Ptr<ConstantPositionMobilityModel> constantMobility = CreateObject<ConstantPositionMobilityModel>();
        Vector fixedPos;
        
        uint32_t userIndex = i - nNewUsers;
        uint32_t row = userIndex / 4;
        uint32_t col = userIndex % 4;
        
        double x = 15.625 + col * 31.25;
        double y = 20.83 + row * 41.67;
        
        fixedPos = Vector(x, y, 0.0);
        
        constantMobility->SetPosition(fixedPos);
        staNodes.Get(i)->AggregateObject(constantMobility);
        g_userMobilityModels[i] = nullptr;
    }

    g_apInfoList.resize(nAPs);
    
    for (uint32_t apId = 0; apId < nAPs; ++apId) {
        g_apInfoList[apId].apId = apId;
        g_apInfoList[apId].connectedUsers = 0;
        g_apInfoList[apId].channel = apId;
        g_apInfoList[apId].totalThroughput = 0.0;
        
        switch (apId) {
            case 0: 
                g_apInfoList[apId].position = Vector(31.25, 31.25, 0.0);
                g_apInfoList[apId].channelUtilization = 0.2; 
                break;
            case 1: 
                g_apInfoList[apId].position = Vector(93.75, 31.25, 0.0);
                g_apInfoList[apId].channelUtilization = 0.3; 
                break;
            case 2: 
                g_apInfoList[apId].position = Vector(31.25, 93.75, 0.0);
                g_apInfoList[apId].channelUtilization = 0.4; 
                break;
            case 3: 
                g_apInfoList[apId].position = Vector(93.75, 93.75, 0.0);
                g_apInfoList[apId].channelUtilization = 0.5; 
                break;
        }
    }

    APSelectionAlgorithm algorithm(movementRadius, 10.0);
    algorithm.UpdateAPInfo(g_apInfoList);
    g_algorithm = &algorithm;

    g_userInfoList.resize(nUsers);
    
    for (uint32_t i = 0; i < nNewUsers; ++i) {
        g_userInfoList[i].userId = i;
        g_userInfoList[i].position = g_userMobilityModels[i]->GetPosition();
        g_userInfoList[i].initialPosition = g_userMobilityModels[i]->GetPosition();
        
        g_userInfoList[i].connectedAP = g_algorithm->SelectNearestAP(g_userInfoList[i].initialPosition);
        g_userInfoList[i].targetPosition = g_userInfoList[i].initialPosition;  // 目標位置を初期化
        
        g_userInfoList[i].throughput = CalculateCurrentThroughput(i, g_userInfoList[i].connectedAP);
        g_userInfoList[i].throughputThreshold = requiredThroughput;
        g_userInfoList[i].hasReachedThreshold = false;
        g_userInfoList[i].isNewUser = true;
        g_userInfoList[i].movementDistance = 0.0;
    }
    
    for (uint32_t i = nNewUsers; i < nUsers; ++i) {
        g_userInfoList[i].userId = i;
        g_userInfoList[i].position = staNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        g_userInfoList[i].initialPosition = g_userInfoList[i].position;
        g_userInfoList[i].targetPosition = g_userInfoList[i].position;
        
        g_userInfoList[i].connectedAP = g_algorithm->SelectNearestAP(g_userInfoList[i].position);
        
        g_userInfoList[i].throughput = CalculateCurrentThroughput(i, g_userInfoList[i].connectedAP);
        g_userInfoList[i].throughputThreshold = 0.0;
        g_userInfoList[i].hasReachedThreshold = true;
        g_userInfoList[i].isNewUser = false;
        g_userInfoList[i].movementDistance = 0.0;
    }

    PrintInitialState();

    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

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

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    AnimationInterface anim(g_sessionDir + "/animation.xml");
    for (uint32_t i = 0; i < nAPs; ++i) {
        anim.UpdateNodeColor(apNodes.Get(i), 0, 255, 0);
    }
    for (uint32_t i = 0; i < nUsers; ++i) {
        anim.UpdateNodeColor(staNodes.Get(i), 
                           g_userInfoList[i].isNewUser ? 255 : 0, 0, 
                           g_userInfoList[i].isNewUser ? 0 : 255);
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Schedule(Seconds(3.0), &StartUserMovement);
    
    Simulator::Run();
    PrintFinalResults();
    
    outputFile.close();
    Simulator::Destroy();
    return 0;
}