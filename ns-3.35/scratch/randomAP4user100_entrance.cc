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

// 構造体定義
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
    Vector targetPosition;
    bool isSpawned;  // 出現済みかどうか
};

// 軽量化されたランダム移動モビリティモデル
class LightweightMobilityModel : public MobilityModel {
private:
    Vector m_currentPosition;
    Vector m_targetPosition;
    bool m_targetReached;
    bool m_hasStartedMovement;
    
public:
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("LightweightMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<LightweightMobilityModel>();
        return tid;
    }
    
    LightweightMobilityModel() : m_targetReached(false), m_hasStartedMovement(false) {}
    
    void SetRandomTarget(double maxDistance, double environmentSize) {
        if (m_hasStartedMovement) return;
        
        static std::mt19937 rng(static_cast<unsigned int>(time(nullptr)) + reinterpret_cast<uintptr_t>(this));
        std::uniform_real_distribution<double> distanceDist(0.0, maxDistance);
        std::uniform_real_distribution<double> angleDist(0.0, 2 * M_PI);
        
        double randomDistance = distanceDist(rng);
        double randomAngle = angleDist(rng);
        
        Vector targetPos;
        targetPos.x = m_currentPosition.x + randomDistance * cos(randomAngle);
        targetPos.y = m_currentPosition.y + randomDistance * sin(randomAngle);
        targetPos.z = 0.0;
        
        targetPos.x = std::max(0.0, std::min(environmentSize, targetPos.x));
        targetPos.y = std::max(0.0, std::min(environmentSize, targetPos.y));
        
        m_targetPosition = targetPos;
        m_hasStartedMovement = true;
        
        // 即座に目標位置に移動（連続的な移動の代わりに）
        SetPosition(m_targetPosition);
        m_targetReached = true;
    }
    
    bool HasReachedTarget() const { return m_targetReached; }
    bool HasStartedMovement() const { return m_hasStartedMovement; }
    
private:
    virtual Vector DoGetPosition() const { return m_currentPosition; }
    virtual void DoSetPosition(const Vector& position) {
        m_currentPosition = position;
        NotifyCourseChange();
    }
    virtual Vector DoGetVelocity() const {
        return Vector(0, 0, 0);
    }
};

// 軽量化されたAP選択アルゴリズム     
class LightweightAPSelection {
private:
    std::vector<APInfo> m_apList;
    double m_dThreshold;

public:
    LightweightAPSelection(double dTh = 25.0) : m_dThreshold(dTh) {}

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

    // 公平なスループットを調和平均で計算
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

    uint32_t SelectOptimalAP(const Vector& userPos) {
        uint32_t bestAP = 0;
        double bestScore = -1.0;

        for (const auto& ap : m_apList) {
            double distance = CalculateDistance(userPos, ap.position);
            
            // 距離制限チェック
            if (distance > m_dThreshold) continue;
            
            double transmissionRate = CalculateTransmissionRate(userPos, ap.position);
            double throughput = CalculateThroughput(ap, transmissionRate);
            
            // シンプルなスコア計算（スループット重視）
            double score = throughput / (1.0 + distance * 0.1);
            
            if (score > bestScore) {
                bestScore = score;
                bestAP = ap.apId;
            }
        }

        return bestAP;
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
static LightweightAPSelection* g_algorithm = nullptr;
static std::vector<APInfo> g_apInfoList;
static std::vector<UserInfo> g_userInfoList;
static std::vector<Ptr<LightweightMobilityModel>> g_userMobilityModels;
static std::string g_sessionDir = "";
static std::mt19937 g_randomEngine;
static std::uniform_real_distribution<double> g_positionDistribution(0.0, 50.0);
static uint32_t g_spawnedUserCount = 0;

// 入り口位置の定義
static const Vector g_doorPositions[2] = {
    Vector(5.0, 25.0, 0.0),   // 入り口1（左側）
    Vector(45.0, 25.0, 0.0)   // 入り口2（右側）
};
static const double g_doorRadius = 3.0;  // 入り口付近の半径

// 出力関数
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

// タイムスタンプとディレクトリ作成
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

// 入り口付近のランダム位置生成
Vector GenerateRandomPositionNearDoor() {
    static std::uniform_int_distribution<int> doorDist(0, 1);
    static std::uniform_real_distribution<double> radiusDist(0.0, g_doorRadius);
    static std::uniform_real_distribution<double> angleDist(0.0, 2 * M_PI);
    
    // ランダムに入り口を選択
    int doorIndex = doorDist(g_randomEngine);
    Vector doorPos = g_doorPositions[doorIndex];
    
    // 選択された入り口の周辺でランダムに位置を生成
    double radius = radiusDist(g_randomEngine);
    double angle = angleDist(g_randomEngine);
    
    double x = doorPos.x + radius * cos(angle);
    double y = doorPos.y + radius * sin(angle);
    
    // 環境の範囲内に制限
    x = std::max(0.0, std::min(50.0, x));
    y = std::max(0.0, std::min(50.0, y));
    
    return Vector(x, y, 0.0);
}

// 位置分散計算
std::pair<double, double> CalculatePositionVariance() {
    if (g_nUsers == 0) return {0.0, 0.0};
    
    std::vector<Vector> positions;
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (!g_userInfoList[i].isSpawned) continue;
        
        Vector pos;
        if (g_userInfoList[i].isNewUser && g_userMobilityModels[i]) {
            pos = g_userMobilityModels[i]->GetPosition();
        } else {
            pos = g_userInfoList[i].position;
        }
        positions.push_back(pos);
    }
    
    if (positions.empty()) return {0.0, 0.0};
    
    double meanX = 0.0, meanY = 0.0;
    for (const auto& pos : positions) {
        meanX += pos.x;
        meanY += pos.y;
    }
    meanX /= positions.size();
    meanY /= positions.size();
    
    double varX = 0.0, varY = 0.0;
    for (const auto& pos : positions) {
        varX += pow(pos.x - meanX, 2);
        varY += pow(pos.y - meanY, 2);
    }
    varX /= positions.size();
    varY /= positions.size();
    
    return {varX, varY};
}

// CSVファイル出力
void OutputResultsToCSV(double finalSystemThroughput, double initialSystemThroughput) {
    std::string csvDir = "results_csv";
    CreateDirectory(csvDir);
    
    std::string csvFile = csvDir + "/random_AP4user100_door.csv";
    
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
    
    // 各新規ユーザについて詳細な情報を出力
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (g_userInfoList[i].isNewUser && g_userInfoList[i].isSpawned) {
            double improvementRate = (initialSystemThroughput > 0.0) ? 
                ((finalSystemThroughput - initialSystemThroughput) / initialSystemThroughput) * 100.0 : 0.0;
            
            csvOutput << std::fixed << std::setprecision(2);
            csvOutput << "random," 
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

// システム全体スループット計算（調和平均）
double CalculateSystemThroughput() {
    if (g_apInfoList.empty()) return 0.0;
    
    // 各APの総スループットを計算
    for (uint32_t apId = 0; apId < g_nAPs; ++apId) {
        double apTotalThroughput = 0.0;
        int connectedUsers = 0;
        
        for (uint32_t userId = 0; userId < g_nUsers; ++userId) {
            if (g_userInfoList[userId].isSpawned && 
                g_userInfoList[userId].connectedAP == apId) {
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

// スループット計算
double CalculateCurrentThroughput(uint32_t userId, uint32_t apId) {
    if (userId >= g_nUsers || apId >= g_nAPs) return 0.0;
    if (!g_userInfoList[userId].isSpawned) return 0.0;
    
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

// ユーザ移動更新
void UpdateUserMovement() {
    if (!g_algorithm || g_userMobilityModels.empty()) return;
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (!g_userInfoList[i].isNewUser || !g_userMobilityModels[i]) continue;
        if (!g_userInfoList[i].isSpawned) continue;
        
        // ランダム移動をまだ開始していない場合は開始
        if (!g_userMobilityModels[i]->HasStartedMovement()) {
            g_userMobilityModels[i]->SetRandomTarget(g_movementRadius, 50.0);
            
            Vector userPos = g_userMobilityModels[i]->GetPosition();
            g_userInfoList[i].position = userPos;
            
            // 移動距離を計算
            Vector initialPos = g_userInfoList[i].initialPosition;
            g_userInfoList[i].movementDistance = sqrt(
                pow(userPos.x - initialPos.x, 2) + pow(userPos.y - initialPos.y, 2));
            
            // AP選択
            uint32_t selectedAP = g_algorithm->SelectOptimalAP(userPos);
            g_userInfoList[i].connectedAP = selectedAP;
            double currentThroughput = CalculateCurrentThroughput(i, selectedAP);
            g_userInfoList[i].throughput = currentThroughput;
            g_userInfoList[i].hasReachedThreshold = true;
            
            OUTPUT("新規ユーザ" << i << ": ランダム移動完了 最終位置(" 
                   << std::fixed << std::setprecision(1) << userPos.x << ", " << userPos.y 
                   << "), 移動距離: " << std::setprecision(2) << g_userInfoList[i].movementDistance 
                   << "m -> AP" << selectedAP << " 接続, スループット: " 
                   << std::setprecision(2) << currentThroughput << "Mbps\n");
        }
    }
}

// 新規ユーザを1人出現させる
void SpawnNewUser() {
    if (g_spawnedUserCount >= g_nNewUsers) return;
    
    uint32_t userId = g_spawnedUserCount;
    
    // 入り口付近の位置を生成
    Vector spawnPos = GenerateRandomPositionNearDoor();
    
    g_userMobilityModels[userId]->SetPosition(spawnPos);
    g_userInfoList[userId].position = spawnPos;
    g_userInfoList[userId].initialPosition = spawnPos;
    g_userInfoList[userId].isSpawned = true;
    
    // 最寄りのAPに接続
    g_userInfoList[userId].connectedAP = g_algorithm->SelectNearestAP(spawnPos);
    g_userInfoList[userId].throughput = CalculateCurrentThroughput(userId, g_userInfoList[userId].connectedAP);
    
    OUTPUT("新規ユーザ" << userId << " が入り口付近(" 
           << std::fixed << std::setprecision(1) << spawnPos.x << ", " << spawnPos.y 
           << ")に出現 -> AP" << g_userInfoList[userId].connectedAP << " 接続\n");
    
    g_spawnedUserCount++;
    
    // 次のユーザの出現をスケジュール
    if (g_spawnedUserCount < g_nNewUsers) {
        Simulator::Schedule(Seconds(1.0), &SpawnNewUser);
    } else {
        // 全ユーザ出現後、移動を開始
        OUTPUT("\n全新規ユーザの出現完了。ランダム移動を開始します。\n");
        Simulator::Schedule(Seconds(1.0), &UpdateUserMovement);
    }
}

// 初期状態表示
void PrintInitialState() {
    OUTPUT("\n=== WiFi APセレクションシミュレーション ===\n");
    OUTPUT("AP数: " << g_nAPs << ", ユーザ数: " << g_nUsers);
    OUTPUT(" (新規:" << g_nNewUsers << ", 既存:" << g_nExistingUsers << ")\n");
    OUTPUT("シミュレーション時間: " << g_simTime << "秒\n");
    OUTPUT("移動許容距離: " << g_movementRadius << "m\n");
    OUTPUT("要求スループット: " << g_requiredThroughput << "Mbps\n");
    OUTPUT("環境サイズ: 50m×50m\n");
    OUTPUT("入り口1: (" << g_doorPositions[0].x << ", " << g_doorPositions[0].y << ")\n");
    OUTPUT("入り口2: (" << g_doorPositions[1].x << ", " << g_doorPositions[1].y << ")\n");
    OUTPUT("入り口半径: " << g_doorRadius << "m\n\n");
    
    for (uint32_t i = g_nNewUsers; i < g_nUsers; ++i) {
        Vector pos = g_userInfoList[i].position;
        OUTPUT("既存ユーザ" << i << " 位置: (" << std::fixed << std::setprecision(1) 
               << pos.x << ", " << pos.y << ") -> 最寄りAP" << g_userInfoList[i].connectedAP << " 接続\n");
    }
    
    double initialSystemThroughput = CalculateSystemThroughput();
    OUTPUT("実験前システム全体スループット(調和平均): " 
           << std::fixed << std::setprecision(2) << initialSystemThroughput << " Mbps\n");
    
    auto initialVariance = CalculatePositionVariance();
    OUTPUT("初期状態の位置分散 - X軸: " << std::setprecision(2) << initialVariance.first 
           << ", Y軸: " << initialVariance.second 
           << ", 合計: " << initialVariance.first + initialVariance.second << "\n");
}

void StartUserSpawning() {
    OUTPUT("\n新規ユーザの順次出現を開始します（1秒ごとに1人）。\n");
    SpawnNewUser();
}

// 最終結果表示
void PrintFinalResults() {
    OUTPUT("\n=== シミュレーション完了 ===\n");
    OUTPUT("結果ディレクトリ: " << g_sessionDir << "\n");
    OUTPUT("移動許容距離: " << g_movementRadius << "m\n");
    OUTPUT("要求スループット: " << g_requiredThroughput << "Mbps\n");
    OUTPUT("環境サイズ: 50m×50m\n");
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (g_userInfoList[i].isNewUser && g_userInfoList[i].isSpawned) {
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
        if (!g_userInfoList[i].isSpawned) continue;
        
        Vector initialPos = g_userInfoList[i].initialPosition;
        uint32_t initialConnectedAP;
        
        if (g_userInfoList[i].isNewUser) {
            initialConnectedAP = g_algorithm->SelectNearestAP(initialPos);
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
    
    OUTPUT("システム全体スループット(調和平均): " 
           << std::fixed << std::setprecision(2) << finalSystemThroughput << " Mbps\n");
    OUTPUT("初期システムスループット: " << std::setprecision(2) << initialSystemThroughput << " Mbps\n");
    OUTPUT("スループット改善量: " << std::setprecision(2) << improvement << " Mbps\n");
    OUTPUT("改善率: " << std::setprecision(1) << improvementPercent << "%\n");
    
    OutputResultsToCSV(finalSystemThroughput, initialSystemThroughput);
}

int main(int argc, char *argv[]) {
    // パラメータ設定（50m×50m環境用）
    uint32_t nAPs = 4;
    uint32_t nNewUsers = 10;
    uint32_t nExistingUsers = 100;
    uint32_t nUsers = nNewUsers + nExistingUsers;
    double simTime = 50.0;  // 新規ユーザ出現と移動のための時間を追加
    double movementRadius = 14.95;
    double requiredThroughput = 30.0;

    // コマンドライン引数の処理
    CommandLine cmd;
    cmd.AddValue("movementRadius", "移動許容距離 (m)", movementRadius);
    cmd.AddValue("requiredThroughput", "要求スループット (Mbps)", requiredThroughput);
    cmd.AddValue("simTime", "シミュレーション時間 (秒)", simTime);
    cmd.Parse(argc, argv);
    
    // グローバル変数設定
    g_nAPs = nAPs;
    g_nUsers = nUsers;
    g_nNewUsers = nNewUsers;
    g_nExistingUsers = nExistingUsers;
    g_simTime = simTime;
    g_movementRadius = movementRadius;
    g_requiredThroughput = requiredThroughput;

    // ランダム生成器の初期化
    g_randomEngine.seed(static_cast<unsigned int>(time(nullptr)));

    // 出力ディレクトリ作成
    std::string outputDir = "results";
    CreateDirectory(outputDir);
    g_sessionDir = outputDir + "/AP4User100_door_" + GetTimestamp();
    CreateDirectory(g_sessionDir);

    std::ofstream outputFile(g_sessionDir + "/output.txt");
    g_outputFile = &outputFile;

    // ノード作成
    NodeContainer apNodes, staNodes;
    apNodes.Create(nAPs);
    staNodes.Create(nUsers);
    g_apNodes = &apNodes;
    g_staNodes = &staNodes;

    // WiFi設定
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

    // 移動端末設定
    MobilityHelper mobility;
    
    // AP配置（50m×50m環境用に調整）
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(12.5, 12.5, 0.0));  // 左下
    apPositionAlloc->Add(Vector(37.5, 12.5, 0.0));  // 右下
    apPositionAlloc->Add(Vector(12.5, 37.5, 0.0));  // 左上
    apPositionAlloc->Add(Vector(37.5, 37.5, 0.0));  // 右上
    
    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // ユーザ配置
    g_userMobilityModels.resize(nUsers);
    
    // 新規ユーザの設定（最初は未出現状態）
    for (uint32_t i = 0; i < nNewUsers; ++i) {
        Ptr<LightweightMobilityModel> lightweightMobility = CreateObject<LightweightMobilityModel>();
        
        // 仮の初期位置（実際の出現位置は後で設定）
        Vector tempPos = Vector(0.0, 0.0, 0.0);
        lightweightMobility->SetPosition(tempPos);
        staNodes.Get(i)->AggregateObject(lightweightMobility);
        g_userMobilityModels[i] = lightweightMobility;
    }
    
    // 既存ユーザの設定（50m×50m環境用グリッド配置）
    for (uint32_t i = nNewUsers; i < nUsers; ++i) {
        Ptr<ConstantPositionMobilityModel> constantMobility = CreateObject<ConstantPositionMobilityModel>();
        Vector fixedPos;
        
        uint32_t userIndex = i - nNewUsers;
        uint32_t row = userIndex / 10;  // 10列
        uint32_t col = userIndex % 10;
        
        // 50m×50mの環境に100ユーザを配置（10行×10列）
        double x = 2.5 + col * 5.0;      // 2.5mから47.5mまで5m間隔
        double y = 2.5 + row * 5.0;      // 2.5mから47.5mまで5m間隔
        
        fixedPos = Vector(x, y, 0.0);
        
        constantMobility->SetPosition(fixedPos);
        staNodes.Get(i)->AggregateObject(constantMobility);
        g_userMobilityModels[i] = nullptr;
    }

    // AP情報を初期化（50m×50m環境用）
    g_apInfoList.resize(nAPs);
    
    for (uint32_t apId = 0; apId < nAPs; ++apId) {
        g_apInfoList[apId].apId = apId;
        g_apInfoList[apId].connectedUsers = 0;
        g_apInfoList[apId].channel = apId;
        g_apInfoList[apId].totalThroughput = 0.0;
        
        switch (apId) {
            case 0: 
                g_apInfoList[apId].position = Vector(12.5, 12.5, 0.0);
                g_apInfoList[apId].channelUtilization = 0.2; 
                break;
            case 1: 
                g_apInfoList[apId].position = Vector(37.5, 12.5, 0.0);
                g_apInfoList[apId].channelUtilization = 0.3; 
                break;
            case 2: 
                g_apInfoList[apId].position = Vector(12.5, 37.5, 0.0);
                g_apInfoList[apId].channelUtilization = 0.4; 
                break;
            case 3: 
                g_apInfoList[apId].position = Vector(37.5, 37.5, 0.0);
                g_apInfoList[apId].channelUtilization = 0.5; 
                break;
        }
    }

    // アルゴリズム初期化
    LightweightAPSelection algorithm(movementRadius);
    algorithm.UpdateAPInfo(g_apInfoList);
    g_algorithm = &algorithm;

    // ユーザ情報初期化
    g_userInfoList.resize(nUsers);
    
    // 新規ユーザの設定（未出現状態）
    for (uint32_t i = 0; i < nNewUsers; ++i) {
        g_userInfoList[i].userId = i;
        g_userInfoList[i].position = Vector(0.0, 0.0, 0.0);
        g_userInfoList[i].initialPosition = Vector(0.0, 0.0, 0.0);
        g_userInfoList[i].connectedAP = 0;
        g_userInfoList[i].targetPosition = Vector(0.0, 0.0, 0.0);
        g_userInfoList[i].throughput = 0.0;
        g_userInfoList[i].throughputThreshold = requiredThroughput;
        g_userInfoList[i].hasReachedThreshold = false;
        g_userInfoList[i].isNewUser = true;
        g_userInfoList[i].movementDistance = 0.0;
        g_userInfoList[i].isSpawned = false;  // 未出現
    }
    
    // 既存ユーザの設定
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
        g_userInfoList[i].isSpawned = true;  // 既存ユーザは最初から出現済み
    }

    PrintInitialState();

    // インターネットスタック
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

    // アプリケーション設定
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

    // AnimationInterface設定
    AnimationInterface anim(g_sessionDir + "/animation.xml");
    for (uint32_t i = 0; i < nAPs; ++i) {
        anim.UpdateNodeColor(apNodes.Get(i), 0, 255, 0);
    }
    for (uint32_t i = 0; i < nUsers; ++i) {
        anim.UpdateNodeColor(staNodes.Get(i), 
                           g_userInfoList[i].isNewUser ? 255 : 0, 0, 
                           g_userInfoList[i].isNewUser ? 0 : 255);
    }

    // シミュレーション実行
    Simulator::Stop(Seconds(simTime));
    
    // 1秒後から新規ユーザの順次出現を開始
    Simulator::Schedule(Seconds(1.0), &StartUserSpawning);
    
    Simulator::Run();
    PrintFinalResults();
    
    outputFile.close();
    Simulator::Destroy();
    return 0;
}