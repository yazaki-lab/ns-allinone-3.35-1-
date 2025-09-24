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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WiFiAPSelection");

// 構造体定義
struct APInfo {
    uint32_t apId;
    Vector position;
    uint32_t connectedUsers;
    double channelUtilization;
    uint32_t channel;
    double totalThroughput; 
};

struct UserInfo {
    uint32_t userId;
    Vector position;
    Vector initialPosition; 
    uint32_t connectedAP;
    double throughput;
    bool isNewUser;
    double movementDistance;
    bool hasMovedRandomly;
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

// 軽量化されたAPセレクションアルゴリズム  
class LightweightAPSelection {
private:
    std::vector<APInfo> m_apList;
    double m_dThreshold;
    
    // 距離計算結果をキャッシュ
    std::map<std::pair<uint32_t, uint32_t>, double> m_distanceCache;

public:
    LightweightAPSelection(double dTh = 25.0) : m_dThreshold(dTh) {}

    void UpdateAPInfo(const std::vector<APInfo>& apList) { 
        m_apList = apList; 
        m_distanceCache.clear(); // キャッシュクリア
    }

    double CalculateDistance(const Vector& pos1, const Vector& pos2) {
        return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
    }

    double CalculateTransmissionRate(double distance) {
        if (distance < 5.0) return 150.0;
        else if (distance < 10.0) return 130.0;
        else if (distance < 15.0) return 100.0;
        else if (distance < 20.0) return 65.0;
        else if (distance < 25.0) return 30.0;
        else return 6.5;
    }

    // シンプルなスループット計算
    double CalculateThroughput(const APInfo& ap, double transmissionRate) {
        return transmissionRate * (1.0 - ap.channelUtilization) / std::max(1.0, (double)ap.connectedUsers);
    }

    uint32_t SelectOptimalAP(const Vector& userPos) {
        uint32_t bestAP = 0;
        double bestScore = -1.0;

        for (const auto& ap : m_apList) {
            double distance = CalculateDistance(userPos, ap.position);
            
            // 距離制限チェック
            if (distance > m_dThreshold) continue;
            
            double transmissionRate = CalculateTransmissionRate(distance);
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
static double g_environmentSize = 50.0;
static LightweightAPSelection* g_algorithm = nullptr;
static std::vector<APInfo> g_apInfoList;
static std::vector<UserInfo> g_userInfoList;
static std::vector<Ptr<LightweightMobilityModel>> g_userMobilityModels;
static std::string g_sessionDir = "";
static std::mt19937 g_randomEngine;
static std::uniform_real_distribution<double> g_positionDistribution(0.0, 50.0);

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

// ランダム位置生成
Vector GenerateRandomPosition() {
    double x = g_positionDistribution(g_randomEngine);
    double y = g_positionDistribution(g_randomEngine);
    return Vector(x, y, 0.0);
}

// 位置分散計算（簡略化）
std::pair<double, double> CalculatePositionVariance() {
    if (g_nUsers == 0) return {0.0, 0.0};
    
    double meanX = 0.0, meanY = 0.0;
    double varX = 0.0, varY = 0.0;
    
    // 平均計算
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Vector pos = g_userInfoList[i].position;
        meanX += pos.x;
        meanY += pos.y;
    }
    meanX /= g_nUsers;
    meanY /= g_nUsers;
    
    // 分散計算
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Vector pos = g_userInfoList[i].position;
        varX += pow(pos.x - meanX, 2);
        varY += pow(pos.y - meanY, 2);
    }
    varX /= g_nUsers;
    varY /= g_nUsers;
    
    return {varX, varY};
}

// CSVファイル出力（詳細化）
void OutputResultsToCSV(double finalSystemThroughput, double initialSystemThroughput) {
    std::string csvDir = "results_csv";
    CreateDirectory(csvDir);
    
    std::string csvFile = csvDir + "/random_AP" + std::to_string(g_nAPs) + "User" + std::to_string(g_nUsers) + "_09241538.csv";
    
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
    for (uint32_t i = 0; i < g_nNewUsers; ++i) {
        if (g_userInfoList[i].isNewUser) {
            double improvementRate = (initialSystemThroughput > 0.0) ? 
                ((finalSystemThroughput - initialSystemThroughput) / initialSystemThroughput) * 100.0 : 0.0;
            
            csvOutput << std::fixed << std::setprecision(2);
            csvOutput << "random," 
                     << "User" << i << ","
                     << g_movementRadius << ","
                     << "N/A" << ","  // lightweightではrequiredThroughputが定義されていないため
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

// スループット計算
double CalculateCurrentThroughput(uint32_t userId, uint32_t apId) {
    if (userId >= g_nUsers || apId >= g_nAPs) return 0.0;
    
    Vector userPos = g_userInfoList[userId].position;
    Vector apPos = g_apInfoList[apId].position;
    
    double distance = sqrt(pow(userPos.x - apPos.x, 2) + pow(userPos.y - apPos.y, 2));
    
    double baseThroughput = g_algorithm->CalculateTransmissionRate(distance);
    return baseThroughput * (1.0 - g_apInfoList[apId].channelUtilization);
}

// ユーザ移動更新（簡略化）
void UpdateUserMovement() {
    if (!g_algorithm || g_userMobilityModels.empty()) return;
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (!g_userInfoList[i].isNewUser || !g_userMobilityModels[i]) continue;
        
        // ランダム移動をまだ開始していない場合は開始
        if (!g_userMobilityModels[i]->HasStartedMovement()) {
            g_userMobilityModels[i]->SetRandomTarget(g_movementRadius, g_environmentSize);
            
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
            g_userInfoList[i].hasMovedRandomly = true;
            
            OUTPUT("新規ユーザ" << i << ": ランダム移動完了 最終位置(" 
                   << std::fixed << std::setprecision(1) << userPos.x << ", " << userPos.y 
                   << "), 移動距離: " << std::setprecision(2) << g_userInfoList[i].movementDistance 
                   << "m -> AP" << selectedAP << " 接続, スループット: " 
                   << std::setprecision(2) << currentThroughput << "Mbps\n");
        }
    }
}

// 動的なAP配置生成
std::vector<Vector> GenerateAPPositions(uint32_t nAPs, double environmentSize) {
    std::vector<Vector> positions;
    
    if (nAPs == 1) {
        positions.push_back(Vector(environmentSize/2, environmentSize/2, 0.0));
    } else if (nAPs == 2) {
        positions.push_back(Vector(environmentSize/4, environmentSize/2, 0.0));
        positions.push_back(Vector(3*environmentSize/4, environmentSize/2, 0.0));
    } else if (nAPs == 4) {
        positions.push_back(Vector(environmentSize/4, environmentSize/4, 0.0));
        positions.push_back(Vector(3*environmentSize/4, environmentSize/4, 0.0));
        positions.push_back(Vector(environmentSize/4, 3*environmentSize/4, 0.0));
        positions.push_back(Vector(3*environmentSize/4, 3*environmentSize/4, 0.0));
    } else {
        // 他のAP数に対する一般的な配置
        double angle = 0.0;
        double radius = environmentSize * 0.3;
        Vector center(environmentSize/2, environmentSize/2, 0.0);
        
        for (uint32_t i = 0; i < nAPs; ++i) {
            double x = center.x + radius * cos(angle);
            double y = center.y + radius * sin(angle);
            positions.push_back(Vector(x, y, 0.0));
            angle += 2 * M_PI / nAPs;
        }
    }
    
    return positions;
}

// 動的な既存ユーザ配置生成
std::vector<Vector> GenerateExistingUserPositions(uint32_t nExistingUsers, double environmentSize) {
    std::vector<Vector> positions;
    
    if (nExistingUsers == 0) return positions;
    
    // グリッド配置を基本とする
    uint32_t gridSize = (uint32_t)ceil(sqrt(nExistingUsers));
    double spacing = environmentSize / (gridSize + 1);
    
    uint32_t index = 0;
    for (uint32_t row = 0; row < gridSize && index < nExistingUsers; ++row) {
        for (uint32_t col = 0; col < gridSize && index < nExistingUsers; ++col) {
            double x = spacing * (col + 1);
            double y = spacing * (row + 1);
            positions.push_back(Vector(x, y, 0.0));
            index++;
        }
    }
    
    return positions;
}

// 初期状態表示（簡略化）
void PrintInitialState() {
    OUTPUT("\n=== WiFi APセレクションシミュレーション（軽量化版）===\n");
    OUTPUT("AP数: " << g_nAPs << ", ユーザ数: " << g_nUsers);
    OUTPUT(" (新規:" << g_nNewUsers << ", 既存:" << g_nExistingUsers << ")\n");
    OUTPUT("シミュレーション時間: " << g_simTime << "秒\n");
    OUTPUT("移動許容距離: " << g_movementRadius << "m\n");
    OUTPUT("環境サイズ: " << g_environmentSize << "m×" << g_environmentSize << "m\n");
    
    // 実験前のシステム全体のスループット
    double initialSystemThroughput = CalculateSystemThroughput();
    OUTPUT("\n実験前システム全体スループット（調和平均）: " 
           << std::fixed << std::setprecision(2) << initialSystemThroughput << " Mbps\n");
}

void StartUserMovement() {
    OUTPUT("\n新規ユーザのランダム移動を開始します。\n");
    UpdateUserMovement();
}

// 最終結果表示
void PrintFinalResults() {
    OUTPUT("\n=== シミュレーション完了 ===\n");
    OUTPUT("結果ディレクトリ: " << g_sessionDir << "\n");
    
    // システム全体スループット評価
    double finalSystemThroughput = CalculateSystemThroughput();
    
    // 初期スループット計算（簡略化）
    double initialSystemThroughput = 0.0;
    for (uint32_t apId = 0; apId < g_nAPs; ++apId) {
        double apThroughput = 0.0;
        for (uint32_t i = 0; i < g_nUsers; ++i) {
            if (g_userInfoList[i].connectedAP == apId) {
                Vector initialPos = g_userInfoList[i].initialPosition;
                Vector apPos = g_apInfoList[apId].position;
                double distance = sqrt(pow(initialPos.x - apPos.x, 2) + pow(initialPos.y - apPos.y, 2));
                double baseThroughput = g_algorithm->CalculateTransmissionRate(distance);
                apThroughput += baseThroughput * (1.0 - g_apInfoList[apId].channelUtilization);
            }
        }
        if (apThroughput > 0.0) {
            initialSystemThroughput += 1.0 / apThroughput;
        }
    }
    if (initialSystemThroughput > 0.0) {
        initialSystemThroughput = g_nAPs / initialSystemThroughput;
    }

    // 改善率の計算
    double improvement = finalSystemThroughput - initialSystemThroughput;
    double improvementPercent = (initialSystemThroughput > 0.0) ? 
        (improvement / initialSystemThroughput) * 100.0 : 0.0;
    
    OUTPUT("\nシステム全体スループット（調和平均）: " 
           << std::fixed << std::setprecision(2) << finalSystemThroughput << " Mbps\n");
    OUTPUT("初期システムスループット: " << std::setprecision(2) << initialSystemThroughput << " Mbps\n");
    OUTPUT("スループット改善量: " << std::setprecision(2) << improvement << " Mbps\n");
    OUTPUT("改善率: " << std::setprecision(1) << improvementPercent << "%\n");
    
    // CSV出力
    OutputResultsToCSV(finalSystemThroughput, initialSystemThroughput);
}

int main(int argc, char *argv[]) {
    // パラメータ設定（動的に変更可能）
    uint32_t nAPs = 4;
    uint32_t nNewUsers = 10;
    uint32_t nExistingUsers = 90;
    uint32_t nUsers = nNewUsers + nExistingUsers;
    double simTime = 10.0; // 短縮
    double movementRadius = 15.0;
    double environmentSize = 50.0;

    // コマンドライン引数の処理
    CommandLine cmd;
    cmd.AddValue("nAPs", "AP数", nAPs);
    cmd.AddValue("nNewUsers", "新規ユーザ数", nNewUsers);
    cmd.AddValue("nExistingUsers", "既存ユーザ数", nExistingUsers);
    cmd.AddValue("movementRadius", "移動許容距離 (m)", movementRadius);
    cmd.AddValue("simTime", "シミュレーション時間 (秒)", simTime);
    cmd.AddValue("environmentSize", "環境サイズ (m)", environmentSize);
    cmd.Parse(argc, argv);

    nUsers = nNewUsers + nExistingUsers;
    
    // グローバル変数設定
    g_nAPs = nAPs;
    g_nUsers = nUsers;
    g_nNewUsers = nNewUsers;
    g_nExistingUsers = nExistingUsers;
    g_simTime = simTime;
    g_movementRadius = movementRadius;
    g_environmentSize = environmentSize;

    // ランダム生成器の初期化
    g_randomEngine.seed(static_cast<unsigned int>(time(nullptr)));
    g_positionDistribution = std::uniform_real_distribution<double>(0.0, environmentSize);

    // 出力ディレクトリ作成
    std::string outputDir = "results";
    CreateDirectory(outputDir);
    g_sessionDir = outputDir + "/AP" + std::to_string(nAPs) + "User" + std::to_string(nUsers) + "_lightweight_" + GetTimestamp();
    CreateDirectory(g_sessionDir);

    std::ofstream outputFile(g_sessionDir + "/output.txt");
    g_outputFile = &outputFile;

    // ノード作成
    NodeContainer apNodes, staNodes;
    apNodes.Create(nAPs);
    staNodes.Create(nUsers);
    g_apNodes = &apNodes;
    g_staNodes = &staNodes;

    // 簡易WiFi設定
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211a);
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
    
    // 動的AP配置
    std::vector<Vector> apPositions = GenerateAPPositions(nAPs, environmentSize);
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    for (const auto& pos : apPositions) {
        apPositionAlloc->Add(pos);
    }
    
    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // ユーザ配置
    g_userMobilityModels.resize(nUsers);
    
    // 新規ユーザの設定
    for (uint32_t i = 0; i < nNewUsers; ++i) {
        Ptr<LightweightMobilityModel> lightweightMobility = CreateObject<LightweightMobilityModel>();
        
        Vector initialPos = GenerateRandomPosition();
        lightweightMobility->SetPosition(initialPos);
        staNodes.Get(i)->AggregateObject(lightweightMobility);
        g_userMobilityModels[i] = lightweightMobility;
    }
    
    // 既存ユーザの設定
    std::vector<Vector> existingUserPositions = GenerateExistingUserPositions(nExistingUsers, environmentSize);
    for (uint32_t i = nNewUsers; i < nUsers; ++i) {
        Ptr<ConstantPositionMobilityModel> constantMobility = CreateObject<ConstantPositionMobilityModel>();
        Vector fixedPos = existingUserPositions[i - nNewUsers];
        constantMobility->SetPosition(fixedPos);
        staNodes.Get(i)->AggregateObject(constantMobility);
        g_userMobilityModels[i] = nullptr;
    }

    // AP情報を動的に初期化
    g_apInfoList.resize(nAPs);
    for (uint32_t apId = 0; apId < nAPs; ++apId) {
        g_apInfoList[apId].apId = apId;
        g_apInfoList[apId].position = apPositions[apId];
        g_apInfoList[apId].connectedUsers = 0;
        g_apInfoList[apId].channel = apId;
        g_apInfoList[apId].totalThroughput = 0.0;
        g_apInfoList[apId].channelUtilization = 0.2 + (apId * 0.1);
        if (g_apInfoList[apId].channelUtilization > 0.6) {
            g_apInfoList[apId].channelUtilization = 0.6;
        }
    }

    // アルゴリズム初期化
    LightweightAPSelection algorithm(movementRadius);
    algorithm.UpdateAPInfo(g_apInfoList);
    g_algorithm = &algorithm;

    // ユーザ情報初期化
    g_userInfoList.resize(nUsers);
    
    // 新規ユーザの設定
    for (uint32_t i = 0; i < nNewUsers; ++i) {
        g_userInfoList[i].userId = i;
        g_userInfoList[i].position = g_userMobilityModels[i]->GetPosition();
        g_userInfoList[i].initialPosition = g_userMobilityModels[i]->GetPosition();
        g_userInfoList[i].connectedAP = g_algorithm->SelectNearestAP(g_userInfoList[i].initialPosition);
        g_userInfoList[i].throughput = CalculateCurrentThroughput(i, g_userInfoList[i].connectedAP);
        g_userInfoList[i].isNewUser = true;
        g_userInfoList[i].movementDistance = 0.0;
        g_userInfoList[i].hasMovedRandomly = false;
    }
    
    // 既存ユーザの設定
    for (uint32_t i = nNewUsers; i < nUsers; ++i) {
        g_userInfoList[i].userId = i;
        g_userInfoList[i].position = staNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        g_userInfoList[i].initialPosition = g_userInfoList[i].position;
        g_userInfoList[i].connectedAP = g_algorithm->SelectNearestAP(g_userInfoList[i].position);
        g_userInfoList[i].throughput = CalculateCurrentThroughput(i, g_userInfoList[i].connectedAP);
        g_userInfoList[i].isNewUser = false;
        g_userInfoList[i].movementDistance = 0.0;
        g_userInfoList[i].hasMovedRandomly = true; // 既存ユーザは移動しない
    }

    PrintInitialState();

    // インターネットスタック（軽量化）
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

    // アプリケーションは最小限に（軽量化のため削除またはシンプル化）
    // FlowMonitorも削除して軽量化

    // シミュレーション実行
    Simulator::Stop(Seconds(simTime));
    Simulator::Schedule(Seconds(1.0), &StartUserMovement); // 早期開始
    
    Simulator::Run();
    PrintFinalResults();
    
    outputFile.close();
    Simulator::Destroy();
    return 0;
}