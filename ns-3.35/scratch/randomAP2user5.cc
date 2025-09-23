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

NS_LOG_COMPONENT_DEFINE("WiFiRandomMovement");

// 構造体定義（元のコードと同じ）
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
    Vector targetPosition;  // ランダム移動の目標位置
    bool hasReachedTarget;  // 目標位置に到達したかどうか
    double movementDistance; // 移動距離を追加
};

// 位置分散計算構造体を追加
struct PositionVariance {
    double varianceX;
    double varianceY;
    double totalVariance;
    Vector centerPosition;
};

// ランダム移動モデル
class RandomMobilityModel : public MobilityModel {
private:
    Vector m_currentPosition;
    Vector m_targetPosition;
    Vector m_initialPosition;
    double m_speed;
    EventId m_moveEvent;
    Time m_moveInterval;
    bool m_isMoving;
    bool m_targetReached;
    double m_movementRadius;
    std::mt19937 m_rng;
    
public:
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("RandomMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("Mobility")
            .AddConstructor<RandomMobilityModel>()
            .AddAttribute("Speed", "移動速度 (m/s)",
                         DoubleValue(1.0),
                         MakeDoubleAccessor(&RandomMobilityModel::m_speed),
                         MakeDoubleChecker<double>())
            .AddAttribute("MoveInterval", "移動更新間隔",
                         TimeValue(Seconds(0.1)),
                         MakeTimeAccessor(&RandomMobilityModel::m_moveInterval),
                         MakeTimeChecker())
            .AddAttribute("MovementRadius", "移動許容半径 (m)",
                         DoubleValue(15.0),
                         MakeDoubleAccessor(&RandomMobilityModel::m_movementRadius),
                         MakeDoubleChecker<double>());
        return tid;
    }
    
    RandomMobilityModel() : m_speed(1.0), m_moveInterval(Seconds(0.1)), 
                           m_isMoving(false), m_targetReached(false), 
                           m_movementRadius(15.0), m_rng(std::random_device{}()) {}
    
    void SetMovementRadius(double radius) {
        m_movementRadius = radius;
    }
    
    void StartRandomMovement() {
        m_initialPosition = m_currentPosition;
        GenerateRandomTarget();
        m_isMoving = true;
        m_targetReached = false;
        ScheduleMove();
    }
    
    void StopMoving() {
        m_isMoving = false;
        m_targetReached = true;
        if (m_moveEvent.IsRunning()) {
            Simulator::Cancel(m_moveEvent);
        }
    }
    
    bool HasReachedTarget() const { return m_targetReached; }
    bool IsMoving() const { return m_isMoving; }
    Vector GetTargetPosition() const { return m_targetPosition; }
    
private:
    void GenerateRandomTarget() {
        std::uniform_real_distribution<double> angleDist(0, 2 * M_PI);
        std::uniform_real_distribution<double> radiusDist(0, m_movementRadius);
        
        double angle = angleDist(m_rng);
        double radius = radiusDist(m_rng);
        
        Vector target = m_initialPosition;
        target.x += radius * cos(angle);
        target.y += radius * sin(angle);
        
        // 境界チェック（0-30mの範囲内に制限）
        target.x = std::max(0.0, std::min(30.0, target.x));
        target.y = std::max(0.0, std::min(30.0, target.y));
        
        m_targetPosition = target;
    }
    
    void ScheduleMove() {
        if (!m_isMoving) return;
        m_moveEvent = Simulator::Schedule(m_moveInterval, &RandomMobilityModel::DoMove, this);
    }
    
    void DoMove() {
        Vector direction = m_targetPosition - m_currentPosition;
        double distance = sqrt(direction.x * direction.x + direction.y * direction.y);
        
        if (distance > 1.0) {
            direction.x /= distance;
            direction.y /= distance;
            direction.z = 0;
            
            double moveDistance = m_speed * m_moveInterval.GetSeconds();
            
            Vector newPosition = m_currentPosition;
            newPosition.x += direction.x * moveDistance;
            newPosition.y += direction.y * moveDistance;
            
            // 境界チェック
            newPosition.x = std::max(0.0, std::min(30.0, newPosition.x));
            newPosition.y = std::max(0.0, std::min(30.0, newPosition.y));
            
            SetPosition(newPosition);
            ScheduleMove();
        } else {
            m_targetReached = true;
            m_isMoving = false;
        }
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

// 最短距離AP選択アルゴリズム
class NearestAPSelection {
public:
    uint32_t SelectNearestAP(const Vector& userPos, const std::vector<APInfo>& apList) {
        double minDistance = 1e9;
        uint32_t selectedAP = 0;
        
        for (const auto& ap : apList) {
            double distance = CalculateDistance(userPos, ap.position);
            if (distance < minDistance) {
                minDistance = distance;
                selectedAP = ap.apId;
            }
        }
        
        return selectedAP;
    }
    
private:
    double CalculateDistance(const Vector& pos1, const Vector& pos2) {
        return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
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
static NearestAPSelection* g_nearestAPSelector = nullptr;
static std::vector<APInfo> g_apInfoList;
static std::vector<UserInfo> g_userInfoList;
static std::vector<Ptr<RandomMobilityModel>> g_userMobilityModels;
static std::string g_sessionDir = "";

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

// 位置の分散を計算する関数
PositionVariance CalculatePositionVariance() {
    PositionVariance variance;
    
    if (g_userInfoList.empty()) {
        variance.varianceX = 0.0;
        variance.varianceY = 0.0;
        variance.totalVariance = 0.0;
        variance.centerPosition = Vector(0.0, 0.0, 0.0);
        return variance;
    }
    
    // 中心位置（重心）を計算
    double sumX = 0.0, sumY = 0.0;
    for (const auto& user : g_userInfoList) {
        sumX += user.position.x;
        sumY += user.position.y;
    }
    
    variance.centerPosition.x = sumX / g_nUsers;
    variance.centerPosition.y = sumY / g_nUsers;
    variance.centerPosition.z = 0.0;
    
    // 分散を計算
    double sumSquaredDeviationX = 0.0;
    double sumSquaredDeviationY = 0.0;
    
    for (const auto& user : g_userInfoList) {
        double deviationX = user.position.x - variance.centerPosition.x;
        double deviationY = user.position.y - variance.centerPosition.y;
        
        sumSquaredDeviationX += deviationX * deviationX;
        sumSquaredDeviationY += deviationY * deviationY;
    }
    
    variance.varianceX = sumSquaredDeviationX / g_nUsers;
    variance.varianceY = sumSquaredDeviationY / g_nUsers;
    variance.totalVariance = variance.varianceX + variance.varianceY;
    
    return variance;
}

// スループット計算関数
double CalculateCurrentThroughput(uint32_t userId, uint32_t apId) {
    if (userId >= g_nUsers || apId >= g_nAPs) return 0.0;
    
    Vector userPos;
    
    // 新規ユーザーはMobilityModelから位置を取得、既存ユーザーは固定位置を使用
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

// ユーザ移動監視とAP接続更新
void MonitorUserMovement() {
    bool anyMoving = false;
    
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (!g_userInfoList[i].isNewUser || !g_userMobilityModels[i]) continue;
        
        if (g_userMobilityModels[i]->IsMoving()) {
            anyMoving = true;
            // 移動中の位置更新
            Vector currentPos = g_userMobilityModels[i]->GetPosition();
            g_userInfoList[i].position = currentPos;
        } else if (!g_userInfoList[i].hasReachedTarget) {
            // 移動停止時の処理
            g_userInfoList[i].hasReachedTarget = true;
            Vector finalPos = g_userMobilityModels[i]->GetPosition();
            g_userInfoList[i].position = finalPos;
            g_userInfoList[i].targetPosition = finalPos;
            
            // 移動距離を計算
            Vector initialPos = g_userInfoList[i].initialPosition;
            g_userInfoList[i].movementDistance = sqrt(
                pow(finalPos.x - initialPos.x, 2) + pow(finalPos.y - initialPos.y, 2));
            
            // 最も近いAPを選択
            uint32_t nearestAP = g_nearestAPSelector->SelectNearestAP(finalPos, g_apInfoList);
            g_userInfoList[i].connectedAP = nearestAP;
            
            // スループット計算
            double currentThroughput = CalculateCurrentThroughput(i, nearestAP);
            g_userInfoList[i].throughput = currentThroughput;
            
            OUTPUT("新規ユーザ" << i << ": 移動完了 位置(" 
                   << std::fixed << std::setprecision(1) << finalPos.x << ", " << finalPos.y 
                   << ") -> AP" << nearestAP << " 接続, スループット: " 
                   << std::setprecision(2) << currentThroughput << "Mbps\n");
        }
    }
    
    if (anyMoving && Simulator::Now().GetSeconds() < g_simTime - 1.0) {
        Simulator::Schedule(Seconds(0.5), &MonitorUserMovement);
    }
}

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

// CSVファイルに結果を追加出力する関数（位置分散情報を追加）
void OutputResultsToCSV(double finalSystemThroughput, double initialSystemThroughput) {
    // 位置の分散を計算
    PositionVariance posVariance = CalculatePositionVariance();
    
    // results_csvディレクトリの作成
    std::string csvDir = "results_csv";
    CreateDirectory(csvDir);
    
    std::string csvFile = csvDir + "/random_AP2user5_1114.csv";
    
    // ファイルが存在しない場合はヘッダーを追加
    bool fileExists = false;
    std::ifstream checkFile(csvFile);
    if (checkFile.good()) {
        fileExists = true;
    }
    checkFile.close();
    
    std::ofstream csvOutput(csvFile, std::ios::app); // 追記モード
    
    if (!fileExists) {
        // ヘッダー行を追加（位置分散関連の列を追加）
        csvOutput << "Method,NewUserName,MovementRadius,RequiredThroughput,MovementDistance,ConnectedAP,"
                  << "FinalSystemThroughput,InitialSystemThroughput,ImprovementRate,"
                  << "CenterX,CenterY,VarianceX,VarianceY,TotalVariance" << std::endl;
    }
    
    // 新規ユーザーごとにデータを出力
    for (uint32_t i = 0; i < g_nNewUsers; ++i) {
        if (g_userInfoList[i].isNewUser) {
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
                     << posVariance.centerPosition.x << ","
                     << posVariance.centerPosition.y << ","
                     << posVariance.varianceX << ","
                     << posVariance.varianceY << ","
                     << posVariance.totalVariance << std::endl;
        }
    }
    
    csvOutput.close();
    OUTPUT("CSVファイルに結果を出力しました: " << csvFile << "\n");
}

// 初期状態表示
void PrintInitialState() {
    OUTPUT("\n=== WiFi ランダム移動シミュレーション（比較用） ===\n");
    OUTPUT("AP数: " << g_nAPs << ", ユーザ数: " << g_nUsers);
    OUTPUT(" (新規:" << g_nNewUsers << ", 既存:" << g_nExistingUsers << ")\n");
    OUTPUT("シミュレーション時間: " << g_simTime << "秒\n");
    OUTPUT("移動許容距離: " << g_movementRadius << "m\n");
    OUTPUT("選択方法: 最短距離AP選択\n");
    
    // 初期システムスループット
    double initialSystemThroughput = CalculateSystemThroughput();
    OUTPUT("実験前システム全体スループット（調和平均）: " 
           << std::fixed << std::setprecision(2) << initialSystemThroughput << " Mbps\n");
    
    // 新規ユーザーの目標位置を表示
    OUTPUT("\n新規ユーザーの移動目標:\n");
    for (uint32_t i = 0; i < g_nNewUsers; ++i) {
        Vector target = g_userMobilityModels[i]->GetTargetPosition();
        Vector initial = g_userInfoList[i].initialPosition;
        double distance = sqrt(pow(target.x - initial.x, 2) + pow(target.y - initial.y, 2));
        OUTPUT("ユーザ" << i << ": (" << std::fixed << std::setprecision(1) 
               << initial.x << ", " << initial.y << ") -> (" 
               << target.x << ", " << target.y << "), 距離: " 
               << std::setprecision(2) << distance << "m\n");
    }
    
    // 初期位置の分散を表示
    PositionVariance initialVariance = CalculatePositionVariance();
    OUTPUT("\n初期位置の分散:\n");
    OUTPUT("重心位置: (" << std::fixed << std::setprecision(2) 
           << initialVariance.centerPosition.x << ", " << initialVariance.centerPosition.y << ")\n");
    OUTPUT("X方向分散: " << std::setprecision(2) << initialVariance.varianceX << "\n");
    OUTPUT("Y方向分散: " << std::setprecision(2) << initialVariance.varianceY << "\n");
    OUTPUT("総分散: " << std::setprecision(2) << initialVariance.totalVariance << "\n");
}

void StartUserMovement() {
    OUTPUT("\n新規ユーザーのランダム移動を開始します。\n");
    
    // すべての新規ユーザーの移動を開始
    for (uint32_t i = 0; i < g_nNewUsers; ++i) {
        if (g_userMobilityModels[i]) {
            g_userMobilityModels[i]->StartRandomMovement();
        }
    }
    
    MonitorUserMovement();
}

// 最終結果表示
void PrintFinalResults() {
    OUTPUT("\n=== シミュレーション完了 ===\n");
    OUTPUT("結果ディレクトリ: " << g_sessionDir << "\n");
    OUTPUT("移動許容距離: " << g_movementRadius << "m\n");
    OUTPUT("選択方法: 最短距離AP選択\n");
    
    // 最終位置とスループット表示
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        if (g_userInfoList[i].isNewUser) {
            Vector pos = g_userInfoList[i].position;
            OUTPUT("新規ユーザ" << i << ": 最終位置(" << std::fixed << std::setprecision(1) 
                   << pos.x << ", " << pos.y << "), 移動距離: " 
                   << std::setprecision(2) << g_userInfoList[i].movementDistance 
                   << "m, 接続AP: " << g_userInfoList[i].connectedAP
                   << ", スループット: " << std::setprecision(2) 
                   << g_userInfoList[i].throughput << "Mbps\n");
        }
    }
    
    // 最終位置の分散を表示
    PositionVariance finalVariance = CalculatePositionVariance();
    OUTPUT("\n最終位置の分散:\n");
    OUTPUT("重心位置: (" << std::fixed << std::setprecision(2) 
           << finalVariance.centerPosition.x << ", " << finalVariance.centerPosition.y << ")\n");
    OUTPUT("X方向分散: " << std::setprecision(2) << finalVariance.varianceX << "\n");
    OUTPUT("Y方向分散: " << std::setprecision(2) << finalVariance.varianceY << "\n");
    OUTPUT("総分散: " << std::setprecision(2) << finalVariance.totalVariance << "\n");
    
    // システム全体スループット評価
    double finalSystemThroughput = CalculateSystemThroughput();
    
    // 初期スループット計算（元のコードと同じ方法）
    // 一時的にAPの情報をリセット
    for (uint32_t apId = 0; apId < g_nAPs; ++apId) {
        g_apInfoList[apId].totalThroughput = 0.0;
        g_apInfoList[apId].connectedUsers = 0;
    }
    
    // 初期位置での各ユーザーのスループットを計算
    for (uint32_t i = 0; i < g_nUsers; ++i) {
        Vector initialPos = g_userInfoList[i].initialPosition;
        uint32_t initialConnectedAP;
        
        if (g_userInfoList[i].isNewUser) {
            // 新規ユーザーの場合、初期位置での最短距離AP選択
            initialConnectedAP = g_nearestAPSelector->SelectNearestAP(initialPos, g_apInfoList);
        } else {
            // 既存ユーザーの場合、現在の接続APを使用
            initialConnectedAP = g_userInfoList[i].connectedAP;
        }
        
        // 初期位置でのスループット計算
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
        
        // APごとの総スループットに追加
        g_apInfoList[initialConnectedAP].totalThroughput += initialThroughput;
        g_apInfoList[initialConnectedAP].connectedUsers++;
    }
    
    // 実験前の調和平均を計算
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

    // 改善率の計算
    double improvement = finalSystemThroughput - initialSystemThroughput;
    double improvementPercent = (initialSystemThroughput > 0.0) ? 
        (improvement / initialSystemThroughput) * 100.0 : 0.0;
    
    OUTPUT("システム全体スループット（調和平均）: " 
           << std::fixed << std::setprecision(2) << finalSystemThroughput << " Mbps\n");
    OUTPUT("初期システムスループット: " << std::setprecision(2) << initialSystemThroughput << " Mbps\n");
    OUTPUT("スループット改善量: " << std::setprecision(2) << improvement << " Mbps\n");
    OUTPUT("改善率: " << std::setprecision(1) << improvementPercent << "%\n");
    
    // CSV出力
    OutputResultsToCSV(finalSystemThroughput, initialSystemThroughput);
}

int main(int argc, char *argv[]) {
    // パラメータ設定（元のコードと同じ）
    uint32_t nAPs = 2;
    uint32_t nNewUsers = 2;
    uint32_t nExistingUsers = 3;
    uint32_t nUsers = nNewUsers + nExistingUsers;
    double simTime = 30.0;
    double movementRadius = 15.0;
    double requiredThroughput = 90.0;

    // コマンドライン引数の処理
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

    // 出力ディレクトリ作成
    std::string outputDir = "results";
    CreateDirectory(outputDir);
    g_sessionDir = outputDir + "/AP2User5_random_" + GetTimestamp();
    CreateDirectory(g_sessionDir);

    std::ofstream outputFile(g_sessionDir + "/output.txt");
    g_outputFile = &outputFile;

    // ノード作成
    NodeContainer apNodes, staNodes;
    apNodes.Create(nAPs);
    staNodes.Create(nUsers);
    g_apNodes = &apNodes;
    g_staNodes = &staNodes;

    // WiFi設定（元のコードと同じ）
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
    
    // AP配置（元のコードと同じ）
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(15.0, 15.0, 0.0));
    apPositionAlloc->Add(Vector(5.0, 5.0, 0.0));
    
    mobility.SetPositionAllocator(apPositionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // ユーザ配置
    g_userMobilityModels.resize(nUsers);
    
    // 新規ユーザの設定（ランダム移動モデル）
    for (uint32_t i = 0; i < nNewUsers; ++i) {
        Ptr<RandomMobilityModel> randomMobility = CreateObject<RandomMobilityModel>();
        Vector initialPos = (i == 0) ? Vector(25.0, 0.0, 0.0) : Vector(0.0, 25.0, 0.0);
        randomMobility->SetPosition(initialPos);
        randomMobility->SetAttribute("Speed", DoubleValue(3.0));
        randomMobility->SetMovementRadius(movementRadius);
        staNodes.Get(i)->AggregateObject(randomMobility);
        g_userMobilityModels[i] = randomMobility;
    }
    
    // 既存ユーザの設定（固定位置、元のコードと同じ）
    for (uint32_t i = nNewUsers; i < nUsers; ++i) {
        Ptr<ConstantPositionMobilityModel> constantMobility = CreateObject<ConstantPositionMobilityModel>();
        Vector fixedPos;
        if (i == 2) fixedPos = Vector(12.0, 8.0, 0.0);
        else if (i == 3) fixedPos = Vector(8.0, 12.0, 0.0);
        else if (i == 4) fixedPos = Vector(20.0, 20.0, 0.0);
        
        constantMobility->SetPosition(fixedPos);
        staNodes.Get(i)->AggregateObject(constantMobility);
        g_userMobilityModels[i] = nullptr;
    }

    // AP情報を初期化（元のコードと同じ）
    g_apInfoList.resize(nAPs);
    
    g_apInfoList[0].apId = 0;
    g_apInfoList[0].position = Vector(15.0, 15.0, 0.0);
    g_apInfoList[0].connectedUsers = 0;
    g_apInfoList[0].channelUtilization = 0.2; // 20%
    g_apInfoList[0].channel = 0;
    g_apInfoList[0].totalThroughput = 0.0;

    g_apInfoList[1].apId = 1;
    g_apInfoList[1].position = Vector(5.0, 5.0, 0.0);
    g_apInfoList[1].connectedUsers = 0;
    g_apInfoList[1].channelUtilization = 0.6; // 60%
    g_apInfoList[1].channel = 1;
    g_apInfoList[1].totalThroughput = 0.0;

    // ユーザ情報初期化
    g_userInfoList.resize(nUsers);
    
    // 新規ユーザの設定
    for (uint32_t i = 0; i < nNewUsers; ++i) {
        g_userInfoList[i].userId = i;
        g_userInfoList[i].position = g_userMobilityModels[i]->GetPosition();
        g_userInfoList[i].initialPosition = g_userMobilityModels[i]->GetPosition();
        g_userInfoList[i].connectedAP = 0; // 初期は適当に設定
        g_userInfoList[i].throughput = 0.0;
        g_userInfoList[i].throughputThreshold = requiredThroughput;
        g_userInfoList[i].hasReachedThreshold = false;
        g_userInfoList[i].isNewUser = true;
        g_userInfoList[i].hasReachedTarget = false;
        g_userInfoList[i].movementDistance = 0.0; // 初期化
    }
    
    // 既存ユーザの設定（元のコードと同じ）
    for (uint32_t i = nNewUsers; i < nUsers; ++i) {
        g_userInfoList[i].userId = i;
        g_userInfoList[i].position = staNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        g_userInfoList[i].initialPosition = g_userInfoList[i].position;
        // 既存ユーザのAP接続を決定（近いAPに接続）
        double distToAP0 = sqrt(pow(g_userInfoList[i].position.x - g_apInfoList[0].position.x, 2) +
                               pow(g_userInfoList[i].position.y - g_apInfoList[0].position.y, 2));
        double distToAP1 = sqrt(pow(g_userInfoList[i].position.x - g_apInfoList[1].position.x, 2) +
                               pow(g_userInfoList[i].position.y - g_apInfoList[1].position.y, 2));
        g_userInfoList[i].connectedAP = (distToAP0 < distToAP1) ? 0 : 1;
        g_userInfoList[i].throughput = CalculateCurrentThroughput(i, g_userInfoList[i].connectedAP);
        g_userInfoList[i].throughputThreshold = 0.0;
        g_userInfoList[i].hasReachedThreshold = true;
        g_userInfoList[i].isNewUser = false;
        g_userInfoList[i].hasReachedTarget = true;
        g_userInfoList[i].movementDistance = 0.0; // 既存ユーザは移動しない
    }

    // 最短距離AP選択アルゴリズム初期化
    NearestAPSelection nearestAPSelector;
    g_nearestAPSelector = &nearestAPSelector;

    PrintInitialState();

    // インターネットスタック（元のコードと同じ）
    InternetStackHelper stack;
    stack.Install(apNodes);
    stack.Install(staNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);

    // UDPアプリケーション設定（FlowMonitor用）
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

    // FlowMonitor設定（統計情報収集用）
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // アニメーション設定
    AnimationInterface anim(g_sessionDir + "/animation.xml");
    anim.UpdateNodeColor(apNodes.Get(0), 0, 255, 0);   // AP0: 緑
    anim.UpdateNodeColor(apNodes.Get(1), 0, 255, 255); // AP1: シアン
    for (uint32_t i = 0; i < nUsers; ++i) {
        anim.UpdateNodeColor(staNodes.Get(i), 
                           g_userInfoList[i].isNewUser ? 255 : 0, 0, 
                           g_userInfoList[i].isNewUser ? 0 : 255);
        // 新規ユーザー: 赤, 既存ユーザー: 青
    }

    // シミュレーション実行
    Simulator::Stop(Seconds(simTime));
    Simulator::Schedule(Seconds(3.0), &StartUserMovement);
    
    Simulator::Run();
    PrintFinalResults();
    
    outputFile.close();
    Simulator::Destroy();
    return 0;
}