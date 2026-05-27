#pragma once

#include <sdk.hpp>
#include <Server/Components/Pawn/pawn.hpp>
#include <Server/Components/NPCs/npcs.hpp>
#include <queue>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <vector>
#include <string>
#include <atomic>
#include <unordered_map>
#include <memory>
#include "pathfinder-utils.hpp"

#pragma pack(push, 1)
struct PathNode {
    uint32_t memAddress;
    uint32_t unknown1;
    int16_t positionX;
    int16_t positionY;
    int16_t positionZ;
    uint16_t unknown2;
    uint16_t linkId;
    uint16_t areaId;
    uint16_t nodeId;
    uint8_t pathWidth;
    uint8_t nodeType;
    uint32_t flags;
};
struct NaviNode {
    int16_t positionX; int16_t positionY;
    uint16_t areaId; uint16_t nodeId;
    uint8_t directionX; uint8_t directionY;
    uint32_t flags;
};
struct LinkNode {
    uint16_t areaId; uint16_t nodeId;
};
struct NodeHeader {
    uint32_t nodesNumber; uint32_t vehicleNodesNumber;
    uint32_t pedNodesNumber; uint32_t naviNodesNumber; uint32_t linksNumber;
};
#pragma pack(pop)

struct alignas(8) NodeLinkInfo {
    uint32_t offset;
    uint32_t count;
};

struct alignas(8) AStarPrecomputedLink {
    uint32_t targetIndex;
    float cost;
};

struct PrecomputedGraph {
    std::vector<Vector3> positions;
    std::vector<NodeLinkInfo> nodeLinkInfo;
    std::vector<AStarPrecomputedLink> links;
    std::vector<std::vector<uint32_t>> areaIdToFlatIndex;
    
    std::vector<uint32_t> wccIds;
    std::vector<uint32_t> sccIds;
};

struct alignas(16) AStarQueueItem {
    float fScore;
    float gScore;
    uint32_t flatIndex;
    bool operator>(const AStarQueueItem& other) const { 
        if (fScore == other.fScore) return gScore < other.gScore;
        return fScore > other.fScore; 
    }
};

struct alignas(16) AStarCell {
    uint32_t cameFrom;
    uint32_t generation;
    float gScore;
};

struct PathTask {
    AMX* amx_ = nullptr;
    uint64_t amx_generation = 0;
    int npc_id = -1;
    std::string callback;
    uint16_t startAreaId = 0, startPointId = 0;
    uint16_t targetAreaId = 0, targetPointId = 0;
};

struct PathResult {
    AMX* amx_ = nullptr;
    uint64_t amx_generation = 0;
    int npc_id = -1;
    std::string callback;
    bool success = false;
    std::vector<Vector3> points;
};

class PathfinderComponent final : public IComponent, public PawnEventHandler, public CoreEventHandler {
private:
    ICore* core_ = nullptr;
    IPawnComponent* pawn_ = nullptr;
    INPCComponent* npcs_ = nullptr; 

    std::vector<std::thread> workers_;
    std::queue<PathTask> tasks_;
    std::queue<PathResult> results_;
    std::mutex taskMutex_;
    std::condition_variable cv_;
    
    alignas(CACHE_LINE) SpinLock resultSpinLock_;
    alignas(CACHE_LINE) std::atomic<int> activeWorkers_{0};
    alignas(CACHE_LINE) bool stopWorkers_ = false;

    inline static PathfinderComponent* instance_ = nullptr;
    std::mutex rawGraphMutex_;

    std::array<std::vector<PathNode>, 64> nodeGraph_;
    std::array<std::vector<LinkNode>, 64> linkGraph_;
    std::array<std::unordered_map<uint16_t, std::vector<LinkNode>>, 64> customLinks_;
    
    std::shared_mutex activeGraphMutex_;
    std::shared_ptr<PrecomputedGraph> activeGraph_;
    
    std::shared_ptr<PrecomputedGraph> getActiveGraph() {
        std::shared_lock<std::shared_mutex> lock(activeGraphMutex_);
        return activeGraph_;
    }

    bool LoadBinaryNodes();
    void RebuildOptimizedGraph();
    void workerLoop();

    std::unordered_map<AMX*, uint64_t> amxGenerations_;
    uint64_t currentGeneration_ = 0;

public:
    uint64_t getAmxGeneration(AMX* amx) {
        auto it = amxGenerations_.find(amx);
        return (it != amxGenerations_.end()) ? it->second : 0;
    }
    
    bool isBusy() { 
        std::lock_guard<std::mutex> lock(taskMutex_);
        return activeWorkers_.load(std::memory_order_relaxed) > 0 || !tasks_.empty(); 
    }

    int addCustomNode(int areaId, float x, float y, float z);
    bool loadCustomNodesFromFile(const std::string& filename);
    bool addCustomLink(int areaId, int nodeId, int targetAreaId, int targetNodeId);
    bool destroyPath(int pathId);
    bool getNodePos(int areaId, int nodeId, float& x, float& y, float& z);
    float getDistanceBetweenNodes(int area1, int node1, int area2, int node2);
    bool getClosestNode(float x, float y, float z, int& outArea, int& outNode);

    PROVIDE_UID(0x45953C1B51ADE1C8);

    PathfinderComponent();
    ~PathfinderComponent();
    static PathfinderComponent* getInstance();

    bool enqueueTask(PathTask task);

    StringView componentName() const override { return "open.mp A* Pathfinder Component"; }
    SemanticVersion componentVersion() const override { return SemanticVersion(1, 2, 1, 0); }

    void onLoad(ICore* c) override;
    void onInit(IComponentList* components) override;
    void onReady() override {}
    void onFree(IComponent* component) override;
    void free() override { delete this; }
    void reset() override {}

    void onAmxLoad(IPawnScript& script) override;
    void onAmxUnload(IPawnScript& script) override;
    void onTick(Microseconds elapsed, TimePoint now) override;
};//PathfinderComponent