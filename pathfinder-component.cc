#include "pathfinder-component.hpp"
#include <Server/Components/Pawn/Impl/pawn_natives.hpp>
#include <Server/Components/Pawn/Impl/pawn_impl.hpp>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <limits>
#include <algorithm>
#include <cmath>

#ifndef OMP_UNLIKELY
#if defined(__GNUC__) || defined(__clang__)
    #define OMP_UNLIKELY(x) __builtin_expect(!!(x), 0)
    #define OMP_LIKELY(x)   __builtin_expect(!!(x), 1)
#else
    #define OMP_UNLIKELY(x) (x)
    #define OMP_LIKELY(x)   (x)
#endif
#endif


inline float SmartHeuristic(const Vector3& curr, const Vector3& target, const Vector3& start) noexcept {
    float dx = curr.x - target.x;
    float dy = curr.y - target.y;
    float dz = curr.z - target.z;
    float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    float dx2 = start.x - target.x;
    float dy2 = start.y - target.y;
    float cross = std::abs(dx * dy2 - dx2 * dy);
    return dist + (cross * 0.001f);
}

PathfinderComponent::PathfinderComponent() { instance_ = this; }

PathfinderComponent::~PathfinderComponent() {
    {
        std::lock_guard<std::mutex> lock(taskMutex_);
        stopWorkers_ = true;
        std::queue<PathTask> empty;
        std::swap(tasks_, empty); 
    }
    cv_.notify_all();
    
    for (std::thread& worker : workers_) {
        if (worker.joinable()) worker.join();
    }
    
    if (pawn_) pawn_->getEventDispatcher().removeEventHandler(this);
    if (core_) core_->getEventDispatcher().removeEventHandler(this);
}

PathfinderComponent* PathfinderComponent::getInstance() {
    if (!instance_) instance_ = new PathfinderComponent();
    return instance_;
}

void PathfinderComponent::RebuildOptimizedGraph() {
    auto newGraph = std::make_shared<PrecomputedGraph>();
    newGraph->areaIdToFlatIndex.resize(64);

    uint32_t totalNodes = 0;
    for (int i = 0; i < 64; ++i) {
        newGraph->areaIdToFlatIndex[i].resize(nodeGraph_[i].size(), 0xFFFFFFFF);
        for (size_t j = 0; j < nodeGraph_[i].size(); ++j) {
            newGraph->areaIdToFlatIndex[i][j] = totalNodes++;
        }
    }

    newGraph->positions.resize(totalNodes);
    newGraph->nodeLinkInfo.resize(totalNodes);
    
    std::vector<std::vector<uint32_t>> undirectedAdj(totalNodes);

    for (int area = 0; area < 64; ++area) {
        for (size_t id = 0; id < nodeGraph_[area].size(); ++id) {
            uint32_t flatId = newGraph->areaIdToFlatIndex[area][id];
            auto& n = nodeGraph_[area][id];
            
            //unquantise position values
            newGraph->positions[flatId] = Vector3(
                static_cast<float>(n.positionX) * 0.125f, 
                static_cast<float>(n.positionY) * 0.125f, 
                static_cast<float>(n.positionZ) * 0.125f + 1.2f
            );
            
            uint32_t linkOffset = static_cast<uint32_t>(newGraph->links.size());
            uint16_t linkStart = n.linkId;
            uint16_t linkCount = n.flags & 0xF;//link count is stored in the lower of 4 bits
            
            for (uint16_t i = 0; i < linkCount; ++i) {
                if (linkStart + i >= linkGraph_[area].size()) continue;
                const auto& ln = linkGraph_[area][linkStart + i];
                if (ln.areaId >= 64 || ln.nodeId >= nodeGraph_[ln.areaId].size()) continue;
                
                uint32_t targetFlat = newGraph->areaIdToFlatIndex[ln.areaId][ln.nodeId];
                if (targetFlat == 0xFFFFFFFF) continue;

                Vector3 targetPos = Vector3(
                    static_cast<float>(nodeGraph_[ln.areaId][ln.nodeId].positionX) * 0.125f,
                    static_cast<float>(nodeGraph_[ln.areaId][ln.nodeId].positionY) * 0.125f,
                    static_cast<float>(nodeGraph_[ln.areaId][ln.nodeId].positionZ) * 0.125f + 1.2f
                );
                float cost = glm::distance(newGraph->positions[flatId], targetPos);
                
                newGraph->links.push_back({targetFlat, cost});                
                undirectedAdj[flatId].push_back(targetFlat);
                undirectedAdj[targetFlat].push_back(flatId);
            }

            if (OMP_UNLIKELY(!customLinks_[area].empty())) {
                auto it = customLinks_[area].find(id);
                if (it != customLinks_[area].end()) {
                    for (const auto& ln : it->second) {
                        if (ln.areaId >= 64 || ln.nodeId >= nodeGraph_[ln.areaId].size()) continue;
                        uint32_t targetFlat = newGraph->areaIdToFlatIndex[ln.areaId][ln.nodeId];
                        
                        Vector3 targetPos = Vector3(
                            static_cast<float>(nodeGraph_[ln.areaId][ln.nodeId].positionX) * 0.125f,
                            static_cast<float>(nodeGraph_[ln.areaId][ln.nodeId].positionY) * 0.125f,
                            static_cast<float>(nodeGraph_[ln.areaId][ln.nodeId].positionZ) * 0.125f + 1.2f
                        );
                        float cost = glm::distance(newGraph->positions[flatId], targetPos);
                        
                        newGraph->links.push_back({targetFlat, cost});
                        undirectedAdj[flatId].push_back(targetFlat);
                        undirectedAdj[targetFlat].push_back(flatId);
                    }
                }
            }
            
            newGraph->nodeLinkInfo[flatId].offset = linkOffset;
            newGraph->nodeLinkInfo[flatId].count = static_cast<uint32_t>(newGraph->links.size()) - linkOffset;
        }
    }

    newGraph->wccIds.assign(totalNodes, 0xFFFFFFFF);
    uint32_t currentWcc = 0;
    std::vector<uint32_t> bfsQueue;
    bfsQueue.reserve(totalNodes);
    
    for (uint32_t i = 0; i < totalNodes; ++i) {
        if (newGraph->wccIds[i] != 0xFFFFFFFF) continue;
        
        bfsQueue.clear();
        bfsQueue.push_back(i);
        newGraph->wccIds[i] = currentWcc;
        size_t head = 0;
        
        while (head < bfsQueue.size()) {
            uint32_t curr = bfsQueue[head++];
            for (uint32_t nbr : undirectedAdj[curr]) {
                if (newGraph->wccIds[nbr] == 0xFFFFFFFF) {
                    newGraph->wccIds[nbr] = currentWcc;
                    bfsQueue.push_back(nbr);
                }
            }
        }
        currentWcc++;
    }

    newGraph->sccIds.assign(totalNodes, 0xFFFFFFFF);
    std::vector<uint32_t> dfn(totalNodes, 0xFFFFFFFF);
    std::vector<uint32_t> low(totalNodes, 0xFFFFFFFF);
    std::vector<bool> onStack(totalNodes, false);
    std::vector<uint32_t> st;
    st.reserve(totalNodes);

    uint32_t timer = 0;
    uint32_t sccCount = 0;

    struct Frame { uint32_t u; uint32_t edgeIdx; };
    std::vector<Frame> callStack;
    callStack.reserve(totalNodes);

    for (uint32_t i = 0; i < totalNodes; ++i) {
        if (dfn[i] != 0xFFFFFFFF) continue;

        callStack.push_back({i, 0});
        dfn[i] = low[i] = timer++;
        st.push_back(i);
        onStack[i] = true;

        while (!callStack.empty()) {
            auto& frame = callStack.back();
            uint32_t u = frame.u;
            const auto& info = newGraph->nodeLinkInfo[u];
            bool pushed = false;

            while (frame.edgeIdx < info.count) {
                uint32_t v = newGraph->links[info.offset + frame.edgeIdx].targetIndex;
                frame.edgeIdx++;

                if (dfn[v] == 0xFFFFFFFF) {
                    dfn[v] = low[v] = timer++;
                    st.push_back(v);
                    onStack[v] = true;
                    callStack.push_back({v, 0});
                    pushed = true;
                    break;
                } else if (onStack[v]) {
                    low[u] = std::min(low[u], dfn[v]);
                }
            }
            if (pushed) continue;

            callStack.pop_back();
            if (!callStack.empty()) {
                uint32_t parent = callStack.back().u;
                low[parent] = std::min(low[parent], low[u]);
            }

            if (low[u] == dfn[u]) {
                while (true) {
                    uint32_t v = st.back();
                    st.pop_back();
                    onStack[v] = false;
                    newGraph->sccIds[v] = sccCount;
                    if (u == v) break;
                }
                sccCount++;
            }
        }
    }

    {
        std::unique_lock<std::shared_mutex> lock(activeGraphMutex_);
        activeGraph_ = newGraph;
    }
}

//if a T2 lawyer is reading this: I AM NOT DISTRIBUTING ANY GTA SA ASSETS.
bool PathfinderComponent::LoadBinaryNodes() {
    std::lock_guard<std::mutex> lock(rawGraphMutex_);
    for (int i = 0; i < 64; ++i) {
        std::string filePath = "scriptfiles/NPCs/nodes/NODES" + std::to_string(i) + ".DAT";
        if (!std::filesystem::exists(filePath)) continue;

        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open()) continue;

        NodeHeader header;
        if (!file.read(reinterpret_cast<char*>(&header), sizeof(NodeHeader))) continue;

        uint32_t totalPathNodes = header.vehicleNodesNumber + header.pedNodesNumber;
        if (totalPathNodes > 0) {
            nodeGraph_[i].resize(totalPathNodes);
            file.read(reinterpret_cast<char*>(nodeGraph_[i].data()), totalPathNodes * sizeof(PathNode));
        }

        if (header.naviNodesNumber > 0) file.seekg(header.naviNodesNumber * sizeof(NaviNode), std::ios::cur); 

        if (header.linksNumber > 0) {
            linkGraph_[i].resize(header.linksNumber);
            file.read(reinterpret_cast<char*>(linkGraph_[i].data()), header.linksNumber * sizeof(LinkNode));
        }
    }
    return true;
}

int PathfinderComponent::addCustomNode(int areaId, float x, float y, float z) {
    if (areaId < 0 || areaId >= 64) return -1;
    
    std::lock_guard<std::mutex> lock(rawGraphMutex_);
    PathNode node{};
    node.positionX = static_cast<int16_t>(std::clamp(x * 8.0f, -32768.0f, 32767.0f));
    node.positionY = static_cast<int16_t>(std::clamp(y * 8.0f, -32768.0f, 32767.0f));
    node.positionZ = static_cast<int16_t>(std::clamp((z - 1.2f) * 8.0f, -32768.0f, 32767.0f));
    node.areaId = static_cast<uint16_t>(areaId);
    node.nodeId = static_cast<uint16_t>(nodeGraph_[areaId].size());
    node.flags = 0; 
    
    nodeGraph_[areaId].push_back(node);
    RebuildOptimizedGraph();
    return node.nodeId;
}

bool PathfinderComponent::addCustomLink(int areaId, int nodeId, int targetAreaId, int targetNodeId) {
    if (areaId < 0 || areaId >= 64 || targetAreaId < 0 || targetAreaId >= 64) return false;
    
    std::lock_guard<std::mutex> lock(rawGraphMutex_);
    if (nodeId >= nodeGraph_[areaId].size() || targetNodeId >= nodeGraph_[targetAreaId].size()) return false;
    
    customLinks_[areaId][static_cast<uint16_t>(nodeId)].push_back({
        static_cast<uint16_t>(targetAreaId), static_cast<uint16_t>(targetNodeId)
    });
    RebuildOptimizedGraph();
    return true;
}

bool PathfinderComponent::destroyPath(int pathId) {
    return npcs_ ? npcs_->destroyPath(pathId) : false;
}

bool PathfinderComponent::enqueueTask(PathTask task) {
    if (task.startAreaId >= 64 || task.targetAreaId >= 64) return false;
    
    auto graph = getActiveGraph();
    if (!graph || task.startPointId >= graph->areaIdToFlatIndex[task.startAreaId].size() || 
        task.targetPointId >= graph->areaIdToFlatIndex[task.targetAreaId].size()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(taskMutex_);
    tasks_.push(std::move(task));
    cv_.notify_one();
    return true;
}

void PathfinderComponent::workerLoop() {
    std::vector<AStarCell> grid;
    std::vector<AStarQueueItem> openSet;
    openSet.reserve(16384);
    
    uint32_t currentGen = 0;
    const uint32_t MAX_ITERATIONS = 200000;

    while (true) {
        PathTask task;
        {
            std::unique_lock<std::mutex> lock(taskMutex_);
            cv_.wait(lock, [this] { return stopWorkers_ || !tasks_.empty(); });
            if (stopWorkers_ && tasks_.empty()) return;

            task = std::move(tasks_.front());
            tasks_.pop();
            activeWorkers_.fetch_add(1, std::memory_order_relaxed);
        }

        auto graph = getActiveGraph();

        PathResult res;
        res.amx_ = task.amx_;
        res.amx_generation = task.amx_generation;
        res.npc_id = task.npc_id;
        res.callback = std::move(task.callback);
        res.success = false;

        uint32_t startFlat = 0xFFFFFFFF;
        uint32_t targetFlat = 0xFFFFFFFF;

        if (task.startAreaId >= 64 || task.targetAreaId >= 64) goto FINISH;
        if (task.startPointId >= graph->areaIdToFlatIndex[task.startAreaId].size() || 
            task.targetPointId >= graph->areaIdToFlatIndex[task.targetAreaId].size()) goto FINISH; 

        startFlat = graph->areaIdToFlatIndex[task.startAreaId][task.startPointId];
        targetFlat = graph->areaIdToFlatIndex[task.targetAreaId][task.targetPointId];

        if (startFlat == 0xFFFFFFFF || targetFlat == 0xFFFFFFFF) goto FINISH;
        if (graph->wccIds[startFlat] != graph->wccIds[targetFlat]) goto FINISH;
        if (graph->sccIds[startFlat] < graph->sccIds[targetFlat]) goto FINISH;
        if (OMP_UNLIKELY(grid.size() < graph->positions.size())) {
            grid.resize(graph->positions.size(), {0xFFFFFFFF, 0, 0.0f});
        }
        if (++currentGen == 0) {
            std::fill(grid.begin(), grid.end(), AStarCell{0xFFFFFFFF, 0, 0.0f});
            currentGen = 1;
        }

        {
            const Vector3* positions_ptr = graph->positions.data();
            const NodeLinkInfo* info_ptr = graph->nodeLinkInfo.data();
            const AStarPrecomputedLink* links_ptr = graph->links.data();
            AStarCell* grid_ptr = grid.data();

            auto getCell = [&](uint32_t flatId) -> AStarCell& {
                AStarCell& cell = grid_ptr[flatId];
                if (OMP_UNLIKELY(cell.generation != currentGen)) {
                    cell.gScore = std::numeric_limits<float>::infinity();
                    cell.cameFrom = 0xFFFFFFFF;
                    cell.generation = currentGen;
                }
                return cell;
            };

            const Vector3& targetPos = positions_ptr[targetFlat];
            const Vector3& startPos = positions_ptr[startFlat];
            getCell(startFlat).gScore = 0.0f;
            
            openSet.clear();
            float initial_h = SmartHeuristic(startPos, targetPos, startPos);
            openSet.push_back({initial_h, 0.0f, startFlat});
            std::make_heap(openSet.begin(), openSet.end(), std::greater<AStarQueueItem>());

            uint32_t iterations = 0;

            while (!openSet.empty()) {
                if (OMP_UNLIKELY(++iterations > MAX_ITERATIONS)) break;

                std::pop_heap(openSet.begin(), openSet.end(), std::greater<AStarQueueItem>());
                AStarQueueItem currentItem = openSet.back();
                openSet.pop_back();
                
                if (currentItem.gScore > getCell(currentItem.flatIndex).gScore) continue;

                uint32_t currentFlat = currentItem.flatIndex;

                if (currentFlat == targetFlat) {
                    //trace back route in reverse sequence
                    std::vector<uint32_t> reversePath;
                    uint32_t curr = targetFlat;
                    while (curr != startFlat) {
                        reversePath.push_back(curr);
                        curr = grid_ptr[curr].cameFrom;
                        if (curr == 0xFFFFFFFF) break;
                    }
                    reversePath.push_back(startFlat);

                    res.points.reserve(reversePath.size());
                    for (auto it = reversePath.rbegin(); it != reversePath.rend(); ++it) {
                        res.points.push_back(positions_ptr[*it]);
                    }
                    res.success = true;
                    break;
                }

                float gScore = getCell(currentFlat).gScore;
                const NodeLinkInfo& currInfo = info_ptr[currentFlat];
                
                for (uint32_t i = 0; i < currInfo.count; ++i) {
                    const AStarPrecomputedLink& link = links_ptr[currInfo.offset + i];
                    float tentative_gScore = gScore + link.cost;

                    AStarCell& neighborCell = getCell(link.targetIndex);
                    if (tentative_gScore < neighborCell.gScore) {
                        neighborCell.cameFrom = currentFlat;
                        neighborCell.gScore = tentative_gScore;
                        
                        float fScore = tentative_gScore + SmartHeuristic(positions_ptr[link.targetIndex], targetPos, startPos);
                        
                        openSet.push_back({fScore, tentative_gScore, link.targetIndex});
                        std::push_heap(openSet.begin(), openSet.end(), std::greater<AStarQueueItem>());
                    }
                }
            }
        }

    FINISH:
        {
            std::lock_guard<SpinLock> resultLock(resultSpinLock_); 
            results_.push(std::move(res));
        }
        activeWorkers_.fetch_sub(1, std::memory_order_relaxed);
    }
}

void PathfinderComponent::onLoad(ICore* c) {
    core_ = c;
    core_->printLn("open.mp A* Pathfinder component loaded.");
    core_->getEventDispatcher().addEventHandler(this);
    setAmxLookups(core_);
}

void PathfinderComponent::onInit(IComponentList* components) {
    pawn_ = components->queryComponent<IPawnComponent>();
    npcs_ = components->queryComponent<INPCComponent>();
    
    if (pawn_) {
        setAmxFunctions(pawn_->getAmxFunctions());
        setAmxLookups(components);
        pawn_->getEventDispatcher().addEventHandler(this);
    }

    if (npcs_ && core_) {
        core_->printLn("[omp-pathfinder] Fetching node data from assets.open.mp...");
        for (int i = 0; i < 64; ++i) npcs_->openNode(i);
    }

    LoadBinaryNodes();
    RebuildOptimizedGraph();

    size_t numWorkers = std::min<size_t>(4, std::max<size_t>(2, std::thread::hardware_concurrency() / 2));
    for (size_t i = 0; i < numWorkers; ++i) {
        workers_.emplace_back(&PathfinderComponent::workerLoop, this);
    }
}

void PathfinderComponent::onFree(IComponent* component) {
    if (component == pawn_) {
        pawn_ = nullptr;
        setAmxFunctions(); setAmxLookups();
    }
    if (component == npcs_) npcs_ = nullptr;
}

void PathfinderComponent::onAmxLoad(IPawnScript& script) {
    pawn_natives::AmxLoad(script.GetAMX());
    amxGenerations_[script.GetAMX()] = ++currentGeneration_;
}

void PathfinderComponent::onAmxUnload(IPawnScript& script) {
    AMX* unloadedAmx = script.GetAMX();
    amxGenerations_.erase(unloadedAmx);

    std::lock_guard<std::mutex> lock(taskMutex_);
    std::queue<PathTask> tempTasks;
    
    while (!tasks_.empty()) {
        PathTask t = std::move(tasks_.front());
        tasks_.pop();
        if (t.amx_ != unloadedAmx) {
            tempTasks.push(std::move(t));
        }
    }
    tasks_ = std::move(tempTasks);
}

bool PathfinderComponent::loadCustomNodesFromFile(const std::string& filename) {
    std::string fullPath = "scriptfiles/" + filename;
    std::ifstream file(fullPath);
    if (!file.is_open()) return false;

    struct TempNode {
        int area; 
        float x, y, z; 
    };
    struct TempLink {
        int area, node, tArea, tNode; 
    };
    std::vector<TempNode> tempNodes;
    std::vector<TempLink> tempLinks;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        std::istringstream iss(line);
        char type;
        if (!(iss >> type)) continue;

        if (type == 'N' || type == 'n') {
            TempNode n;
            if (iss >> n.area >> n.x >> n.y >> n.z) tempNodes.push_back(n);
        } else if (type == 'L' || type == 'l') {
            TempLink l;
            if (iss >> l.area >> l.node >> l.tArea >> l.tNode) tempLinks.push_back(l);
        }
    }

    if (tempNodes.empty() && tempLinks.empty()) return false;

    {
        std::lock_guard<std::mutex> lock(rawGraphMutex_);
        for (const auto& n : tempNodes) {
            if (n.area < 0 || n.area >= 64) continue;
            PathNode pn{};
            pn.positionX = static_cast<int16_t>(std::clamp(n.x * 8.0f, -32768.0f, 32767.0f));
            pn.positionY = static_cast<int16_t>(std::clamp(n.y * 8.0f, -32768.0f, 32767.0f));
            pn.positionZ = static_cast<int16_t>(std::clamp((n.z - 1.2f) * 8.0f, -32768.0f, 32767.0f));
            pn.areaId = static_cast<uint16_t>(n.area);
            pn.nodeId = static_cast<uint16_t>(nodeGraph_[n.area].size());
            nodeGraph_[n.area].push_back(pn);
        }

        for (const auto& l : tempLinks) {
            if (l.area < 0 || l.area >= 64 || l.tArea < 0 || l.tArea >= 64) continue;
            if (l.node >= nodeGraph_[l.area].size() || l.tNode >= nodeGraph_[l.tArea].size()) continue;
            customLinks_[l.area][static_cast<uint16_t>(l.node)].push_back({
                static_cast<uint16_t>(l.tArea), static_cast<uint16_t>(l.tNode)
            });
        }
    }

    RebuildOptimizedGraph();
    return true;
}

void PathfinderComponent::onTick(Microseconds elapsed, TimePoint now) {
    if (!pawn_ || !npcs_) return;

    std::queue<PathResult> batch;
    {
        std::lock_guard<SpinLock> lock(resultSpinLock_);
        if (results_.empty()) return; 
        batch.swap(results_);
    }

    while (!batch.empty()) {
        PathResult res = std::move(batch.front());
        batch.pop();

        auto it = amxGenerations_.find(res.amx_);
        if (it == amxGenerations_.end() || it->second != res.amx_generation) continue;

        int index;
        if (amx_FindPublic(res.amx_, res.callback.c_str(), &index) == AMX_ERR_NONE) {
            int pathId = -1;
            if (res.success && !res.points.empty()) {
                pathId = npcs_->createPath();
                for (const auto& pt : res.points) {
                    npcs_->addPointToPath(pathId, pt, 1.0f);
                }
            }
            
            amx_Push(res.amx_, res.success ? 1 : 0);
            amx_Push(res.amx_, pathId);
            amx_Push(res.amx_, res.npc_id);

            cell retval;
            amx_Exec(res.amx_, &retval, index);
        }
    }
}

bool PathfinderComponent::getNodePos(int areaId, int nodeId, float& x, float& y, float& z) {
    auto graph = getActiveGraph();
    if (!graph || areaId < 0 || areaId >= 64) return false;
    if (nodeId < 0 || nodeId >= graph->areaIdToFlatIndex[areaId].size()) return false;

    uint32_t flatId = graph->areaIdToFlatIndex[areaId][nodeId];
    if (flatId == 0xFFFFFFFF) return false;

    const Vector3& pos = graph->positions[flatId];
    x = pos.x; 
    y = pos.y; 
    z = pos.z;
    return true;
}

float PathfinderComponent::getDistanceBetweenNodes(int area1, int node1, int area2, int node2) {
    auto graph = getActiveGraph();
    if (!graph || area1 < 0 || area1 >= 64 || area2 < 0 || area2 >= 64) return -1.0f;
    
    if (node1 < 0 || node1 >= graph->areaIdToFlatIndex[area1].size() ||
        node2 < 0 || node2 >= graph->areaIdToFlatIndex[area2].size()) return -1.0f;

    uint32_t flatId1 = graph->areaIdToFlatIndex[area1][node1];
    uint32_t flatId2 = graph->areaIdToFlatIndex[area2][node2];

    if (flatId1 == 0xFFFFFFFF || flatId2 == 0xFFFFFFFF) return -1.0f;

    const Vector3& p1 = graph->positions[flatId1];
    const Vector3& p2 = graph->positions[flatId2];
    
    return glm::distance(p1, p2);
}

bool PathfinderComponent::getClosestNode(float x, float y, float z, int& outArea, int& outNode) {
    auto graph = getActiveGraph();
    if (!graph) return false;

    float minDistSq = std::numeric_limits<float>::max();
    int bestArea = -1;
    int bestNode = -1;

    for (int a = 0; a < 64; ++a) {
        const auto& nodesInArea = graph->areaIdToFlatIndex[a];
        for (size_t n = 0; n < nodesInArea.size(); ++n) {
            uint32_t flatId = nodesInArea[n];
            if (flatId == 0xFFFFFFFF) continue;

            const Vector3& p = graph->positions[flatId];
            float dx = p.x - x;
            float dy = p.y - y;
            float dz = p.z - z;
            
            float distSq = (dx * dx) + (dy * dy) + (dz * dz);

            if (distSq < minDistSq) {
                minDistSq = distSq;
                bestArea = a;
                bestNode = static_cast<int>(n);
            }
        }
    }

    if (bestArea != -1 && bestNode != -1) {
        outArea = bestArea;
        outNode = bestNode;
        return true;
    }
    
    return false;
}
