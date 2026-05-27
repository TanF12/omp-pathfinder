#include "pathfinder-component.hpp"
#include <Server/Components/Pawn/Impl/pawn_natives.hpp>

// TODO: unfuck this API's syntax by removing this AStar prefix

//native bool:AStar_CalculatePath(npc_id, start_area, start_node, target_area, target_node, const callback[] = "OnPathCalculated");
SCRIPT_API(AStar_CalculatePath, bool(int npc_id, int start_area, int start_node, int target_area, int target_node, const std::string& callback)) {
    if (start_area < 0 || start_area > 63 || target_area < 0 || target_area > 63) return false; 

    PathTask task;
    task.amx_ = GetAMX();
    task.npc_id = npc_id;
    task.callback = callback.empty() ? "OnPathCalculated" : callback;
    task.startAreaId = static_cast<uint16_t>(start_area);
    task.startPointId = static_cast<uint16_t>(start_node);
    task.targetAreaId = static_cast<uint16_t>(target_area);
    task.targetPointId = static_cast<uint16_t>(target_node);

    if (auto comp = PathfinderComponent::getInstance()) {
        task.amx_generation = comp->getAmxGeneration(task.amx_);
        return comp->enqueueTask(std::move(task));
    }
    return false;
}

//native bool:AStar_IsBusy();
SCRIPT_API(AStar_IsBusy, bool()) {
    if (auto comp = PathfinderComponent::getInstance()) return comp->isBusy();
    return false;
}

//native bool:AStar_DestroyPath(path_id);
SCRIPT_API(AStar_DestroyPath, bool(int path_id)) {
    if (auto comp = PathfinderComponent::getInstance()) return comp->destroyPath(path_id);
    return false;
}

//native AStar_AddNode(area_id, Float:x, Float:y, Float:z);
SCRIPT_API(AStar_AddNode, int(int area_id, float x, float y, float z)) {
    if (auto comp = PathfinderComponent::getInstance()) return comp->addCustomNode(area_id, x, y, z);
    return -1;
}

//native bool:AStar_AddLink(area_id, node_id, target_area_id, target_node_id);
SCRIPT_API(AStar_AddLink, bool(int area_id, int node_id, int target_area_id, int target_node_id)) {
    if (auto comp = PathfinderComponent::getInstance()) return comp->addCustomLink(area_id, node_id, target_area_id, target_node_id);
    return false;
}

//native bool:AStar_LoadCustomNodes(const filename[]);
SCRIPT_API(AStar_LoadCustomNodes, bool(const std::string& filename)) {
    if (auto comp = PathfinderComponent::getInstance()) return comp->loadCustomNodesFromFile(filename);
    return false;
}

// native bool:AStar_GetNodePos(area_id, node_id, &Float:x, &Float:y, &Float:z);
SCRIPT_API(AStar_GetNodePos, bool(int area_id, int node_id, float& x, float& y, float& z)) {
    if (auto comp = PathfinderComponent::getInstance()) {
        return comp->getNodePos(area_id, node_id, x, y, z);
    }
    return false;
}

// native Float:AStar_GetDistanceBetweenNodes(area_id1, node_id1, area_id2, node_id2);
SCRIPT_API(AStar_GetDistanceBetweenNodes, float(int area_id1, int node_id1, int area_id2, int node_id2)) {
    if (auto comp = PathfinderComponent::getInstance()) {
        return comp->getDistanceBetweenNodes(area_id1, node_id1, area_id2, node_id2);
    }
    return -1.0f;
}

// native bool:AStar_GetClosestNode(Float:x, Float:y, Float:z, &area_id, &node_id);
SCRIPT_API(AStar_GetClosestNode, bool(float x, float y, float z, int& area_id, int& node_id)) {
    if (auto comp = PathfinderComponent::getInstance()) {
        return comp->getClosestNode(x, y, z, area_id, node_id);
    }
    return false;
}