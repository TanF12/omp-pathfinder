#if defined _inc_omp_pathfinder
	#undef _inc_omp_pathfinder
#endif

#if defined _omp_pathfinder_included
	#endinput
#endif
#define _omp_pathfinder_included


native bool:AStar_CalculatePath(npc_id, start_area, start_node, target_area, target_node, const callback[] = "OnPathCalculated");
native bool:AStar_IsBusy();
native bool:AStar_DestroyPath(path_id);
native AStar_AddNode(area_id, Float:x, Float:y, Float:z);
native bool:AStar_AddLink(area_id, node_id, target_area_id, target_node_id);
native bool:AStar_LoadCustomNodes(const filename[]);
native bool:AStar_GetNodePos(area_id, node_id, &Float:x, &Float:y, &Float:z);
native Float:AStar_GetDistanceBetweenNodes(area_id1, node_id1, area_id2, node_id2);
native bool:AStar_GetClosestNode(Float:x, Float:y, Float:z, &area_id, &node_id);