#include <open.mp>
#include <colandreas>
#include <omp_pathfinder>

#define NPC_HEIGHT                  1.8
#define PED_Z_OFFSET                1.2
#define MAX_PATH_SEGMENT_LENGTH     4.0 
#define TERRAIN_MAX_Z_CHANGE        1.5
#define MAX_CLIMBABLE_OBSTACLE_Z    0.8

#define RAYCAST_STEP_UP             (MAX_CLIMBABLE_OBSTACLE_Z + 0.1) // 0.9m up
#define RAYCAST_STEP_DOWN           -2.0                             // 2.0m down
#define RAYCAST_BODY_MID_Z          (NPC_HEIGHT * 0.5)               // 0.9m height for wall checks

enum E_NPC_DATA {
    PathId_Direct,
    PathId_Start,
    PathId_AStar,
    
    Float:TargetX,
    Float:TargetY,
    Float:TargetZ,
    Float:TargetNodeX,
    Float:TargetNodeY,
    Float:TargetNodeZ,
    
    StartArea,
    StartNode
}

new gNPC[MAX_NPCS][E_NPC_DATA];
new gTestNPC = INVALID_NPC_ID;

main(){}

public OnGameModeInit()
{
    print("[NPC Pathfinding] Initialising optimal pathfinder...");
    
    CA_Init();

    for (new i = 0; i < MAX_NPCS; i++) 
    {
        gNPC[i][PathId_Direct] = INVALID_PATH_ID;
        gNPC[i][PathId_Start]  = INVALID_PATH_ID;
        gNPC[i][PathId_AStar]  = INVALID_PATH_ID;
    }

    gTestNPC = NPC_Create("testNPC");
    if (gTestNPC != INVALID_NPC_ID)
    {
        NPC_SetSkin(gTestNPC, 0);
        NPC_SetPos(gTestNPC, -1997.964, 187.519, 27.539);
        NPC_Spawn(gTestNPC);
        printf("[NPC Pathfinding] testNPC created successfully (ID: %d)", gTestNPC);
    } 
    return 1;
}

public OnGameModeExit()
{
    if (gTestNPC != INVALID_NPC_ID)
    {
        NPC_Destroy(gTestNPC);
        gTestNPC = INVALID_NPC_ID;
    }
    return 1;
}

stock ResetNPCPaths(npcid)
{
    if (gNPC[npcid][PathId_Direct] != INVALID_PATH_ID) { 
        NPC_DestroyPath(gNPC[npcid][PathId_Direct]); 
        gNPC[npcid][PathId_Direct] = INVALID_PATH_ID; 
    }
    if (gNPC[npcid][PathId_Start]  != INVALID_PATH_ID) { 
        NPC_DestroyPath(gNPC[npcid][PathId_Start]);  
        gNPC[npcid][PathId_Start]  = INVALID_PATH_ID; 
    }
    if (gNPC[npcid][PathId_AStar]  != INVALID_PATH_ID) { 
        NPC_DestroyPath(gNPC[npcid][PathId_AStar]);  
        gNPC[npcid][PathId_AStar]  = INVALID_PATH_ID; 
    }
}

stock Float:GetDistance3D(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2)
{
    return floatsqroot((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) + (z2 - z1)*(z2 - z1));
}

// Checks if a straight line is walkable using tunnel step checks
stock bool:IsPathWalkable(Float:startX, Float:startY, Float:startZ, Float:endX, Float:endY, Float:endZ)
{
    #pragma unused endZ
    new Float:dx = endX - startX;
    new Float:dy = endY - startY;
    new Float:dist2D = floatsqroot(dx*dx + dy*dy);
    
    if (dist2D < 0.5) return true; 

    new steps = floatround(dist2D / MAX_PATH_SEGMENT_LENGTH, floatround_ceil);
    new Float:hitX, Float:hitY, Float:hitZ;
    new Float:prevPx = startX;
    new Float:prevPy = startY;
    new Float:prevFloorZ;

    // Initial ground lock
    if (CA_RayCastLine(startX, startY, startZ + RAYCAST_STEP_UP, startX, startY, startZ + RAYCAST_STEP_DOWN, hitX, hitY, hitZ))
        prevFloorZ = hitZ;
    else 
        prevFloorZ = startZ - PED_Z_OFFSET;

    for (new i = 1; i <= steps; i++)
    {
        new Float:pct = float(i) / float(steps);
        new Float:px = startX + (dx * pct);
        new Float:py = startY + (dy * pct);
        new Float:currFloorZ;

        // Tunnel ground lookup
        if (CA_RayCastLine(px, py, prevFloorZ + RAYCAST_STEP_UP, px, py, prevFloorZ + RAYCAST_STEP_DOWN, hitX, hitY, hitZ))
            currFloorZ = hitZ;
        else 
            return false; // Dropped off map / cliff

        if (floatabs(currFloorZ - prevFloorZ) > TERRAIN_MAX_Z_CHANGE) return false; // Too steep

        // Body collision check (walls/fences)
        new Float:bodyHitZ;
        if (CA_RayCastLine(prevPx, prevPy, prevFloorZ + RAYCAST_BODY_MID_Z, px, py, currFloorZ + RAYCAST_BODY_MID_Z, hitX, hitY, bodyHitZ)) 
        {
            if (bodyHitZ - prevFloorZ > MAX_CLIMBABLE_OBSTACLE_Z) return false; // Wall is too high
        }

        prevPx = px;
        prevPy = py;
        prevFloorZ = currFloorZ;
    }
    return true; 
}

// Appends raycasted waypoints to bridge off-road gaps safely
stock AppendGroundHuggingPath(pathid, Float:startX, Float:startY, Float:startZ, Float:endX, Float:endY, Float:endZ)
{
    #pragma unused endZ
    new Float:dx = endX - startX;
    new Float:dy = endY - startY;
    new Float:dist2D = floatsqroot(dx*dx + dy*dy);
    
    new steps = floatround(dist2D / MAX_PATH_SEGMENT_LENGTH, floatround_ceil);
    if (steps < 1) steps = 1;

    new Float:hitX, Float:hitY, Float:hitZ, Float:currentFloorZ;

    if (CA_RayCastLine(startX, startY, startZ + RAYCAST_STEP_UP, startX, startY, startZ + RAYCAST_STEP_DOWN, hitX, hitY, hitZ))
        currentFloorZ = hitZ;
    else 
        currentFloorZ = startZ - PED_Z_OFFSET;

    NPC_AddPointToPath(pathid, startX, startY, currentFloorZ + PED_Z_OFFSET, 1.0);

    for (new i = 1; i <= steps; i++)
    {
        new Float:pct = float(i) / float(steps);
        new Float:px = startX + (dx * pct);
        new Float:py = startY + (dy * pct);
        
        if (CA_RayCastLine(px, py, currentFloorZ + RAYCAST_STEP_UP, px, py, currentFloorZ + RAYCAST_STEP_DOWN, hitX, hitY, hitZ))
            currentFloorZ = hitZ;
        
        NPC_AddPointToPath(pathid, px, py, currentFloorZ + PED_Z_OFFSET, 1.0);
    }
}

// Attempts a simplified V-shape bypass if straight line is blocked
stock bool:CalculateLocalBypass(pathid, Float:startX, Float:startY, Float:startZ, Float:endX, Float:endY, Float:endZ)
{
    new Float:dx = endX - startX;
    new Float:dy = endY - startY;
    new Float:dist = floatsqroot(dx*dx + dy*dy);
    if (dist < 1.0) return false;

    new Float:dirX = dx / dist;
    new Float:dirY = dy / dist;
    new Float:perpX = -dirY;
    new Float:perpY = dirX;

    new Float:offsets[] = { 6.0, 12.0 }; 
    new Float:midX = startX + (dx * 0.5);
    new Float:midY = startY + (dy * 0.5);
    
    for (new i = 0; i < sizeof(offsets); i++)
    {
        // Try Right side bypass
        new Float:wpX = midX + (perpX * offsets[i]);
        new Float:wpY = midY + (perpY * offsets[i]);
        new Float:hitX, Float:hitY, Float:hitZ, Float:wpZ;
        
        // offset lookup
        if (CA_RayCastLine(wpX, wpY, startZ + 3.0, wpX, wpY, startZ - 5.0, hitX, hitY, hitZ)) wpZ = hitZ;
        else wpZ = startZ;
        
        if (IsPathWalkable(startX, startY, startZ, wpX, wpY, wpZ) && IsPathWalkable(wpX, wpY, wpZ, endX, endY, endZ))
        {
            AppendGroundHuggingPath(pathid, startX, startY, startZ, wpX, wpY, wpZ);
            AppendGroundHuggingPath(pathid, wpX, wpY, wpZ, endX, endY, endZ);
            return true;
        }

        // Try Left side bypass
        wpX = midX - (perpX * offsets[i]);
        wpY = midY - (perpY * offsets[i]);
        
        if (CA_RayCastLine(wpX, wpY, startZ + 3.0, wpX, wpY, startZ - 5.0, hitX, hitY, hitZ)) wpZ = hitZ;
        else wpZ = startZ;
        
        if (IsPathWalkable(startX, startY, startZ, wpX, wpY, wpZ) && IsPathWalkable(wpX, wpY, wpZ, endX, endY, endZ))
        {
            AppendGroundHuggingPath(pathid, startX, startY, startZ, wpX, wpY, wpZ);
            AppendGroundHuggingPath(pathid, wpX, wpY, wpZ, endX, endY, endZ);
            return true;
        }
    }
    return false;
}

// A* state machine
stock NPC_GoByPathFinding(npcid, Float:targetX, Float:targetY, Float:targetZ)
{
    if (!NPC_IsValid(npcid)) return 0;

    new Float:npcX, Float:npcY, Float:npcZ;
    NPC_GetPos(npcid, npcX, npcY, npcZ);

    ResetNPCPaths(npcid);

    // If within 50m, skip the A* network entirely and try local micro-nav
    if (GetDistance3D(npcX, npcY, npcZ, targetX, targetY, targetZ) <= 50.0)
    {
        if (IsPathWalkable(npcX, npcY, npcZ, targetX, targetY, targetZ))
        {
            gNPC[npcid][PathId_Direct] = NPC_CreatePath();
            AppendGroundHuggingPath(gNPC[npcid][PathId_Direct], npcX, npcY, npcZ, targetX, targetY, targetZ);
            NPC_MoveByPath(npcid, gNPC[npcid][PathId_Direct]);
            return 1;
        }
        else 
        {
            gNPC[npcid][PathId_Direct] = NPC_CreatePath();
            if (CalculateLocalBypass(gNPC[npcid][PathId_Direct], npcX, npcY, npcZ, targetX, targetY, targetZ))
            {
                NPC_MoveByPath(npcid, gNPC[npcid][PathId_Direct]);
                return 1;
            }
            NPC_DestroyPath(gNPC[npcid][PathId_Direct]);
            gNPC[npcid][PathId_Direct] = INVALID_PATH_ID;
        }
    }
    
    // Macro Navigation: AStar
    new startArea, startNode, targetArea, targetNode;
    if (!AStar_GetClosestNode(npcX, npcY, npcZ, startArea, startNode)) return 0;
    if (!AStar_GetClosestNode(targetX, targetY, targetZ, targetArea, targetNode)) return 0;
    
    AStar_GetNodePos(targetArea, targetNode, gNPC[npcid][TargetNodeX], gNPC[npcid][TargetNodeY], gNPC[npcid][TargetNodeZ]);

    // Abort if the nearest road is on a vastly different Z-level (e.g., target is underground but nearest node is on a bridge)
    if (floatabs(gNPC[npcid][TargetNodeZ] - targetZ) > 6.0)
    {
        SendClientMessageToAll(-1, "[NPC Pathfinding] Cannot reach. Target is on a different vertical level (Bridge/Tunnel).");
        return 0;
    }

    gNPC[npcid][StartArea] = startArea;
    gNPC[npcid][StartNode] = startNode;
    gNPC[npcid][TargetX]   = targetX;
    gNPC[npcid][TargetY]   = targetY;
    gNPC[npcid][TargetZ]   = targetZ;

    AStar_CalculatePath(npcid, startArea, startNode, targetArea, targetNode, "OnPathCalculated");
    return 1;
}

forward OnPathCalculated(npcid, path_id, success);
public OnPathCalculated(npcid, path_id, success)
{
    if (!success || path_id == INVALID_PATH_ID)
    {
        SendClientMessageToAll(-1, "[NPC Pathfinding] A* calculation failed (no route).");
        ResetNPCPaths(npcid);
        return 1;
    }

    gNPC[npcid][PathId_AStar] = path_id;

    // Append the gap between the final A* node and the true target coordinates
    if (IsPathWalkable(gNPC[npcid][TargetNodeX], gNPC[npcid][TargetNodeY], gNPC[npcid][TargetNodeZ], gNPC[npcid][TargetX], gNPC[npcid][TargetY], gNPC[npcid][TargetZ]))
    {
        AppendGroundHuggingPath(path_id, gNPC[npcid][TargetNodeX], gNPC[npcid][TargetNodeY], gNPC[npcid][TargetNodeZ], gNPC[npcid][TargetX], gNPC[npcid][TargetY], gNPC[npcid][TargetZ]);
    }
    else if (!CalculateLocalBypass(path_id, gNPC[npcid][TargetNodeX], gNPC[npcid][TargetNodeY], gNPC[npcid][TargetNodeZ], gNPC[npcid][TargetX], gNPC[npcid][TargetY], gNPC[npcid][TargetZ]))
    {
        SendClientMessageToAll(-1, "[NPC Pathfinding] Target off-road is blocked. Stopping at nearest road.");
    }

    // MICRO-NAV: Walk from NPC's current position to the first A* node
    new Float:npcX, Float:npcY, Float:npcZ, Float:sNodeX, Float:sNodeY, Float:sNodeZ;
    NPC_GetPos(npcid, npcX, npcY, npcZ);
    AStar_GetNodePos(gNPC[npcid][StartArea], gNPC[npcid][StartNode], sNodeX, sNodeY, sNodeZ);
    
    if (GetDistance3D(npcX, npcY, npcZ, sNodeX, sNodeY, sNodeZ) > MAX_PATH_SEGMENT_LENGTH) 
    {
        gNPC[npcid][PathId_Start] = NPC_CreatePath();

        if (IsPathWalkable(npcX, npcY, npcZ, sNodeX, sNodeY, sNodeZ)) 
        {
            AppendGroundHuggingPath(gNPC[npcid][PathId_Start], npcX, npcY, npcZ, sNodeX, sNodeY, sNodeZ);
            NPC_MoveByPath(npcid, gNPC[npcid][PathId_Start]); // Trigger Start Path First
        } 
        else if (CalculateLocalBypass(gNPC[npcid][PathId_Start], npcX, npcY, npcZ, sNodeX, sNodeY, sNodeZ))
        {
            NPC_MoveByPath(npcid, gNPC[npcid][PathId_Start]); 
        }
        else 
        {
            SendClientMessageToAll(-1, "[NPC Pathfinding] Trapped: Cannot reach the street.");
            ResetNPCPaths(npcid);
        }
    } 
    else 
    {
        // NPC is already on the road, skip the start gap and execute A*
        NPC_MoveByPath(npcid, gNPC[npcid][PathId_AStar]);
    }
    
    return 1;
}

public OnNPCFinishMovePath(npcid, pathid)
{
    if (pathid == gNPC[npcid][PathId_Direct])
    {
        NPC_DestroyPath(pathid);
        gNPC[npcid][PathId_Direct] = INVALID_PATH_ID;
    }
    else if (pathid == gNPC[npcid][PathId_Start])
    {
        // Once the NPC finishes walking off-road to the road, start the A* network path
        NPC_DestroyPath(pathid);
        gNPC[npcid][PathId_Start] = INVALID_PATH_ID;
        
        if (gNPC[npcid][PathId_AStar] != INVALID_PATH_ID)
        {
            NPC_MoveByPath(npcid, gNPC[npcid][PathId_AStar]);
        }
    }
    else if (pathid == gNPC[npcid][PathId_AStar])
    {
        NPC_DestroyPath(pathid);
        gNPC[npcid][PathId_AStar] = INVALID_PATH_ID;
    }
    return 1;
}

public OnPlayerCommandText(playerid, cmdtext[])
{
    if (strcmp(cmdtext, "/npcgotome", true) == 0)
    {
        if (gTestNPC == INVALID_NPC_ID || !NPC_IsValid(gTestNPC)) return 1;

        new Float:px, Float:py, Float:pz;
        GetPlayerPos(playerid, px, py, pz);

        if (NPC_GoByPathFinding(gTestNPC, px, py, pz))
            SendClientMessage(playerid, -1, "NPC is calculating an optimal route...");
        
        return 1;
    }
    return 0;
}

public OnPlayerClickMap(playerid, Float:fX, Float:fY, Float:fZ)
{
    SetPlayerPos(playerid, fX, fY, fZ);
    return 1;
}
