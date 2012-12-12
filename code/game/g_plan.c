/*!
 *	@file g_plan.c
 *	@author Clinton Freeman
 */

#include "g_local.h"
#include <float.h>

//============================================================================
// Private RRT state tree data structures
//============================================================================

// forward declarations
typedef struct sedge_s sedge_t;
typedef struct sedgearray_s sedgearray_t;
typedef struct spath_s spath_t;
typedef struct snode_s snode_t;
typedef struct snodearray_s snodearray_t;
typedef struct stree_s stree_t;

struct sedge_s
{
    usercmd_t	controls;
    size_t		duration;	
    snode_t		*dst;
};

struct sedgearray_s
{
    sedge_t		*data;
    size_t		used;
    size_t		size;
};

struct spath_s
{
    size_t		nodeIdx;
    size_t		edgeIdx;
};

struct snode_s
{
    //! @todo replace with lightweight structures
    gentity_t		ent;
    gclient_t		client;

    // path to root data
    size_t			depth;		
    spath_t			path;

    sedgearray_t	neighbors;
};

struct snodearray_s
{
    snode_t		*data;
    size_t		used;
    size_t		size;
};

struct stree_s
{
    snodearray_t	states;
    size_t			expansionNodeIdx;
};

//============================================================================
// Private RRT state tree function declarations
//============================================================================

static void         constructSNode(snode_t * const sn, 
                                   const gentity_t * const ent, 
                                   const gclient_t * const client, 
                                   const size_t depth, const spath_t path,
                                   const sedgearray_t * const neighbors);

static void			initSNodeArray(snodearray_t * const sna, 
                                   const size_t initialSize);

static size_t		addToSNodeArray(snodearray_t * const sna, 
                                    const snode_t * const elt);

static void			freeSNodeArray(snodearray_t * const sna);

static void			constructSEdge(sedge_t * const se, 
                                   const usercmd_t * const controls, 
                                   const size_t duration, 
                                   snode_t * const dst);

static void			initSEdgeArray(sedgearray_t * const sea, 
                                   const size_t initialSize);

static size_t		addToSEdgeArray(sedgearray_t * const sea, 
                                    const sedge_t * const elt);

static void			freeSEdgeArray(sedgearray_t * const sea);

static void			initSTree(stree_t * const st, 
                              const snode_t * const root);

static void			addToSTree(stree_t * const st, snode_t * const sn);

static size_t		getNNIdxFromSTree(stree_t * const st, vec3_t bias);

static spath_t*		getMinPathToGoal(const stree_t * const st);

//============================================================================
// Private Convenience Function Declarations
//============================================================================

static int	rIntBetween(const int low, const int high);
static void printVec3(vec3_t out);

//============================================================================
// Public RRT Function Definitions
//============================================================================

struct
{
    stree_t		sTree;
    spath_t		*solutionPath;
    //size_t		solutionPathIdx;
    qboolean	bAlgorithmIsRunning;
    qboolean	bSolutionIsPlaying;
} 
static rrt;

static gentity_t *rrtBot;

size_t	G_Q3P_RRT_NumDebugFrames;	
size_t  G_Q3P_RRT_SolutionPathIdx;

static void printNode(snode_t *node)
{
    G_Printf("Depth: %d, Parent Node Idx: %d, Parent Node Edge Idx: %d, Number of Neighbors: %d\n", 
        node->depth, node->path.nodeIdx, node->path.edgeIdx, node->neighbors.used);
}

/*!
 *	G_Q3P_RRT_SpawnBot
 *	The first thing the user must do is spawn in a special AI bot.
 *	@todo maybe check for one already existing...
 */
void G_Q3P_RRT_SpawnBot()
{
    rrtBot = G_Q3P_AddPlannerBot();
}

/*!
 *	G_Q3P_RRT_InitTree
 *	The second thing the user must do is initialize the state tree with the
 *	RRT bot's current state. 
 *	@note this isn't combined with spawning the bot because it could take
 *		  several frames for information to be processed (I'm 100% not sure)
 */
void G_Q3P_RRT_InitTree()
{
    snode_t rootState;
    spath_t rootPath;

    G_Printf("Initializing RRT state tree.\n");
    
    // construct the initial state from wherever the bot is currently
    rootPath.nodeIdx = rootPath.edgeIdx = UINT_MAX;
    constructSNode(&rootState, rrtBot, rrtBot->client, 0, rootPath, NULL);

    //G_Printf("Constructed root node: ");
    //printNode(&rootState);

    // add initial state to tree
    initSTree(&(rrt.sTree), &rootState);

    //G_Printf("Added it to the tree.\n");
}

/*!
 *	G_Q3P_RRT_RunAlgorithm
 *	After the tree is initialized, the user can let the algorithm run until
 *	a new vertex lands inside of the goal region.
 */
void G_Q3P_RRT_RunAlgorithm()
{
    rrt.bAlgorithmIsRunning = qtrue;
}

/*!
 *	G_Q3P_RRT_RunDebugFrames
 *	After the tree is initialized, the user can manually step through a 
 *	specified number of frames.
 *	@todo better way of communicating than using global variable?
 */
void G_Q3P_RRT_RunDebugFrames(const size_t n)
{
    if(rrt.bAlgorithmIsRunning || rrt.bSolutionIsPlaying) return;

    G_Q3P_RRT_NumDebugFrames += n;
}

void G_Q3P_RRT_PauseAlgorithm()
{
    rrt.bAlgorithmIsRunning = qfalse;
}

void G_Q3P_RRT_PlaySolution()
{
    rrt.bSolutionIsPlaying = qtrue;
}

void G_Q3P_RRT_PauseSolution()
{
    rrt.bSolutionIsPlaying = qfalse;
}

/*!
 *	G_Q3P_RRT_PauseAlgorithm
 *	@todo flesh this out
 */
void G_Q3P_PauseAlgorithm()
{
    rrt.bAlgorithmIsRunning = qfalse;
}

/*!
 *	G_Q3P_RRT_RestoreNewExpansionState
 *	At the beginning of each bot AI frame, RRT bots have their state restored
 *	to an existing state in the tree, which is selected using a random bias
 *	and a nearest neighbor query against the tree. 
 */
void G_Q3P_RRT_RestoreStateForExpansion(void)
{
    vec3_t randomState;
    int stateContents;
    snode_t *closestNode;

    do
    {
        randomState[0] = (float)rIntBetween(-512,  512);
        randomState[1] = (float)rIntBetween(-256, 1088);
        randomState[2] = (float)rIntBetween( -64,  512);
        stateContents = trap_PointContents(randomState, -1);
    }
    while((stateContents & (CONTENTS_SOLID|CONTENTS_LAVA|CONTENTS_SLIME)));

    rrt.sTree.expansionNodeIdx = getNNIdxFromSTree(&(rrt.sTree), randomState);
    closestNode = rrt.sTree.states.data + rrt.sTree.expansionNodeIdx;

    //G_Printf("Expanding from node %d: ", rrt.sTree.expansionNodeIdx);
    //printNode(closestNode);

    // restore planner bot state
    Com_Memcpy(rrtBot->client, &(closestNode->client), sizeof(gclient_t));
    Com_Memcpy(rrtBot, &(closestNode->ent), sizeof(gentity_t));

    // restore times
    rrtBot->client->ps.commandTime = level.time;
    rrtBot->client->pers.cmd.serverTime = level.time;
}

/*!
 *	G_Q3P_RRT_AddNewState
 *	Adds a newly expanded state to the tree.
 */
void G_Q3P_RRT_AddNewState(void)
{
    snode_t newNode, *parentNode;
    spath_t path;
    static int temp = 0;

    parentNode = rrt.sTree.states.data + rrt.sTree.expansionNodeIdx;
    path.nodeIdx = rrt.sTree.expansionNodeIdx;
    path.edgeIdx = parentNode->neighbors.used;

    constructSNode(&newNode, rrtBot, rrtBot->client, parentNode->depth + 1,
                   path, NULL);

    //G_Printf("Constructed a new node: ");
    //printNode(&newNode);

    addToSTree(&(rrt.sTree), &newNode);
    trap_SendServerCommand(-1, va("addSNode %i %f %f %f", 
        rrt.sTree.states.used - 1, rrtBot->client->ps.origin[0], 
        rrtBot->client->ps.origin[1], rrtBot->client->ps.origin[2]));

    // check for stopping condition
    if(rrtBot->client->ps.origin[0] > -320.0f  &&
       rrtBot->client->ps.origin[0] < -192.0f  &&
       rrtBot->client->ps.origin[1] > -64.0f  &&
       rrtBot->client->ps.origin[1] < 64.0f &&
       rrtBot->client->ps.origin[2] > 0.0f  &&
       rrtBot->client->ps.origin[2] < 88.0f)
    {
        rrt.solutionPath = getMinPathToGoal(&(rrt.sTree));
        G_Q3P_RRT_SolutionPathIdx = 0;
        rrt.bAlgorithmIsRunning = qfalse;
        rrt.bSolutionIsPlaying = qtrue;

        //G_Printf("Restoring RRT Bot to init state\n");

        // restore planner bot state to init node
        Com_Memcpy(rrtBot->client, &(rrt.sTree.states.data[0].client), 
            sizeof(gclient_t));
        Com_Memcpy(rrtBot, &(rrt.sTree.states.data[0].ent), 
            sizeof(gentity_t));

        // restore times
        rrtBot->client->ps.commandTime = level.time;
        rrtBot->client->pers.cmd.serverTime = level.time;
        rrtBot->client->lastCmdTime = level.time - 50;
        rrtBot->client->ps.eFlags ^= EF_TELEPORT_BIT;
    }
}

void printControls(usercmd_t * cmd)
{
    G_Printf("Applying controls: %d, %d, %d, %d, %d, %d\n", cmd->forwardmove, 
        cmd->rightmove, cmd->upmove, cmd->angles[0], cmd->angles[1], cmd->angles[2]);
}

/*!
 *	G_Q3P_RRT_SelectControls
 *	If the algorithm is still running, controls are selected randomly after
 *	an expansion state is restored. If the solution is playing, controls are
 *	selected from the current edge on the solution path.
 */
usercmd_t savedControls;
void G_Q3P_RRT_SelectControls(usercmd_t * const out)
{
    spath_t *p;
    snode_t *node;
    usercmd_t *cmd;

    if(rrt.bAlgorithmIsRunning)
    {
        out->forwardmove	= savedControls.forwardmove		= rIntBetween(-127, 127);
        out->rightmove		= savedControls.rightmove		= rIntBetween(-127, 127);
        out->upmove			= savedControls.upmove			= rIntBetween(-127, 127);
        out->angles[0]		= savedControls.angles[0]		= ANGLE2SHORT(rIntBetween(0, 360));
        out->angles[1]		= savedControls.angles[1]		= ANGLE2SHORT(rIntBetween(0, 360));
        out->angles[2]		= savedControls.angles[2]		= ANGLE2SHORT(rIntBetween(0, 360));
    }
    else if(rrt.bSolutionIsPlaying)
    {
        p = rrt.solutionPath + G_Q3P_RRT_SolutionPathIdx;

        //G_Printf("Getting controls from solution path %d: parent idx %d, parent edge idx %d\n", 
        //	G_Q3P_RRT_SolutionPathIdx, p->nodeIdx, p->edgeIdx);

        if(G_Q3P_RRT_SolutionPathIdx && p->nodeIdx == UINT_MAX) 
        {
            rrt.bSolutionIsPlaying = qfalse;
            return;
        }

        trap_SendServerCommand(-1, va("colorSNode %i 255 0 0", p->nodeIdx));

        node = rrt.sTree.states.data + p->nodeIdx;
        cmd = &(node->neighbors.data[p->edgeIdx].controls);
        out->forwardmove	= cmd->forwardmove;
        out->rightmove		= cmd->rightmove;
        out->upmove			= cmd->upmove;
        out->angles[0]		= cmd->angles[0];
        out->angles[1]		= cmd->angles[1];
        out->angles[2]		= cmd->angles[2];
    }

    //printControls(out);
}

qboolean G_Q3P_RRT_AlgorithmIsRunning()
{
    return rrt.bAlgorithmIsRunning;
}

qboolean G_Q3P_RRT_SolutionIsPlaying()
{
    return rrt.bSolutionIsPlaying;
}

//============================================================================
// snode_t and snodearray_t support routines
//============================================================================

/*!
 *	constructSNode
 *	Properly assigns/copies the given data into corresponding members.
 */
static void constructSNode(snode_t * const sn, const gentity_t * const ent, 
                           const gclient_t * const client, const size_t depth,
                           const spath_t path, 
                           const sedgearray_t * const neighbors)
{
    sn->depth = depth;
    sn->path  = path;

    memcpy(&(sn->client), client, sizeof(gclient_t));
    memcpy(&(sn->ent), ent, sizeof(gentity_t));

    if(neighbors) 
        sn->neighbors = *neighbors;
    else
        initSEdgeArray(&(sn->neighbors), 16); //! @todo magic number
}

/*!
 *	initSNodeArray
 *	Allocates an initial chunk of memory for storage.
 */
static void initSNodeArray(snodearray_t * const sna, const size_t initialSize)
{
    sna->data = (snode_t*)malloc(initialSize * sizeof(snode_t));
    sna->size = initialSize;
    sna->used = 0;
}

/*!
 *	addToSNodeArray
 *	Appends input snode_t to the current memory chunk, resizing if necessary.
 *	Returns idx for new elt in array.
 */
static size_t addToSNodeArray(snodearray_t * const sna, 
                              const snode_t * const elt)
{
    if(sna->used == sna->size)
    {
        sna->size *= 2;
        sna->data = (snode_t*)realloc(sna->data, sna->size * sizeof(snode_t));
    }

    memcpy(sna->data + sna->used, elt, sizeof(snode_t));
    return sna->used++;
}

/*!
 *	freeSNodeArray
 *	free()'s the memory chunk and zeroes the size members.
 */
static void freeSNodeArray(snodearray_t * const sna)
{
    free(sna->data);
    sna->data = NULL;
    sna->size = sna->used = 0;
}

//============================================================================
// sedge_t and sedgearray_t support routines
//============================================================================

/*!
 *	constructSEdge
 *	Properly assigns/copies the given data into corresponding members.
 */
static void constructSEdge(sedge_t * const se, 
                           const usercmd_t * const controls, 
                           const size_t duration, snode_t * const dst)
{
    memcpy(&(se->controls), controls, sizeof(usercmd_t));
    se->duration = duration;
    se->dst = dst;
}

/*!
 *	initSEdgeArray
 *	Allocates an initial chunk of memory for storage.
 */
static void initSEdgeArray(sedgearray_t * const sea, const size_t initialSize)
{
    sea->data = (sedge_t*)malloc(initialSize * sizeof(sedge_t));
    sea->size = initialSize;
    sea->used = 0;
}

/*!
 *	addToSEdgeArray
 *	Appends input sedge_t to the current memory chunk, resizing if necessary.
 *	Returns index to new elt in the array.
 */
static size_t addToSEdgeArray(sedgearray_t * const sea, 
                              const sedge_t * const elt)
{
    if(sea->used == sea->size)
    {
        sea->size *= 2;
        sea->data = (sedge_t*)realloc(sea->data, sea->size * sizeof(sedge_t));
    }

    memcpy(sea->data + sea->used, elt, sizeof(sedge_t));
    return sea->used++;
}

/*!
 *	freeSEdgeArray
 *	free()'s the memory chunk and zeroes the size members.
 */
static void freeSEdgeArray(sedgearray_t * const sea)
{
    free(sea->data);
    sea->data = NULL;
    sea->size = sea->used = 0;
}

//============================================================================
// stree_t support routines
//============================================================================

/*!
 *	initSTree
 *	Allocates a chunk of memory for vertex storage and adds a root node.
 */
static void initSTree(stree_t * const st, const snode_t * const root)
{
    initSNodeArray(&(st->states), 2048);
    st->expansionNodeIdx = addToSNodeArray(&(st->states), root);
}

/*!
 *	addToSTree
 *	Adds a new node to the tree. Responsible for constructing edges from
 *	from the expansion node to the inserted node.
 */
static void addToSTree(stree_t * const st, snode_t * const sn)
{
    sedge_t newEdge;

    savedControls.serverTime = sn->client.pers.cmd.serverTime;
    constructSEdge(&newEdge, &savedControls, 1, sn);
    //constructSEdge(&newEdge, &(sn->client.pers.cmd), 1, sn);
    addToSEdgeArray(&(st->states.data[st->expansionNodeIdx].neighbors), 
                    &newEdge);
    addToSNodeArray(&(st->states), sn);
}

/*!
 *	getNNIdxFromSTree
 *	Currently takes as input a random point bias and returns the Euclidean
 *	nearest neighbor using linear search.
 */
static size_t getNNIdxFromSTree(stree_t * const st, vec3_t bias)
{
    int biasAreaNum, sAreaNum, sTTime, minTTime;
    size_t i, nnIdx;
    float minSqDist, sqDistToRandom;
    vec3_t vecToRandom;

    biasAreaNum = trap_AAS_PointAreaNum(bias);
    // find state in graph closest to bias using Euclidean distance
    minSqDist = FLT_MAX;
    minTTime = INT_MAX;
    for(i = 0; i < st->states.used; i++)
    {
        sAreaNum = trap_AAS_PointAreaNum(st->states.data[i].client.ps.origin);
        sTTime = trap_AAS_AreaTravelTimeToGoalArea(sAreaNum, 
            st->states.data[i].client.ps.origin, biasAreaNum, 0);
        VectorSubtract(st->states.data[i].client.ps.origin, bias, vecToRandom);
        sqDistToRandom = DotProduct(vecToRandom, vecToRandom);

        if(sTTime < minTTime)
        {
            minTTime = sTTime;
            minSqDist = sqDistToRandom;
            nnIdx = i;
        }
        else if (sTTime == minTTime)
        {
            if(sqDistToRandom < minSqDist)
            {
                minSqDist = sqDistToRandom;
                nnIdx = i;
            }
        }
    }

    return nnIdx;
}

/*!
 *	getMinPathToGoal
 *	@note Client responsible for freeing memory
 */
static spath_t* getMinPathToGoal(const stree_t * const st)
{
    size_t i;
    spath_t *minPath, endMarker;
    snode_t *curNode, *rootNode;

    endMarker.nodeIdx = endMarker.edgeIdx = UINT_MAX;

    curNode  = st->states.data + st->states.used - 1; 
    rootNode = st->states.data;
    minPath  = (spath_t*)malloc((curNode->depth + 1) * sizeof(spath_t));
    i		 = curNode->depth;

    G_Printf("^5Solution found. Traversal time = ^1 %d ms.\n", curNode->depth * 50);  

    minPath[i--] = endMarker;
    while(curNode != rootNode)
    {
        //G_Printf("%d curNode: ", i);
        //printNode(curNode);
        minPath[i--] = curNode->path;
        curNode = st->states.data + curNode->path.nodeIdx;
    }

    return minPath;
}

//============================================================================
// Convenience Function Definitions
//============================================================================

static int rIntBetween(const int low, const int high) 
{
    return low + (rand() % (int)(high - low + 1));
}

static void printVec3(vec3_t out)
{
    G_Printf("x %f, y %f, z %f\n", out[0], out[1], out[2]);
}


