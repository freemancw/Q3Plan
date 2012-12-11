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

static void			constructSNode(snode_t * const sn, 
								   const gentity_t * const ent, 
								   const gclient_t * const client, 
								   const size_t depth,
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
	qboolean	isRunning;
	qboolean	isPlayingSolution;
} 
static rrt;
static gentity_t *pBot;

size_t	rrtDebugFrames;	

/*!
 *	G_Q3P_RRTSelectVertex
 */
void G_Q3P_RRTSelectVertex(void)
{
	vec3_t randomState;
	int stateContents;
	snode_t *closestVertex;

	do
	{
		// generate a "random state"
		randomState[0] = (float)rIntBetween(-512,  512);
		randomState[1] = (float)rIntBetween(-256, 1088);
		randomState[2] = (float)rIntBetween( -64,  512);
		stateContents = trap_PointContents(randomState, -1);
	}
	while((stateContents & (CONTENTS_SOLID|CONTENTS_LAVA|CONTENTS_SLIME)));

	closestVertex = rrt.sTree.states.data + 
					getNNIdxFromSTree(&(rrt.sTree), randomState);

	// restore planner bot state
	Com_Memcpy(pBot->client, &(closestVertex->client), sizeof(gclient_t));
	Com_Memcpy(pBot, &(closestVertex->ent), sizeof(gentity_t));

	// restore times
	pBot->client->ps.commandTime = level.time;
	pBot->client->pers.cmd.serverTime = level.time;

	// for presentation purposes
	VectorCopy(pBot->client->ps.origin, pBot->s.origin); 
}

/*!
 *	G_Q3P_RRTAddNode
 */
void G_Q3P_RRTAddNode(void)
{
	snode_t currentState, *parentState;
	static int temp = 0;

	parentState = rrt.sGraph.states.data + rrt.sGraph.lastSelectedIdx;
	constructSVert(&currentState, pBot, pBot->client, parentState->depth + 1,
				   rrt.sGraph.lastSelectedIdx, NULL);
	addToSTree(&(rrt.sGraph), &currentState);

	VectorCopy(pBot->client->ps.origin, pBot->s.origin2);
	G_AddEvent(pBot, EV_VIZ_RRT, 0);

	if(!rrt.isRunning) return;

	if(temp++ == 64)
	{
		rrt.isRunning = qfalse;
		rrt.isPlayingSolution = qtrue;
		rrt.solutionEdges = shortestPathToGoal(&(rrt.sGraph));
		rrt.solutionEdgePtr = rrt.solutionEdges;

		// restore planner bot state to init vertex
		Com_Memcpy(pBot->client, &(rrt.sGraph.states.data[0].client), 
			sizeof(gclient_t));
		Com_Memcpy(pBot, &(rrt.sGraph.states.data[0].ent), 
			sizeof(gentity_t));

		// restore times
		pBot->client->ps.commandTime = level.time;
		pBot->client->pers.cmd.serverTime = level.time;

		// for presentation purposes
		VectorCopy(pBot->client->ps.origin, pBot->s.origin); 
	}
}

/*!
 *	G_Q3P_RRTRunDebugFrames
 */
void G_Q3P_RRTRunDebugFrames(const size_t n)
{
	if(rrt.isRunning) return;

	rrtDebugFrames += n;
}

/*!
 *	G_Q3P_RRTIsRunning
 */
qboolean G_Q3P_RRTIsRunning(void)
{
	return rrt.isRunning;
}

/*!
 *	G_Q3P_RRTIsPlayingSolution
 */
qboolean G_Q3P_RRTIsPlayingSolution(void)
{
	return rrt.isPlayingSolution;
}

/*!
 *	G_Q3P_RRTStartAlgorithm
 */
void G_Q3P_RRTStartAlgorithm(qboolean debug)
{
	snode_t rootState;

	G_Printf("PlannerBot RRT Initialized\n");
	
	// construct the initial state from wherever the bot is currently
	constructSNode(&rootState, pBot, pBot->client, 0, NULL);

	// add initial state to tree
	initSTree(&(rrt.sTree), &rootState);

	if(!debug)
		rrt.isRunning = qtrue;
	else
		rrt.isRunning = qfalse;
}

/*!
 *	G_Q3P_RRTRestoreEdgeControls
 */
void G_Q3P_RRTRestoreEdgeControls(usercmd_t *out)
{
	memcpy(out, &(*rrt.solutionEdgePtr++)->controls, sizeof(usercmd_t));
	if(!rrt.solutionEdgePtr) rrt.isPlayingSolution = qfalse;
}

/*!
 *	G_Q3P_RRTAdvanceSolution
 */
void G_Q3P_RRTAdvanceSolution(void) 
{
	
}

/*!
 *	G_Q3P_SpawnPlannerBot
 */
void G_Q3P_SpawnPlannerBot(void)
{
	pBot = G_Q3P_AddPlannerBot();
}

/*!
 *	G_Q3P_AdvancePlannerBot
 */
void G_Q3P_AdvancePlannerBot(const int n)
{
	if(pBot)
	{
		//pBot->q3p_advanceFrameNum += n;
	}
}

/*!
 *	G_Q3P_SelectRandomControls
 */
void G_Q3P_SelectRandomControls(usercmd_t *out)
{
	out->forwardmove	= rIntBetween(-127, 127);
	out->rightmove		= rIntBetween(-127, 127);
	out->upmove			= rIntBetween(-127, 127);
	out->angles[0]		= ANGLE2SHORT(rIntBetween(0, 360));
	out->angles[1]		= ANGLE2SHORT(rIntBetween(0, 360));
	out->angles[2]		= ANGLE2SHORT(rIntBetween(0, 360));
}

/*!
 *	G_Q3P_SavePlannerBotState
 */
static gentity_t savedState;
static struct gclient_s savedClient;
void G_Q3P_SavePlannerBotState(void)
{
	Com_Memcpy(&savedClient, pBot->client, sizeof(gclient_t));
	Com_Memcpy(&savedState, pBot, sizeof(gentity_t));
}

/*!
 *	G_Q3P_RestorePlannerBotState
 */
void G_Q3P_RestorePlannerBotState(void)
{
	Com_Memcpy(pBot->client, &savedClient, sizeof(struct gclient_s));
	Com_Memcpy(pBot, &savedState, sizeof(gentity_t));
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
						   const sedgearray_t * const neighbors)
{
	sn->depth = depth;

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

	constructSEdge(&newEdge, &(sn->client.pers.cmd), 1, sn);
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

	st->expansionNodeIdx = nnIdx;
	return nnIdx;
}

/*!
 *	getMinPathToGoal
 *	@note Client responsible for freeing memory
 */
static spath_t* getMinPathToGoal(const stree_t * const st)
{

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


