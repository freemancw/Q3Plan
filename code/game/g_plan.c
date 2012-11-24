/*!
 *	Motion Planning
 *	@author Clinton Freeman
 */

#include "g_local.h"
#include <float.h>

static gentity_t *pBot;
static int rIntBetween(const int low, const int high);

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
	//Com_Memset(&savedState, 0, sizeof(gentity_t));
	//Com_Memset(&savedClient, 0, sizeof(struct gclient_s));

	Com_Memcpy(&savedClient, pBot->client, sizeof(struct gclient_s));
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

/*!
 *	rIntBetween
 *	local convenience function for ranged random ints
 */
static int rIntBetween(const int low, const int high) 
{
	return low + (rand() % (int)(high - low + 1));
}

//============================================================================
// Graph data structures
//============================================================================

// forward declarations
typedef struct svert_s svert_t;
typedef struct svertarray_s svertarray_t;
typedef struct sedge_s sedge_t;
typedef struct sedgearray_s sedgearray_t;
typedef struct sgraph_s sgraph_t;

struct sedge_s
{
	usercmd_t	controls;
	svert_t		*dst;
};

struct sedgearray_s
{
	sedge_t		*data;
	size_t		used;
	size_t		size;
};

struct svert_s
{
	//! @todo replace with lightweight structures
	gentity_t		ent;
	gclient_t		client;

	sedgearray_t	neighbors;
};

struct svertarray_s
{
	svert_t		*data;
	size_t		used;
	size_t		size;
};

struct sgraph_s
{
	svertarray_t states;
};

//============================================================================
// svertex_t support routines
//============================================================================

/*!
 *	constructSVert
 *	Properly assigns/copies the given data into corresponding members.
 */
static void constructSVert(svert_t * const sv, const gentity_t * const ent, 
						   const gclient_t * const client,
						   const sedgearray_t * const neighbors)
{
	memcpy(&(sv->client), client, sizeof(gclient_t));
	memcpy(&(sv->ent), ent, sizeof(gentity_t));
	
	if(neighbors) 
		sv->neighbors = *neighbors;
}

/*!
 *	initSVertArray
 *	Allocates an initial chunk of memory for storage.
 */
static void initSVertArray(svertarray_t * const sva, const size_t initialSize)
{
	sva->data = (svert_t*)malloc(initialSize * sizeof(svert_t));
	sva->size = initialSize;
	sva->used = 0;
}

/*!
 *	addToSVertArray
 *	Appends input svert_t to the current memory chunk, resizing if necessary.
 */
static void addToSVertArray(svertarray_t * const sva, 
							const svert_t * const elt)
{
	if(sva->used == sva->size)
	{
		sva->size *= 2;
		sva->data = (svert_t*)realloc(sva->data, sva->size * sizeof(svert_t));
	}

	memcpy(&sva->data[sva->used++], elt, sizeof(svert_t));
}

/*!
 *	freeSVertArray
 *	free()'s the memory chunk and zeroes the size members.
 */
static void freeSVertArray(svertarray_t * const sva)
{
	free(sva->data);
	sva->data = NULL;
	sva->size = sva->used = 0;
}

//============================================================================
// svertex_t support routines
//============================================================================

static void initSGraph(sgraph_t *sg, const size_t stateArraySize)
{
	initSArray(&(sg->states), stateArraySize);
}

static void insertSVert(sgraph_t *sg, svert_t *sv)
{
	addToSArray(&(sg->states), sv);
}


static sgraph_t rrtStateGraph;
static int lastState;
static int rrtNumVerts;

/*!
 *	G_Q3P_RRTSelectVertex
 */
void G_Q3P_RRTSelectVertex(void)
{
	int minIdx, i;
	float minSqDist, sqDistToRandom;
	vec3_t randomState, vecToRandom;

	// generate a random "state"
	randomState[0] = (float)rIntBetween(-512,  512);
	randomState[1] = (float)rIntBetween(-256, 1088);
	randomState[2] = (float)rIntBetween( -64,  512);

	// find state in graph closest to random state in the 
	// Euclidean sense
	minSqDist = FLT_MAX;
	for(i = 0; i < rrtStateGraph.states.used; i++)
	{
		VectorSubtract(rrtStateGraph.states.data[i].client.ps.origin,
			randomState, vecToRandom);
		sqDistToRandom = DotProduct(vecToRandom, vecToRandom);

		if(sqDistToRandom < minSqDist)
		{
			minSqDist = sqDistToRandom;	
			minIdx = i;
		}
	}

	// restore planner bot state
	Com_Memcpy(pBot->client, &(rrtStateGraph.states.data[minIdx].client), 
		sizeof(gclient_t));

	Com_Memcpy(pBot, &(rrtStateGraph.states.data[minIdx].ent), 
		sizeof(gentity_t));

	lastState = minIdx;
}

/*!
 *	G_Q3P_RRTAddVertex
 */
void G_Q3P_RRTAddVertex(void)
{
	svert_t currentState;

	constructSVert(&currentState, pBot, pBot->client, NULL);
	insertSVert(&rrtStateGraph, &currentState);
	rrtNumVerts--;

	if(pBot->client->ps.origin[0] > 336.0f  &&
	   pBot->client->ps.origin[0] < 432.0f  &&
	   pBot->client->ps.origin[1] > 912.0f  &&
	   pBot->client->ps.origin[1] < 1008.0f &&
	   pBot->client->ps.origin[2] > 256.0f  &&
	   pBot->client->ps.origin[2] < 344.0f)
	{

	}
}

qboolean G_Q3P_RRTIsRunning(void)
{
	if(rrtNumVerts > 0) return qtrue; else return qfalse;
}

/*!
 *	G_Q3P_RunPlannerBotRRT
 *
 *	For the first demo, the goal is fixed as a rocket launcher up
 *	on a ledge. In particular, the bot reaches the goal region if its
 *	origin is contained within the minkowski sum of the bot's AABB with the
 *	RL's BBox.
 *
 *	Leftmost X:		336
 *	Rightmost X:	432
 *	Bottommost Y:	912
 *	Topmost Y:		1008
 *	Bottommost Z:	256
 *	Topmost Z:		344
 *
 *	As a first attempt at RRT, I'm just going to randomly generate coordinates
 *	within the level's AABB and use these to bias the search by performing
 *	Euclidean nearest neighbor to find the appropriate vertex. 
 */
void G_Q3P_RunPlannerBotRRT(void)
{
	int i, minIdx;
	float distToRandom, minDist;
	svert_t stateInit;
	svert_t stateGoal;
	vec3_t randomState, vecToRandom;

	int areaNum;

	G_Printf("Running PlannerBot RRT\n");

	// init the graph with 512 state slots
	initSGraph(&rrtStateGraph, 512);

	// for now, let's just add 2048 states to the graph and see what happens
	rrtNumVerts = 2048;
	
	// construct the initial state from wherever the bot is currently
	constructSVert(&stateInit, pBot, pBot->client, NULL);

	// add initial state to graph
	insertSVert(&rrtStateGraph, &stateInit);

	//areaNum = trap_AAS_PointAreaNum(pBot->client->ps.origin);
	//G_Printf("%d\n", areaNum);
}

//============================================================================
// KPIECE data structures
//============================================================================

typedef struct
{
	gentity_t ent;
	gclient_t client;
} 
kstate_t;

typedef struct
{
	kstate_t	startState;
	usercmd_t	controls;
	size_t		duration; // number of frames to run
}
kmotion_t;

typedef struct
{
	// contains array of motions
	float coverage;
	size_t selections;
	float score;
	size_t iteration;
	float importance;
}
kcelldata_t;

typedef struct
{
	// integer array
	int x, y, z;
}
kcoord_t;

typedef struct
{
	// contains celldata array, this isn't correct
	kcelldata_t cellData;
	size_t	dimension;
	size_t	maxNeighbors;
}
kgrid_t;

typedef struct
{
	kgrid_t grid;
	kcelldata_t *recentCell;
	size_t	size;
	size_t	iteration;
	float selectBorderFraction;
}
kdiscretization_t;

/*!
 *	G_Q3P_RunPlannerBotKPIECE
 */
void G_Q3P_RunPlannerBotKPIECE(void)
{

}
