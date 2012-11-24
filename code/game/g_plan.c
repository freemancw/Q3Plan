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
// RRT state graph data structures
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
	svertarray_t	states;
	svert_t			*lastSelected;	// last vertex selected for expansion
};

//============================================================================
// svertex_t and svertexarray_t support routines
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
 *	Returns pointer to new elt in array.
 */
static svert_t* addToSVertArray(svertarray_t * const sva, 
							const svert_t * const elt)
{
	if(sva->used == sva->size)
	{
		sva->size *= 2;
		sva->data = (svert_t*)realloc(sva->data, sva->size * sizeof(svert_t));
	}

	memcpy(sva->data + sva->used, elt, sizeof(svert_t));
	return sva->data + sva->used++;
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
// sedge_t and sedgearray_t support routines
//============================================================================

/*!
 *	constructSEdge
 *	Properly assigns/copies the given data into corresponding members.
 */
static void constructSEdge(sedge_t * const se, 
						   const usercmd_t * const controls, 
						   svert_t * const dst)
{
	memcpy(&(se->controls), controls, sizeof(usercmd_t));
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
 *	Returns pointer to new elt in the array.
 */
static sedge_t* addToSEdgeArray(sedgearray_t * const sea, 
								const sedge_t * const elt)
{
	if(sea->used == sea->size)
	{
		sea->size *= 2;
		sea->data = (sedge_t*)realloc(sea->data, sea->size * sizeof(sedge_t));
	}

	memcpy(sea->data + sea->used, elt, sizeof(sedge_t));
	return sea->data + sea->used++;
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
// sgraph_t support routines
//============================================================================

/*!
 *	initSGraph
 *	Allocates a chunk of memory for vertex storage and adds an initial vertex.
 */
static void initSGraph(sgraph_t * const sg, const svert_t * const initSv)
{
	initSVertArray(&(sg->states), 2048);
	sg->lastSelected = addToSVertArray(&(sg->states), initSv);
}

/*!
 *	addToSGraph
 *	Adds a new vertex to the graph. Responsible for constructing edges from
 *	from the expansion vertex to the inserted vertex.
 */
static void addToSGraph(sgraph_t * const sg, svert_t * const sv)
{
	sedge_t newEdge;

	constructSEdge(&newEdge, &(sv->client.pers.cmd), sv);
	addToSEdgeArray(&(sg->lastSelected->neighbors), &newEdge);
	addToSVertArray(&(sg->states), sv);
}

//============================================================================
// RRT main functions
//============================================================================

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
	addToSGraph(&rrtStateGraph, &currentState);
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
	svert_t initState;

	G_Printf("Running PlannerBot RRT\n");
	
	// construct the initial state from wherever the bot is currently
	constructSVert(&initState, pBot, pBot->client, NULL);

	// add initial state to graph
	initSGraph(&rrtStateGraph, &initState);
}
