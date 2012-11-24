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
	size_t		duration;	
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

	// shortest path data
	size_t			dist;		
	qboolean		visited;
	svert_t			*path;

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
	// defaults
	sv->dist = UINT_MAX;
	sv->visited = qfalse;
	sv->path = NULL;

	// parameter assignments
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
						   const size_t duration, svert_t * const dst)
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

	constructSEdge(&newEdge, &(sv->client.pers.cmd), 1, sv);
	addToSEdgeArray(&(sg->lastSelected->neighbors), &newEdge);
	addToSVertArray(&(sg->states), sv);
}

/*!
 *	selectNNFromSGraph
 *	Currently takes as input a random point bias and returns the Euclidean
 *	nearest neighbor using linear search.
 */
static svert_t* selectNNFromSGraph(sgraph_t * const sg, const vec3_t bias)
{
	size_t i, nnIdx;
	float minSqDist, sqDistToRandom;
	vec3_t vecToRandom;

	// find state in graph closest to bias using Euclidean distance
	minSqDist = FLT_MAX;
	for(i = 0; i < sg->states.used; i++)
	{
		VectorSubtract(sg->states.data[i].client.ps.origin, bias, vecToRandom);
		sqDistToRandom = DotProduct(vecToRandom, vecToRandom);

		if(sqDistToRandom < minSqDist)
		{
			minSqDist = sqDistToRandom;	
			nnIdx = i;
		}
	}

	sg->lastSelected = sg->states.data + nnIdx;
	return sg->states.data + nnIdx;
}

/*!
 *	shortestPathInSGraph
 *	Uses Dijkstra's algorithm to find the shortest path from the first vertex 
 *	to the goal vertex. Returns a sequence of edge indices which can be used 
 *	to navigate from the start state to the end state. The end of the index
 *	sequence is flagged with UINT_MAX.
 */
static size_t* shortestPathInSGraph(const sgraph_t * const sg)
{
	size_t i, minDist, numEdges, *eIndices, *eIdxPtr;
	svert_t *curNode;

	// distance from source to source
	sg->states.data[0].dist = 0;
	sg->states.data[0].visited = qfalse;

	// tentative distances for all other verts is "infinity"
	for(i = 1; i < sg->states.used; i++)
	{
		sg->states.data[i].dist = UINT_MAX;
		sg->states.data[i].visited = qfalse;
	}

	curNode = sg->states.data;
	while(!sg->states.data[sg->states.used - 1].visited)
	{
		for(i = 0; i < curNode->neighbors.used; i++)
		{
			if(curNode->neighbors.data[i].dst->visited) continue;

			if(curNode->dist + curNode->neighbors.data[i].duration <
			   curNode->neighbors.data[i].dst->dist)
			{
				curNode->neighbors.data[i].dst->dist = 
					curNode->dist + curNode->neighbors.data[i].duration;
				curNode->neighbors.data[i].dst->path = curNode;
			}
		}
		curNode->visited = qtrue;

		minDist = UINT_MAX;
		for(i = 0; i < sg->states.used; i++)
		{
			if(sg->states.data[i].visited) continue;

			if(sg->states.data[i].dist < minDist)
			{
				minDist = sg->states.data[i].dist;
				curNode = sg->states.data + i;
			}
		}
	}

	curNode = sg->states.data + sg->states.used - 1;
	while(curNode != sg->states.data)
	{
		curNode = curNode->path;
		numEdges++;
	}

	eIndices = (size_t*)malloc(sizeof(size_t) * (numEdges + 1));
	eIdxPtr = eIndices + numEdges;
	*eIdxPtr = UINT_MAX;
	eIdxPtr--;
	curNode = sg->states.data + sg->states.used - 1;
	while(curNode != sg->states.data)
	{
		for(i = 0; i < curNode->path->neighbors.used; i++)
		{
			if(curNode->path->neighbors.data[i].dst == curNode)
			{
				*eIdxPtr = i;
				break;
			}
		}
		curNode = curNode->path;
		eIdxPtr--;
	}

	return eIndices;
}

//============================================================================
// RRT main functions
//============================================================================

struct
{
	sgraph_t sGraph;
	qboolean isRunning;
} 
static rrt;

/*!
 *	G_Q3P_RRTSelectVertex
 */
void G_Q3P_RRTSelectVertex(void)
{
	vec3_t randomState;
	svert_t *closestVertex;

	// generate a "random state"
	randomState[0] = (float)rIntBetween(-512,  512);
	randomState[1] = (float)rIntBetween(-256, 1088);
	randomState[2] = (float)rIntBetween( -64,  512);

	closestVertex = selectNNFromSGraph(&(rrt.sGraph), randomState);

	// restore planner bot state
	Com_Memcpy(pBot->client, &(closestVertex->client), sizeof(gclient_t));
	Com_Memcpy(pBot, &(closestVertex->ent), sizeof(gentity_t));
}

/*!
 *	G_Q3P_RRTAddVertex
 */
void G_Q3P_RRTAddVertex(void)
{
	svert_t currentState;

	constructSVert(&currentState, pBot, pBot->client, NULL);
	addToSGraph(&(rrt.sGraph), &currentState);

	if(pBot->client->ps.origin[0] > 336.0f  &&
	   pBot->client->ps.origin[0] < 432.0f  &&
	   pBot->client->ps.origin[1] > 912.0f  &&
	   pBot->client->ps.origin[1] < 1008.0f &&
	   pBot->client->ps.origin[2] > 256.0f  &&
	   pBot->client->ps.origin[2] < 344.0f)
	{
		rrt.isRunning = qfalse;

	}
}

qboolean G_Q3P_RRTIsRunning(void)
{
	return rrt.isRunning;
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
	initSGraph(&(rrt.sGraph), &initState);
}

//============================================================================
// TEMPORARY WORKING AREA
//============================================================================

/*
struct gentity_lean_s 
{
	entityState_t	s;				// communicated by server to clients
	entityShared_t	r;				// shared by both the server system and game

	// DO NOT MODIFY ANYTHING ABOVE THIS, THE SERVER
	// EXPECTS THE FIELDS IN THAT ORDER!
	//================================

	struct gclient_s	*client;			// NULL if not a client

	qboolean	inuse;

	char		*classname;			// set in QuakeEd
	int			spawnflags;			// set in QuakeEd

	qboolean	neverFree;			// if true, FreeEntity will only unlink
									// bodyque uses this

	int			flags;				// FL_* variables

	char		*model;
	char		*model2;
	int			freetime;			// level.time when the object was freed
	
	int			eventTime;			// events will be cleared EVENT_VALID_MSEC after set
	qboolean	freeAfterEvent;
	qboolean	unlinkAfterEvent;

	qboolean	physicsObject;		// if true, it can be pushed by movers and fall off edges
									// all game items are physicsObjects, 
	float		physicsBounce;		// 1.0 = continuous bounce, 0.0 = no bounce
	int			clipmask;			// brushes with this content value will be collided against
									// when moving.  items and corpses do not collide against
									// players, for instance

	// freemancw - motion planning
	qboolean	q3p_isPlannerBot;		// is this client being used for motion planning?

	// movers
	moverState_t moverState;
	int			soundPos1;
	int			sound1to2;
	int			sound2to1;
	int			soundPos2;
	int			soundLoop;
	gentity_t	*parent;
	gentity_t	*nextTrain;
	gentity_t	*prevTrain;
	vec3_t		pos1, pos2;

	char		*message;

	int			timestamp;		// body queue sinking, etc

	char		*target;
	char		*targetname;
	char		*team;
	char		*targetShaderName;
	char		*targetShaderNewName;
	gentity_t	*target_ent;

	float		speed;
	vec3_t		movedir;

	int			nextthink;
	void		(*think)(gentity_t *self);
	void		(*reached)(gentity_t *self);	// movers call this when hitting endpoint
	void		(*blocked)(gentity_t *self, gentity_t *other);
	void		(*touch)(gentity_t *self, gentity_t *other, trace_t *trace);
	void		(*use)(gentity_t *self, gentity_t *other, gentity_t *activator);
	void		(*pain)(gentity_t *self, gentity_t *attacker, int damage);
	void		(*die)(gentity_t *self, gentity_t *inflictor, gentity_t *attacker, int damage, int mod);

	int			pain_debounce_time;
	int			fly_sound_debounce_time;	// wind tunnel
	int			last_move_time;

	int			health;

	qboolean	takedamage;

	int			damage;
	int			splashDamage;	// quad will increase this without increasing radius
	int			splashRadius;
	int			methodOfDeath;
	int			splashMethodOfDeath;

	int			count;

	gentity_t	*chain;
	gentity_t	*enemy;
	gentity_t	*activator;
	gentity_t	*teamchain;		// next entity in team
	gentity_t	*teammaster;	// master of the team

	int			watertype;
	int			waterlevel;

	int			noise_index;

	// timing variables
	float		wait;
	float		random;

	gitem_t		*item;			// for bonus items
};
*/