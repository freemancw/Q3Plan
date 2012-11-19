/*!
 *	Motion Planning
 *	@author Clinton Freeman
 */

#include "g_local.h"

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
		pBot->q3p_advanceFrameNum += n;
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

typedef struct svert_s svert_t;
typedef struct svertarray_s svertarray_t;
typedef struct sedge_s sedge_t;
typedef struct sedgenode_s sedgenode_t;
typedef struct sgraph_s sgraph_t;

struct svert_s
{
	// these should be replaced with lightweight structures
	gentity_t ent;
	gclient_t client;

	// neighbor edge list
	sedgenode_t *neighbors;
};

struct svertarray_s
{
	svert_t		*data;
	size_t		used;
	size_t		size;
};

struct sedge_s
{
	usercmd_t controls;

	// edges are located in src, so no need to store
	svert_t *dst;
};

struct sedgenode_s
{
	sedge_t				sedge;
	struct sedgenode_s	*next;
};

struct sgraph_s
{
	svertarray_t	states;
};

static void initSArray(svertarray_t *sva, const size_t initialSize)
{
	sva->data = (svert_t*)malloc(initialSize * sizeof(svert_t));
	sva->size = initialSize;
	sva->used = 0;
}

static void addToSArray(svertarray_t *sva, svert_t *elt)
{
	if(sva->used == sva->size)
	{
		sva->size *= 2;
		sva->data = (svert_t*)realloc(sva->data, sva->size * sizeof(svert_t));
	}

	memcpy(&sva->data[sva->used++], elt, sizeof(svert_t));
}

static void freeSArray(svertarray_t *sva)
{
	free(sva->data);
	sva->data = NULL;
	sva->size = sva->used = 0;
}

static void initSGraph(sgraph_t *sg, const size_t stateArraySize)
{
	initSArray(&(sg->states), stateArraySize);
}

static void insertSVert(sgraph_t *sg, svert_t *sv)
{
	addToSArray(&(sg->states), sv);
}

static void createSVert(svert_t *sv, gentity_t *ent, gclient_t *client)
{
	memcpy(&(sv->client), client, sizeof(gclient_t));
	memcpy(&(sv->ent), ent, sizeof(gentity_t));
	sv->neighbors = NULL;
}

/*!
 *	G_Q3P_RunPlannerBotRRT
 */
void G_Q3P_RunPlannerBotRRT(void)
{
	sgraph_t stateGraph;
	svert_t stateInit;
	svert_t stateGoal;

	int areaNum;

	G_Printf("Running PlannerBot RRT\n");

	// init the graph with 512 state slots
	initSGraph(&stateGraph, 512);
	
	// create the initial state from wherever the bot is currently
	createSVert(&stateInit, pBot, pBot->client);

	// add initial state to graph
	insertSVert(&stateGraph, &stateInit);

	areaNum = trap_AAS_PointAreaNum(pBot->client->ps.origin);
	G_Printf("%d\n", areaNum);

	/*
	while(qtrue)
	{
		// generate a random state
	}
	*/
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
	// contains celldata, this isn't correct
	kcelldata_t cellData;
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
