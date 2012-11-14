/*!
 *	Motion Planning
 *	@author Clinton Freeman
 */

#include "g_local.h"

static gentity_t *pBot;

/*!
 *	G_Q3P_SpawnPlannerBot
 */
void G_Q3P_SpawnPlannerBot(void)
{
	pBot = G_Q3P_AddPlannerBot();
}





Q3P_State_t stateArray[1024];
int numStates = 0;

int rIntBetween(int min, int max) {
	return min + (rand() % (int)(max - min + 1));
}

Q3P_State_t* G_Q3P_SelectState( void )
{
	//return stateArray;
	return stateArray + rIntBetween(0, numStates - 1);
}

void G_Q3P_AddState( Q3P_State_t *newState )
{
	memcpy(stateArray + numStates, newState, sizeof(Q3P_State_t));
	numStates++;
}

void G_Q3P_SelectControls( usercmd_t* out )
{
	out->angles[0] = ANGLE2SHORT(rIntBetween(0, 360));
	out->angles[1] = ANGLE2SHORT(rIntBetween(0, 360));
	out->angles[2] = ANGLE2SHORT(rIntBetween(0, 360));

	out->forwardmove	= rIntBetween(-128, 127);
	out->rightmove		= rIntBetween(-128, 127);
	//out->upmove			= rIntBetween(-128, 127);
	out->upmove			= 0;

	out->serverTime = level.time;
}

void G_Q3P_CreateState( gentity_t* ent, Q3P_State_t* state )
{
	memcpy(&(state->gClient), ent->client, sizeof(struct gclient_s));
	memcpy(&(state->gState), ent, sizeof(gentity_t));
}

void G_Q3P_GenRandomSample( void ) 
{
	gentity_t *pBotEntity;

	G_Printf("G_Q3P_GenRandomSample\n");

	pBotEntity = G_AddPlannerBot( qtrue, 5000 );
	G_Q3P_CreateState( pBotEntity, stateArray );
	
	numStates++;
}

