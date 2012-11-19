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

