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
void G_Q3P_AdvancePlannerBot(void)
{
	pBot->client->pers.cmd.forwardmove = 64;

	if(pBot)
	{
		pBot->q3p_advanceFrameNum++;
	}
}

/*!
 *	rIntBetween
 *	local convenience function for ranged random ints
 */
static int rIntBetween(const int low, const int high) 
{
	return low + (rand() % (int)(high - low + 1));
}




void G_Q3P_SelectControls(usercmd_t* out)
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

