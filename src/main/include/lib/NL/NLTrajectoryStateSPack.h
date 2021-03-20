#pragma once
#include "../N/NType.h"
#include "../N/Containers/NArray.h"
//#include "NLPathGeometry.h"
#include "NLTrajectoryStateS.h"

#ifdef _NEDITOR
#include "../NL2DOrthogonalCoordinateSystem.h"
#endif

class NLTRAJECTORY_STATE_S_PACK
{
public:
	NLTRAJECTORY_STATE_S_PACK();
	~NLTRAJECTORY_STATE_S_PACK();

	Nu32	read(FILE *pfile);
	Nu32	write(FILE *pfile);

	NLTRAJECTORY_STATE_S*	getState(NLTRAJECTORY_STATE_S *pstate, const Nf32 t);

#ifdef _NEDITOR
	void drawTrajectoryStateSArray(NL2DOCS * p2docs);
#endif

	NARRAY					m_trajectoryStateSArray;
	Nf32					m_dt;	// Dur�e totale n�c�ssaire pour parcourir la trajectoire
	Nf32					m_ds;	// Distance totale couverte par la trajectoire
};
