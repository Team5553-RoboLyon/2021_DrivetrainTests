#pragma once
#include "lib/N/NType.h"
//#include "lib/N/Maths/NVec2f32.h"
#include "lib/NL/NLKin.h"

typedef struct NLTRAJECTORY_STATE_S NLTRAJECTORY_STATE_S;
struct NLTRAJECTORY_STATE_S
{
	inline void set(const NLKIN *pkin, const Nf32 curvature)
	{
		m_localCurvature = curvature;
		m_kin = *pkin;
	}
	inline void null() { Nmem0(this, NLTRAJECTORY_STATE_S); }

	NLKIN m_kin;		   // kin repr�sentant le robot ( son centre d'inertie )
	Nf32 m_localCurvature; // courbure � l'abscisse curviligne m_kin.m_s
};
