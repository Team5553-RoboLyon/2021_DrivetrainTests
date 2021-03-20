#include "lib/NL/NLKin.h"
#include "lib/NL/NLTrajectoryStateSPack.h"
#include <iostream>

NLTRAJECTORY_STATE_S_PACK::NLTRAJECTORY_STATE_S_PACK()
{
	NSetupArray(&m_trajectoryStateSArray, 0, sizeof(NLTRAJECTORY_STATE_S));
}

NLTRAJECTORY_STATE_S_PACK::~NLTRAJECTORY_STATE_S_PACK()
{
	NClearArray(&m_trajectoryStateSArray, NULL);
}

Nu32 NLTRAJECTORY_STATE_S_PACK::read(FILE *pfile)
{
	// Pas de gestion de version pour le moment !
	// on ecrit � la main, direct, pour s'affranchir de la lib NFILE...
	// --> extrait de NFile:	NFileWrite(pfile, parray->pFirst, parray->ElementSize, parray->Size) != parray->Size
	// -->						(Nu32)fwrite(ptr, element_size_in_byte, element_count, pfile->pFile);
	Nu32 ret = 0;
	ret += (Nu32)fread(&m_ds, sizeof(Nf32), 1, pfile);
	ret += (Nu32)fread(&m_dt, sizeof(Nf32), 1, pfile);

	Nu32 capacity, elementsize, size;
	ret += (Nu32)fread(&capacity, sizeof(Nu32), 1, pfile);
	ret += (Nu32)fread(&elementsize, sizeof(Nu32), 1, pfile);
	ret += (Nu32)fread(&size, sizeof(Nu32), 1, pfile);

	NErrorIf(elementsize != m_trajectoryStateSArray.ElementSize, NERROR_INCONSISTENT_VALUES);
	if (size > m_trajectoryStateSArray.Capacity) // !! on se fiche de 'capacity' en fait !!!
	{
		std::cout << size << std::endl;

		m_trajectoryStateSArray.pFirst = (NBYTE *)realloc(m_trajectoryStateSArray.pFirst, size*m_trajectoryStateSArray.ElementSize);
		NErrorIf(!m_trajectoryStateSArray.pFirst, NERROR_NULL_POINTER);
		m_trajectoryStateSArray.Capacity = size;
	}

	ret += (Nu32)fread(m_trajectoryStateSArray.pFirst, m_trajectoryStateSArray.ElementSize, size, pfile);
	m_trajectoryStateSArray.Size = size;
	return ret;
}

Nu32 NLTRAJECTORY_STATE_S_PACK::write(FILE *pfile)
{
	// Pas de gestion de version pour le moment !
	// on ecrit � la main, direct, pour s'affranchir de la lib NFILE...
	// --> extrait de NFile:	NFileWrite(pfile, parray->pFirst, parray->ElementSize, parray->Size) != parray->Size
	// -->						(Nu32)fwrite(ptr, element_size_in_byte, element_count, pfile->pFile);
	Nu32 ret = 0;
	ret += (Nu32)fwrite(&m_ds, sizeof(Nf32), 1, pfile);
	ret += (Nu32)fwrite(&m_dt, sizeof(Nf32), 1, pfile);

	ret += (Nu32)fwrite(&m_trajectoryStateSArray.Capacity, sizeof(Nu32), 1, pfile);
	ret += (Nu32)fwrite(&m_trajectoryStateSArray.ElementSize, sizeof(Nu32), 1, pfile);
	ret += (Nu32)fwrite(&m_trajectoryStateSArray.Size, sizeof(Nu32), 1, pfile);
	ret += (Nu32)fwrite(m_trajectoryStateSArray.pFirst, m_trajectoryStateSArray.ElementSize, m_trajectoryStateSArray.Size, pfile);
	return ret;
}

NLTRAJECTORY_STATE_S *NLTRAJECTORY_STATE_S_PACK::getState(NLTRAJECTORY_STATE_S *pstate, const Nf32 t)
{
	if (t <= 0.0f)
	{
		pstate->null();
	}
	else if (t < m_dt)
	{
		NLTRAJECTORY_STATE_S *ps1 = (NLTRAJECTORY_STATE_S *)m_trajectoryStateSArray.pFirst;
		while (ps1->m_kin.m_t < t)
		{
			ps1++;
		}
		NLTRAJECTORY_STATE_S *ps0 = ps1 - 1;
		pstate->m_kin.from(&ps0->m_kin,ps1->m_kin.m_j, t - ps0->m_kin.m_t);
		pstate->m_localCurvature = ps0->m_localCurvature + (ps1->m_localCurvature - ps0->m_localCurvature) * (pstate->m_kin.m_s - ps0->m_kin.m_s) / (ps1->m_kin.m_s - ps0->m_kin.m_s);
	}
	else // t >= m_dt
	{
		*pstate = *(NLTRAJECTORY_STATE_S *)NGetLastArrayPtr(&m_trajectoryStateSArray);
	}
	return pstate;
}
#ifdef _NEDITOR
void NLTRAJECTORY_STATE_S_PACK::drawTrajectoryStateSArray(NL2DOCS *p2docs)
{
	NLTRAJECTORY_STATE_S *pstate = (NLTRAJECTORY_STATE_S *)m_trajectoryStateSArray.pFirst;
	pstate++;

	for (Nu32 i = 1; i < m_trajectoryStateSArray.Size; i++, pstate++)
	{
		pstate->m_kin.draw(p2docs, &(pstate - 1)->m_kin);
	}
}
#endif

/*
void NLTRAJECTORY_TRACKER::build(NLPATH_GEOMETRY * ppath_geometry, const NLDRIVETRAINSPECS * pdtspecs)
{
	NLDIFFERENTIAL_DRIVETRAIN_POSE	d2tp;

	NLKIN	 kin;
	NLKIN	*pkin0 = (NLKIN*)m_kinsArray.pFirst;
	NLKIN	*pkin1 = pkin0;
	Nu32	 kin1_id = 0;

	NLPATH_POINT *pkp0 = (NLPATH_POINT*)ppath->m_geometry.m_keyPointsArray.pFirst;
	NLPATH_POINT *pkp1 = pkp0 + 1;
	NLPATH_PRIMITIVE *pprim = (NLPATH_PRIMITIVE*)ppath->m_geometry.m_primitivesArray.pFirst;

	Nf32	curvature;
	Nf32	slocal;

	NResizeArray(&m_differentialDriveTrainPosesArray, 0, NULL, NULL);

	for (Nu32 i = 0; i < ppath->m_geometry.m_primitivesArray.Size; i++, pkp0 = pkp1, pkp1++, pprim++)
	{
		while ((kin1_id < m_kinsArray.Size) && (pkin1->m_s < pkp1->s))
		{
			// on recup�re la courbure en pkin->m_s
					// ( on sait que pkin->m_s est situ� "entre" pkp0 et pkp1 sur la primitive pprim ...
			if (pprim->m_core.m_id == NLPATH_PRIMITIVE_ID_CLOTHOID)
			{
				slocal = pkin1->m_s - pkp0->s;
				curvature = ISFLAG_ON(pprim->m_core.m_flags, FLAG_NLPATH_CLOTHOID_SECOND) ? (pprim->m_core.m_l - slocal) * pkp0->k / pprim->m_core.m_l : slocal * pkp1->k / pprim->m_core.m_l;
			}
			else // pprim->m_core.m_id ==  NLPATH_PRIMITIVE_ID_SEGMENT ou pprim->m_core.m_id ==  NLPATH_PRIMITIVE_ID_ARC
			{
				curvature = pkp1->k;
			}
			d2tp.set(pkin1, curvature);
			NArrayPushBack(&m_differentialDriveTrainPosesArray, (NBYTE*)&d2tp);

			// Kin suivant
			// !!! ATTENTION !!! � la toute fin, "kin1_id = m_kinsArray.Size" et le pointeur pkin1 pointe en dehors de m_kinsArray !!!
			pkin0 = pkin1;
			pkin1++;
			kin1_id++;
		}

		// A partir d'ici nous savons:
		//  pkin1->m_s >= pkp1->s 
		//		OU
		//  kin1_id == m_kinsArray.Size, 
		//	ce qui signifie que l'abscisse curviligne du dernier Kin de m_kinsArray est inf�rieure � longueur totale du chemin...
		//	... et que le pointeur pkin1 courant est invalide ( hors array )
		if (kin1_id < m_kinsArray.Size)
		{
			// on calcule et on ins�re une pose issue de "pkp1"
			getKinAtS(&kin, pkin0, pkin1, pkp1->s);
			d2tp.set(&kin, pkp1->k);
			NArrayPushBack(&m_differentialDriveTrainPosesArray, (NBYTE*)&d2tp);
		}
		else
		{
			break;
		}
	}
	// Une derni�re chose,
	// il est possible que l'abscisse curviligne du dernier kin soit l�g�rement sup�rieure � la longueur totale du chemin.
	if (kin1_id < m_kinsArray.Size)
	{
		kin1_id++;
	}
}
*/