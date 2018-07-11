#include <math.h>
#include "EqSource.h"


static double vsa = (1.0 / 4294967295.0); 



void init_3band_state(EQSTATE* es, int lowfreq, int highfreq, int mixfreq)
{

	// Set Low/Mid/High

	es->lg = 1.0;
	es->mg = 1.0;
	es->hg = 1.0;

	
	es->lf = 2 * sin(M_PI * ((double)lowfreq / (double)mixfreq));
	es->hf = 2 * sin(M_PI * ((double)highfreq / (double)mixfreq));
}



double do_3band(EQSTATE* es, double sample)
{


	double  l,m,h;      // Low / Mid / High


	es->f1p0  += (es->lf * (sample   - es->f1p0)) + vsa;
	es->f1p1  += (es->lf * (es->f1p0 - es->f1p1));
	es->f1p2  += (es->lf * (es->f1p1 - es->f1p2));
	es->f1p3  += (es->lf * (es->f1p2 - es->f1p3));

	l          = es->f1p3;

	
	es->f2p0  += (es->hf * (sample   - es->f2p0)) + vsa;
	es->f2p1  += (es->hf * (es->f2p0 - es->f2p1));
	es->f2p2  += (es->hf * (es->f2p1 - es->f2p2));
	es->f2p3  += (es->hf * (es->f2p2 - es->f2p3));

	h          = es->sdm3 - es->f2p3;


	m          = es->sdm3 - (h + l);


	l         *= es->lg;
	m         *= es->mg;
	h         *= es->hg;


	es->sdm3   = es->sdm2;
	es->sdm2   = es->sdm1;
	es->sdm1   = sample;


	return(l + m + h);
}