/*
 * EqSource.h
 *
 * Created: 10/07/2018 16:44:51
 *  Author: Gabriel
 */ 


#ifndef EQSOURCE_H_
#define EQSOURCE_H_


typedef struct
{


	double  lf;      
	double  f1p0;     
	double  f1p1;
	double  f1p2;
	double  f1p3;


	double  hf;     
	double  f2p0;    
	double  f2p1;
	double  f2p2;
	double  f2p3;


	double  sdm1;     
	double  sdm2;   
	double  sdm3;   


	double  lg;    
	double  mg;      
	double  hg;     
	
} EQSTATE;

//Funções Def

extern void   init_3band_state(EQSTATE* es, int lowfreq, int highfreq, int mixfreq);
extern double do_3band(EQSTATE* es, double sample);



#endif /* EQSOURCE_H_ */