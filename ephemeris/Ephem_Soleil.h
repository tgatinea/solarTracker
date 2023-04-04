#ifndef Ephem_Soleil_h
#define Ephem_Soleil_h

#include <Arduino.h>

int jourAnnee(int, int, int);

void posSoleil(int, int, int, int, int, int, int, double, double, double*, double*);

void posSoleil(String, int, double, double, double*, double*);

void lmvSoleil(int, int, int, int, int, double, double, String*, String*, String*, int = 0);

void lmvSoleil(String, int, int, double, double, String*, String*, String*, int = 0);

void lmvSoleil(int, int, int, int, int, double, double, double*, double*, double*, int = 0);

void lmvSoleil(String, int, int, double, double, double*, double*, double*, int = 0);

#endif
