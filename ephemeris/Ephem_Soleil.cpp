#include <Arduino.h>
#include<Ephem_Soleil.h>

// Les calculs suivants sont basés sur le document : https://www.esrl.noaa.gov/gmd/grad/solcalc/solareqns.PDF

#define PI_180  0.017453292
#define A180_PI 57.29577951

int nbJmois[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
double eqT; // minutes
double decl; // radians

String conv_Mn_HMS(double mn) { // retourne l'heure, les minutes et les secondes sous forme d'une chaîne au format : HH:MM:SS
  int hr = mn / 60;
  int min = trunc(mn - 60 * hr);
  int sec = trunc((mn - trunc(mn)) * 60);
  char stx[10]; // stockage de la chaïne complète
  sprintf(stx, "%02u:%02u:%02u", hr, min, sec);
  return stx;
}

int jourAnnee(int a, int m, int j) { // retourne le jour de l'année "a" au mois "m" et au jour "j"
  int nbJ = 0;
  for (byte i = 0; i < m; i++)
    nbJ += nbJmois[i];
  if (a % 4 == 0 && m > 2) // Ca s d'une année bissextile
    nbJ++;
  return nbJ + j;
}

double fracAnnee(int a, int m, int j, int h, int mn, int s) { // fraction de l'année en radians le jour de l'année "a", au mois "m", au jour "j", à l'heure "h", la minute "mn" et la seconde "s"
  int n = 365;
  if (a % 4 == 0) // Cas de l'année bissextile
    n = 366;
  return 2 * PI * (jourAnnee(a, m, j) - 1 + (h + mn / 60 + s / 3600 - 12) / 24) / n; // radians
}

void eqT_Decl(double fA) { // retourne l'équation du temps en minutes et l'angle de déclinaison du soleil en radians
  eqT = 229.18 * (0.000075 + 0.001868 * cos(fA) - 0.032077 * sin(fA) - 0.014615 * cos(2 * fA) - 0.040849 * sin(2 * fA)); // minutes
  decl = 0.006918 - 0.399912 * cos(fA) + 0.070257 * sin(fA) - 0.006758 * cos(2 * fA) + 0.000907 * sin(2 * fA) - 0.002697 * cos(3 * fA) + 0.00148 * sin(3 * fA); // radians
}

void posSoleil(int a, int m, int j, int h, int mn, int s, int tZ, double la, double lon, double *ha, double *az) {
	// "tZ" est la zone horaire du lieu
	// En plus de données calendaires vues ci-avant, "la" et "lon" sont respectivement la latitude et la longitude du lieu en degrés décimaux.
	// La latitude est comprise entre - 90 ° et +90 °. Un angle positif caractérise l'hémisphère Nord
	// "ha" est la hauteur  du soleil au-dessu de l'horizon. "az" est l'azimut du soleil compté positivement dans le sens des aiguilles d'une motre à partir du Nord
	//"ha" et "az" doivent être transmis par adresse
  eqT_Decl(fracAnnee(a, m, j, h, mn, s));
  double tOff = eqT + 4.0 * lon - 60.0 * tZ; // minutes
  double tSv = h * 60.0 + mn + s / 60.0 + tOff; // minutes
  double aS = tSv / 4.0 - 180; // degrés
  la = la * PI_180; // radians
  lon = lon * PI_180; // radians
  double ze = acos(sin(la) * sin(decl) + cos(la) * cos(decl) * cos(aS * PI_180)); // radians
  double cs = -(sin(la) * cos(ze) - sin(decl)) / (cos(la) * sin(ze)); // radians
  if (cs < -1) // protection contre les imprécisions de calcul
    cs = -1;
  else if (cs > 1)
    cs = 1;
  *az = acos(cs) * A180_PI; // degrés
  if (aS > 0)
    *az = 360 - *az;
  *ha = 90 - ze * A180_PI; // degrés
}

void posSoleil(String dh, int tZ, double la, double lon, double *ha, double *az) {
	// Même fonction que le précédente sauf que la date et l'heure sont fournies au format ci-après
  // dh est au format : JJsMMsAAAAsHHsMMsSS où s est un séparateur quelconque de un caractère
  int a = dh.substring(6, 10).toInt();
  int m = dh.substring(3, 5).toInt();
  int j = dh.substring(0, 2).toInt();
  int h = dh.substring(11, 13).toInt();
  int mn = dh.substring(14, 16).toInt();
  int s = dh.substring(17).toInt();
  posSoleil(a, m, j, h, mn, s, tZ, la, lon, ha, az);
}

void lmvSoleil(int a, int m, int j, int tZ, int tcr, double la, double lon, double *lSo, double *mSo, double *cSo, int alt) {
	// Mêmes données calendaires que ci-avant.
	// "tcr" est le type de crépuscule retenu :
	//   -  0 : crépuscule sur l'horizon avec correction de la réfraction athmosphérique  : r = 0.833°
	//   -  6 : crépuscule civil
	//   -  12 : crépuscule nautique
	//   -  18 : crépuscule astronomique
	// "lSo" est l'heure de levée du soleil, "mSo" est celle du zénit et "cSo" celle du coucher. Ces valeurs doivent être fournies par adresse
	// alt est l'altitude du lieu en mètres
  eqT_Decl(fracAnnee(a, m, j, 12, 0, 0));
  la = la * PI_180; // radians
  double acr; // cosinus de l'angle crépusculaire
  switch (tcr) {
    case 0: // crépuscule sur l'horizon
      acr = cos(PI_180 * (90.833 + 0.015 * sqrt(alt)));
      break;
    case 6: // crépuscule civil
      acr = cos(PI_180 * 96);
      break;
    case 12 : // crépuscule nautique
      acr = cos(PI_180 * 102);
      break;
    case 18 : // crépuscule astronomique
      acr = cos(PI_180 * 108);
      break;
  }
  double aS = acos(acr / (cos(la) * cos(decl)) - tan(la) * tan(decl)) * A180_PI;
  *lSo = 720 + 4 * (-lon - aS) - eqT + 60.0 * tZ;
  *cSo = 720 + 4 * (-lon + aS) - eqT + 60.0 * tZ;
  *mSo = 720 + 4 * -lon - eqT + 60.0 * tZ;
}

void lmvSoleil(int a, int m, int j, int tZ, int tcr, double la, double lon, String *lSo, String *mSo, String *cSo, int alt) {
	// Mêmes données calendaires que ci-avant.
	// "tcr" est le type de crépuscule retenu :
	//   -  0 : crépuscule sur l'horizon avec correction de la réfraction athmosphérique  : r = 0.833°
	//   -  6 : crépuscule civil
	//   -  12 : crépuscule nautique
	//   -  18 : crépuscule astronomique
	// "lSo" est l'heure de levée du soleil, "mSo" est celle du zénit et "cSo" celle du coucher. Ces valeurs doivent être fournies par adresse
	// alt est l'altitude du lieu en mètres
  eqT_Decl(fracAnnee(a, m, j, 12, 0, 0));
  la = la * PI_180; // radians
  double acr; // cosinus de l'angle crépusculaire
  switch (tcr) {
    case 0: // crépuscule sur l'horizon
      acr = cos(PI_180 * (90.833 + 0.015 * sqrt(alt)));
      break;
    case 6: // crépuscule civil
      acr = cos(PI_180 * 96);
      break;
    case 12 : // crépuscule nautique
      acr = cos(PI_180 * 102);
      break;
    case 18 : // crépuscule astronomique
      acr = cos(PI_180 * 108);
      break;
  }
  double aS = acos(acr / (cos(la) * cos(decl)) - tan(la) * tan(decl)) * A180_PI;
  *lSo = conv_Mn_HMS(720 + 4 * (-lon - aS) - eqT + 60.0 * tZ);
  *cSo = conv_Mn_HMS(720 + 4 * (-lon + aS) - eqT + 60.0 * tZ);
  *mSo = conv_Mn_HMS(720 + 4 * -lon - eqT + 60.0 * tZ);
}

void lmvSoleil(String dh, int tZ, int tcr, double la, double lon, String *lSo, String *mSo, String *cSo, int alt) {
	// Même fonction que le précédente sauf que la date et l'heure sont fournies au format ci-après
  // dh est au format : JJsMMsAAAAsHHsMMsSS où s est un séparateur quelconque de un caractère
  int a = dh.substring(6, 10).toInt();
  int m = dh.substring(3, 5).toInt();
  int j = dh.substring(0, 2).toInt();
  int h = dh.substring(11, 13).toInt();
  int mn = dh.substring(14, 16).toInt();
  int s = dh.substring(17).toInt();
  lmvSoleil(a, m, j, tZ, tcr, la, lon, lSo, mSo, cSo, alt);
}


void lmvSoleil(String dh, int tZ, int tcr, double la, double lon, double *lSo, double *mSo, double *cSo, int alt) {
	// Même fonction que le précédente sauf que la date et l'heure sont fournies au format ci-après
  // dh est au format : JJsMMsAAAAsHHsMMsSS où s est un séparateur quelconque de un caractère
  int a = dh.substring(6, 10).toInt();
  int m = dh.substring(3, 5).toInt();
  int j = dh.substring(0, 2).toInt();
  int h = dh.substring(11, 13).toInt();
  int mn = dh.substring(14, 16).toInt();
  int s = dh.substring(17).toInt();
  lmvSoleil(a, m, j, tZ, tcr, la, lon, lSo, mSo, cSo, alt);
}
