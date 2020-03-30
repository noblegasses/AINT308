#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace std;

#define pi 3.1415
#define IPD 65
#define pwmDeg 10.73
#define RxC 1445
#define LxC 1430

void distCalc(int DataR [10], int DataL [10], int x);

int Run1Rx [10] = {1310, 1355, 1385, 1400, 1414, 1419, 1434, 1435, 1433, 1434};
int Run1Lx [10] = {1595, 1520, 1485, 1455, 1460, 1455, 1460, 1454, 1449, 1449};
int Run2Rx [10] = {1277, 1360, 1377, 1400, 1414, 1414, 1415, 1415, 1425, 1425};
int Run2Lx [10] = {1555, 1514, 1475, 1469, 1459, 1449, 1449, 1445, 1445, 1444};
int Run3Rx [10] = {1319, 1385, 1414, 1419, 1449, 1432, 1443, 1443, 1453, 1454};
int Run3Lx [10] = {1594, 1535, 1514, 1484, 1489, 1469, 1469, 1459, 1459, 1459};

int main()
{
    distCalc(Run1Rx, Run1Lx, 1);
    distCalc(Run2Rx, Run2Lx, 2);
    distCalc(Run3Rx, Run3Lx, 3);

}

void distCalc(int DataR [10], int DataL [10], int x)
{
    cout << "Run" << x << ": \n\r\n\r";
    double Dist [10];
    for(int i = 0; i<10; i++){
        double rEyeAngle=((double(DataR[i]-RxC)*pi)/(pwmDeg*180))*-1; // in radians
        double lEyeAngle=(double(DataL[i]-LxC)*pi)/(pwmDeg*180);
        double lDist = (IPD*cos(rEyeAngle))/sin(lEyeAngle+rEyeAngle);
        Dist[i]= (sqrt((lDist*lDist)+((IPD/2)*(IPD/2)) - (lDist*IPD*sin(lEyeAngle)))/13);
        cout << "Actual Distance: " << ((i+1)*10) << "; Calculated Distance: " << Dist[i] << "\n\r";
    }
    cout << "\n\r";
}
