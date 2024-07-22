#include <stdio.h>
#include <math.h>


int Nlt[] = {0, 100, 300, 500, 700, 900, 1000, 1100, 1200, 1200};
int Nrt[] = {0, 100, 200, 300, 400, 500, 600, 700, 800, 800};
int size_tableau = sizeof(Nlt) / sizeof(Nlt[0]);

float r = 0.05, b = 0.2, C = 100; // r et b en m
int diffL = 0, diffR = 0;

void encUpdate(int t) {
    if (t != 0) {
        diffL = Nlt[t] - Nlt[t-1];
        printf("La diff de gauche est de %d\n", diffL);
        diffR = Nrt[t] - Nrt[t-1];
        printf("La diff de droite est de %d\n", diffR);
    } else {
        diffL = 0;
        diffR = 0;
    }
}

void poseUpdate(float NL, float NR, float r, float b, float C){
    float DL = ((2 * 3.14 * r) / C) * NL;
    float DR = ((2 * 3.14 * r) / C) * NR;
    printf("DL: %.4f\n", DL);
    printf("DR: %.4f\n", DR);
    float D = (DL+DR)/2;
    printf("la moyenne des 2 distances: %.4f\n", D);
    float dtelta=(DR-DL)/b;
    float dx=D*cos(dtheta);
    float dy=D*sin(dtheta);
    
}

void cmd_vel(){
    float deltat=0.1;
    float Velo=((2*3.14*r)/C)*((NR+NL)/2)/deltat;
    float WVelo=((2*3.14*r)/C)*((NR-NL)/2)/deltat;

}

int main() {

    for (int t = 1; t < size_tableau; t++) {
        encUpdate(t);
    }
    
    for (int t = 1; t < size_tableau; t++) {
        poseUpdate(Nlt[t], Nrt[t], r, b, C);
    }

    return 0;
}
