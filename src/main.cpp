
#include "mbed.h"
#include <Matrix.h>
#include <stdio.h>
#include <Servo.h>
#include <Manipulator.h>
#include <math.h>
#define _USE_MATH_DEFINES

// Link Lengths for 6R Manipulator
#define L1 2
#define L2 4
#define L3 2
#define L4 3
#define L5 1.5
#define L5_P 1
#define L6 5

int main(void) {
    // Create Manipulator object
    Manipulator robo6R;

    /*  DH Table: 6R Orthogonal Manipulator
        link    theta   d       a       alpha
        1       0       L1      0       PI/2
        2       PI/2    0       L2      0
        3       -PI/2   0       0       -PI/2
        4       0       L3+L4   0       PI/2
        5       -PI/2   0       L5      -PI/2
        6       PI      L5_P    L6      0
    */
    float theta1 = 0.0;
    double d1 = L1;
    double a1 = 0.0;
    double alpha1 = M_PI / 2.0;
    robo6R.add_Joint(REVOLUTE, theta1,      alpha1,     d1,     a1);
    robo6R.add_Joint(REVOLUTE, M_PI/2.0,     0.0,       L1,      0.0);
    robo6R.add_Joint(REVOLUTE, -M_PI/2.0,    -M_PI/2.0, 0.0,     0.0); 
    robo6R.add_Joint(REVOLUTE, 0.0,          M_PI/2.0,  L3 + L4, 0.0);
    robo6R.add_Joint(REVOLUTE, -M_PI/2.0,    -M_PI/2.0, 0.0,     L5); 
    robo6R.add_Joint(REVOLUTE, M_PI,         0.0,       L5_P,    L6); 
    // View Configuration
    robo6R.print();

    // Servo objects for each revolute joint
    Servo s1(PA_8, 0.02f, 560, 2440, 0.0,       M_PI); // Sets the home angle with +/- PI radians on either side 
    Servo s2(PC_6, 0.02f, 580, 2570, M_PI/2.0,  M_PI);
    Servo s3(PA_9, 0.02f, 560, 2440, -M_PI/2.0, M_PI);
    Servo s4(PC_7, 0.02f, 580, 2570, 0.0      , M_PI);
    Servo s5(PC_8, 0.02f, 560, 2440, -M_PI/2.0, M_PI);
    Servo s6(PC_9, 0.02f, 580, 2570, M_PI     , M_PI);

    // Calculate Translation matrix from base (0) to manipulator (6)
    Matrix<double> T_06 = robo6R.getTranslation(0,6);
    T_06.print_detailed();

    while(true) {
        // Main process
    }

    return 0;
}