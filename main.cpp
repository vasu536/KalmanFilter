
/***************************************
 *  Created by Srinivas Karuparthi
 *  Kalman filter PUMA 3DOF Robotic Arm
****************************************/

#include <fstream>
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "Matrix.h"
#define TIMESTEP 0.05

using namespace std;

float timeIndex,m_a1,m_a2,m_a3,c_a1,c_a2,c_a3,tau1,tau2,tau3;
float a1,a2,a3,a1d,a2d,a3d,a1dd,a2dd,a3dd;
float new_a1d, new_a2d, new_a3d;
float TIME=0;

Matrix A(6,6);
Matrix B(6,3);
Matrix C=Matrix::createIdentity(6);  //initialize them as 6×6 identity matrices
Matrix Q=Matrix::createIdentity(6);
Matrix R=Matrix::createIdentity(6);
Matrix y(6,1);
Matrix K(6,6);
Matrix x(6,1);
Matrix state(6,1);
Matrix action(3,1);
Matrix lastState(6,1);
Matrix P=Matrix::createIdentity(6);
Matrix p=Matrix::createIdentity(6);
Matrix measurement(6,1);

void initKalman(){

    float a[6][6]={
            {1.004,    0.0001,  0.001,    0.0014,    0.0000,  -0.0003  },
            {0.000,    1.000,     -0.00,      0.0000,    0.0019,   0          },
            {0.0004,  0.0002,  1.002,    0.0003,    0.0001,   0.0015   },
            {0.2028,  0.0481,  0.0433,  0.7114,   -0.0166,  -0.1458   },
            {0.0080,  0.0021, -0.0020, -0.0224,    0.9289,   0.005    },
            {0.1895,  0.1009,  0.1101, -0.1602,    0.0621,   0.7404  }
    };

    float b[6][3] = {
            {0.0000,   0.0000 ,  0.0000  },
            {0.0000,   0.0000,  -0.0000  },
            {0.0000,  -0.0000,   0.0000  },
            {0.007,    -0.0000,   0.0005 },
            {0.0001,   0.0000,  -0.0000 },
            {0.0003,  -0.0000,   0.0008 }
    };

/*loads the A and B matrices from above*/


    A=(&a, A);

    B=(&b, B);

/*initializes the state*/
    state.operator+=(0.1);

    lastState=state;

}

void kalman(){
    lastState=state;
    float state_array[6][1] = {c_a1, c_a2, c_a3, a1d, a2d, a3d};
    state=(&state_array, state);

    float measurement_array[6][1] = {m_a1, m_a2, m_a3, a1d, a2d, a3d};
    measurement=(&measurement_array, measurement);

    float action_array[3][1] = {tau1, tau2, tau3};
    action=(&action_array, action);

    Matrix temp1(6,6);
    Matrix temp2(6,6);
    Matrix temp3(6,6);
    Matrix temp4(6,1);
/************ Prediction Equations*****************/
    x = A*lastState + B*action;
    p = A*P*(A.transpose()) + Q;
/************ Update Equations **********/
    K = p*C*((C*p*(C.transpose())+R).inverse());

    y=C*state;

    state = x + K*(y-C*lastState);

    Matrix I=Matrix::createIdentity(6);

    P = (I-(K*C))*p;

    float a_array[6][1] = {a1, a2, a3, a1d, a2d, a3d};
    state=(a_array, state);

    a1=a_array[0][0];
    a2=a_array[1][0];
    a3=a_array[2][0];
    a1d=a_array[3][0];
    a2d=a_array[4][0];
    a3d=a_array[5][0];
}

/* This function is not used since I am using position to get velocity (i.e. differentiation).
* However I think that it is useful to include if you want velocity and position
* from acceleration you would use it */
void integrate(){
    new_a1d = a1d + a1dd*TIMESTEP;
    a1 += (new_a1d + a1d)*TIMESTEP/2;
    a1d = new_a1d;
    new_a2d = a2d + a2dd*TIMESTEP;
    a2 += (new_a2d + a2d)*TIMESTEP/2;
    a2d = new_a2d;
    new_a3d = a3d + a3dd*TIMESTEP;
    a3 += (new_a3d + a3d)*TIMESTEP/2;
    a3d = new_a3d;
    TIME+=TIMESTEP;
}

/*This gives me velocity from position*/
void differentiation(){

    float a_array[6][1] = {a1, a2, a3, a1d, a2d, a3d};
    float lastState_array[6][1] = {};

    lastState=(lastState_array, lastState);
    state=(a_array, state);

    a1d=(a_array[0][0]-lastState_array[0][0])/TIMESTEP;
    a2d=(a_array[1][0]-lastState_array[1][0])/TIMESTEP;
    a3d=(a_array[2][0]-lastState_array[2][0])/TIMESTEP;
    TIME+=TIMESTEP;
}

int main ()
{
    initKalman();
    char buffer[500];
    ifstream readFile ("Data.txt"); // this is where I read my data since I am proccessing it all offline

    while (!readFile.eof()){
        readFile.getline (buffer,500);
        sscanf(buffer, "%f %f %f %f %f %f %f %f %f %f ",&timeIndex,&m_a1,&m_a2,&m_a3,&c_a1,&c_a2,&c_a3,&tau1,&tau2,&tau3);

        kalman();
        differentiation();
//integrate();

/*this is where I log the results to and/or print it to the screen */
    FILE *file=fopen("filterOutput.txt", "a");
    fprintf(file,"%f %f %f %f %f %f %f %f %f %f \n",TIME,a1,a2,a3,a1d,a2d,a3d,tau1,tau2,tau3);
    fprintf(stderr,"%f %f %f %f %f %f %f %f %f %f \n",TIME,a1,a2,a3,a1d,a2d,a3d,tau1,tau2,tau3);
    fclose(file);
}
    return 1;
}