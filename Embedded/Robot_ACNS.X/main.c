#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ChipConfig.h"
#include "IO.h"
#include "timer.h"
#include "PWM.h"
#include "ADC.h"
#include "Robot.h"
#include "main.h"
#include "UART.h"
#include "CB_TX1.h"
#include "CB_RX1.h"
#include "CB_RX2.h"
#include "UART_Protocol.h"
#include <libpic30.h>
#include "QEI.h"
#include "Utilities.h"
#include "asservissement.h"
#include "GhostManager.h"

#include <string.h> 
unsigned char controlState;
int mode = 0;
double thetaRes = 0;
double thetawp = 0;
double thetaRobot = 0;
double thetaStop = 0;
double aAng = 0;
double vMax = 3;
double tolAng = 0;
double vAng = 0;

float vitessed = 25;
float vitesseg = 25;

extern volatile GhostPosition ghostPosition;


int main(void) {
    /***************************************************************************************************/
    //Initialisation de l?oscillateur
    /****************************************************************************************************/
    InitOscillator();
    /****************************************************************************************************/
    // Configuration des éentres sorties
    /****************************************************************************************************/
    InitIO();
    InitTimer23();
    InitTimer1();
    InitPWM();
    InitADC1();
    InitTimer4();
    InitUART();
    InitUART2();
    InitQEI1();
    InitQEI2();
    
    InitTrajectoryGenerator();
    
    robotState.angleRadianFromOdometry = ghostPosition.theta;
    robotState.xPosFromOdometry = ghostPosition.x;
    robotState.yPosFromOdometry = ghostPosition.y;
    
    SetupPidAsservissement(&robotState.PidX, 1.0f,  30.0f,0.0f, 100.0f, 100.0f, 100.0f);
    SetupPidAsservissement(&robotState.PidTheta, 1.0f,  30.0f,0.0f, 100.0f, 100.0f, 100.0f);
    SetupPidAsservissement(&robotState.PdTheta, 0.625f,  0.0f, 0.5f, 100.0f, 100.0f, 100.0f);
    SetupPidAsservissement(&robotState.PdLin, 0.0f,  0.0f, 0.5f, 100.0f, 100.0f, 100.0f);
    



    /****************************************************************************************************/
    // Boucle Principale
    /****************************************************************************************************/
    while (1) {
        
        while(CB_RX1_GetDataSize()>0)
        {
            unsigned char c = CB_RX1_Get();
            UartDecodeMessage(c);
        }
        
        while(CB_RX2_GetDataSize()>0)
        {
            unsigned char c = CB_RX2_Get();
            UartDecodeMessage_UART2(c);
        }   
       
        if (ADCIsConversionFinished()) {
            ADCClearConversionFinishedFlag();
            unsigned int * result = ADCGetResult();
            
            float volts = ((float) result [0])* 3.3 / 4096;
            if (volts < 0.325)volts = 0.325;
            robotState.distanceTelemetreExtremeGauche = 34 / volts - 5;
            
            volts = ((float) result [1])* 3.3 / 4096;
            if (volts < 0.325)volts = 0.325;
            robotState.distanceTelemetreGauche = 34 / volts - 5;
            
            volts = ((float) result [2])* 3.3 / 4096;
            if (volts < 0.325)volts = 0.325;
            robotState.distanceTelemetreCentre = 34 / volts - 5;
            
            volts = ((float) result [3])* 3.3 / 4096;
            if (volts < 0.325)volts = 0.325;
            robotState.distanceTelemetreDroit = 34 / volts - 5;
            
            volts = ((float) result [4])* 3.3 / 4096;
            if (volts < 0.325)volts = 0.325;
            robotState.distanceTelemetreExtremeDroit = 34 / volts - 5;
            
//            if (robotState.distanceTelemetreExtremeGauche <= 20) LED_BLANCHE_1 = 1;
//            else LED_BLANCHE_1 = 0;
//            if (robotState.distanceTelemetreGauche <= 20) LED_BLEUE_1 = 1;
//            else LED_BLEUE_1 = 0;
//            if (robotState.distanceTelemetreCentre <= 20) LED_ORANGE_1 = 1;
//            else LED_ORANGE_1 = 0;
//            if (robotState.distanceTelemetreDroit <= 20) LED_ROUGE_1 = 1;
//            else LED_ROUGE_1 = 0;
//            if (robotState.distanceTelemetreExtremeDroit <= 20) LED_VERTE_1 = 1;
//            else LED_VERTE_1 = 0;
          
        }
    //PWMSetSpeedConsigne(vitessed,MOTEUR_GAUCHE);
    //PWMSetSpeedConsigne(vitesseg,MOTEUR_DROIT);
    //PWMSetSpeedConsignePolaire(1, 0);
    }

} // fin main

unsigned char stateRobot;

void SetRobotState(unsigned char state) {
    robotState.mode = state;
}

//void OperatingSystemLoop(void) {
//    switch (stateRobot) {
//        case STATE_ATTENTE:
//            timestamp = 0;
//            PWMSetSpeedConsignePolaire(0, 0);
//            stateRobot = STATE_ATTENTE_EN_COURS;
//            break;
//        case STATE_ATTENTE_EN_COURS:
//            if (timestamp > 1000)
//                stateRobot = STATE_AVANCE;
//            break;
//        case STATE_AVANCE:
//            PWMSetSpeedConsignePolaire(1, 0);
//            stateRobot = STATE_AVANCE_EN_COURS;
//            LED_BLANCHE_1 = 1;
//            break;
//        case STATE_AVANCE_EN_COURS:
//            SetNextRobotStateInAutomaticMode();
//            break;
//        case STATE_TOURNE_GAUCHE:
//            PWMSetSpeedConsignePolaire(1, 3);
//            stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
//            LED_ORANGE_1 = 1;
//            break;
//        case STATE_TOURNE_GAUCHE_EN_COURS:
//            SetNextRobotStateInAutomaticMode();
//            break;
//        case STATE_TOURNE_DROITE:
//            PWMSetSpeedConsignePolaire(1, -3);
//            stateRobot = STATE_TOURNE_DROITE_EN_COURS;
//            LED_BLEUE_1 = 1;
//            break;
//        case STATE_TOURNE_DROITE_EN_COURS:
//            SetNextRobotStateInAutomaticMode();
//            break;
//        case STATE_TOURNE_SUR_PLACE_GAUCHE:
//            PWMSetSpeedConsignePolaire(0, 3);
//            stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
//            LED_ROUGE_1 = 1;
//            break;
//        case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
//            SetNextRobotStateInAutomaticMode();
//            break;
//        case STATE_TOURNE_SUR_PLACE_DROITE:
//            PWMSetSpeedConsignePolaire(0, -3);
//            stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
//            LED_VERTE_1 = 1;
//            break;
//        case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
//            SetNextRobotStateInAutomaticMode();
//            break;
//        default:
//            stateRobot = STATE_ATTENTE;
//            break;
//    }
//}

void OperatingSystemLoop(void) {
    switch (stateRobot) {
        case STATE_ATTENTE:
            timestamp = 0;
            PWMSetSpeedConsigne(0, MOTEUR_DROIT);
            PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
            stateRobot = STATE_ATTENTE_EN_COURS;
        case STATE_ATTENTE_EN_COURS:
            if (timestamp > 1000) {
                stateRobot = STATE_AVANCE;
            }
            break;
        case STATE_AVANCE:
            PWMSetSpeedConsigne(vitessed, MOTEUR_DROIT);
            PWMSetSpeedConsigne(vitesseg, MOTEUR_GAUCHE);
            stateRobot = STATE_AVANCE_EN_COURS;
            LED_BLANCHE_1 = 1;
            break;
        case STATE_AVANCE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_TOURNE_GAUCHE:
            PWMSetSpeedConsigne(vitessed, MOTEUR_DROIT);
            PWMSetSpeedConsigne(vitesseg, MOTEUR_GAUCHE);
            stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
            LED_VERTE_1 = 1;
            break;
        case STATE_TOURNE_GAUCHE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_TOURNE_DROITE:
            PWMSetSpeedConsigne(vitessed, MOTEUR_DROIT);
            PWMSetSpeedConsigne(vitesseg, MOTEUR_GAUCHE);
            stateRobot = STATE_TOURNE_DROITE_EN_COURS;
            LED_BLEUE_1 = 1;
            break;
        case STATE_TOURNE_DROITE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_TOURNE_SUR_PLACE_GAUCHE:
            PWMSetSpeedConsigne(vitessed, MOTEUR_DROIT);
            PWMSetSpeedConsigne(vitesseg, MOTEUR_GAUCHE);
            stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
            LED_ROUGE_1 = 1;
            break;
        case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_TOURNE_SUR_PLACE_DROITE:
            PWMSetSpeedConsigne(vitessed, MOTEUR_DROIT);
            PWMSetSpeedConsigne(vitesseg, MOTEUR_GAUCHE);
            stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
            LED_ORANGE_1 = 1;
            break;
        case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        case STATE_RECULE:
            PWMSetSpeedConsigne(vitessed, MOTEUR_DROIT);
            PWMSetSpeedConsigne(vitesseg, MOTEUR_GAUCHE);
            stateRobot = STATE_RECULE_EN_COURS;
            LED_VERTE_1 = 1;
            break;
        case STATE_RECULE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;
        default:
            stateRobot = STATE_ATTENTE;
            break;
    }
}

unsigned char nextStateRobot = 0;

void SetNextRobotStateInAutomaticMode() {
    unsigned char positionObstacle = PAS_D_OBSTACLE;
    //éDtermination de la position des obstacles en fonction des ééètlmtres
    if (((robotState.distanceTelemetreDroit < 30) && robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 30 &&
            robotState.distanceTelemetreExtremeGauche > 20)) //Obstacle àdroite
        positionObstacle = OBSTACLE_A_GAUCHE;
    else if (robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreExtremeDroit > 20 &&
            robotState.distanceTelemetreCentre > 20 &&
            (robotState.distanceTelemetreGauche < 30 &&
            robotState.distanceTelemetreExtremeGauche < 20)) //Obstacle àgauche
        positionObstacle = OBSTACLE_A_DROITE;

    else if ((robotState.distanceTelemetreCentre < 30) ||
            (robotState.distanceTelemetreCentre < 30 &&
            robotState.distanceTelemetreDroit < 30) ||
            (robotState.distanceTelemetreCentre < 30 &&
            robotState.distanceTelemetreGauche < 30) ||
            (robotState.distanceTelemetreDroit < 30 && robotState.distanceTelemetreGauche < 30)) {
        positionObstacle = OBSTACLE_EN_FACE;
    }//Obstacle en face


    else if (robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 30 &&
            robotState.distanceTelemetreExtremeGauche > 20 &&
            robotState.distanceTelemetreExtremeDroit > 20) //pas d?obstacle
        positionObstacle = PAS_D_OBSTACLE;

        //Ajout des cas en fonction des cas déjà éxistants
    else if ((robotState.distanceTelemetreDroit < 30 && robotState.distanceTelemetreExtremeDroit < 20) &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 30 &&
            robotState.distanceTelemetreExtremeGauche > 20) //Obstacle à droite
        positionObstacle = OBSTACLE_A_GAUCHE;

    else if ((robotState.distanceTelemetreDroit > 30 && robotState.distanceTelemetreExtremeDroit > 20) &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche < 30 &&
            robotState.distanceTelemetreExtremeGauche < 20) //Obstacle à droite
        positionObstacle = OBSTACLE_A_DROITE;

    else if (robotState.distanceTelemetreCentre < 20 && robotState.distanceTelemetreGauche < 30 &&
            robotState.distanceTelemetreCentre < 20) {
        positionObstacle = OBSTACLE_EN_FACE;
    }//Obstacle en face

    else if (robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 30 &&
            robotState.distanceTelemetreExtremeGauche > 20 &&
            robotState.distanceTelemetreExtremeDroit > 20) //pas d?obstacle
        positionObstacle = PAS_D_OBSTACLE;

        //Ajout d'autres cas de position d'obstacles
        //|C1 |C2 |C3 |C4 C5
    else if (robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 30 &&
            robotState.distanceTelemetreExtremeGauche < 20 &&
            robotState.distanceTelemetreExtremeDroit > 20)
        positionObstacle = OBSTACLE_A_DROITE;

        //    //C1 |C2 |C3 |C4 |C5
    else if (robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 30 &&
            robotState.distanceTelemetreExtremeGauche > 20 &&
            robotState.distanceTelemetreExtremeDroit < 20)
        positionObstacle = OBSTACLE_A_GAUCHE;

        //C1 |C2 |C3 |C4 C5
    else if (robotState.distanceTelemetreExtremeGauche < 20 &&
            robotState.distanceTelemetreExtremeDroit < 20 &&
            robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 30)
        positionObstacle = OBSTACLE_EN_FACE;

        //    //|C1 C2 C3 |C4 |C5
    else if (robotState.distanceTelemetreExtremeGauche > 20 &&
            robotState.distanceTelemetreExtremeDroit > 20 &&
            robotState.distanceTelemetreDroit < 30 &&
            robotState.distanceTelemetreCentre < 30 &&
            robotState.distanceTelemetreGauche > 30)
        positionObstacle = OBSTACLE_A_GAUCHE;
        //
        //    //|C1 |C2 C3 C4 |C5
    else if (robotState.distanceTelemetreExtremeGauche > 20 &&
            robotState.distanceTelemetreExtremeDroit > 20 &&
            robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre < 30 &&
            robotState.distanceTelemetreGauche < 30)
        positionObstacle = OBSTACLE_A_DROITE;
        //
        // |C1 |C2 C3 C4 C5
    else if (robotState.distanceTelemetreExtremeGauche < 20 &&
            robotState.distanceTelemetreExtremeDroit > 20 &&
            robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre < 20 &&
            robotState.distanceTelemetreGauche < 30)
        positionObstacle = OBSTACLE_A_GAUCHE;

        // |C1 |C2 |C3 C4 C5
    else if (robotState.distanceTelemetreExtremeGauche < 20 &&
            robotState.distanceTelemetreExtremeDroit > 20 &&
            robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche < 30)
        positionObstacle = OBSTACLE_A_GAUCHE;
        //
        //    
        //    // |C1 |C2 C3 |C4 |C5
        //    else if (robotState.distanceTelemetreExtremeGauche > 10 &&
        //            robotState.distanceTelemetreExtremeDroit > 10 &&
        //            robotState.distanceTelemetreDroit > 30 &&
        //            robotState.distanceTelemetreCentre < 20 &&
        //            robotState.distanceTelemetreGauche > 30)
        //        positionObstacle = OBSTACLE_A_DROITE;
        //   
        //    // C1 C2 C3 C4 |C5 //OK Marche relativement
    else if (robotState.distanceTelemetreExtremeGauche > 20 &&
            robotState.distanceTelemetreExtremeDroit < 20 &&
            robotState.distanceTelemetreDroit < 30 &&
            robotState.distanceTelemetreCentre < 20 &&
            robotState.distanceTelemetreGauche < 30)
        positionObstacle = OBSTACLE_A_GAUCHE;

        // C1 C2 C3 |C4 |C5
    else if (robotState.distanceTelemetreExtremeGauche > 20 &&
            robotState.distanceTelemetreExtremeDroit < 20 &&
            robotState.distanceTelemetreDroit < 30 &&
            robotState.distanceTelemetreCentre < 20 &&
            robotState.distanceTelemetreGauche > 30)
        positionObstacle = OBSTACLE_A_GAUCHE;

        // C1 C2 |C3 |C4 |C5
    else if (robotState.distanceTelemetreExtremeGauche > 20 &&
            robotState.distanceTelemetreExtremeDroit < 20 &&
            robotState.distanceTelemetreDroit < 30 &&
            robotState.distanceTelemetreCentre > 30 &&
            robotState.distanceTelemetreGauche > 30)
        positionObstacle = OBSTACLE_A_GAUCHE;
        //
        //    // |C1 C2 C3 C4 C5 
    else if (robotState.distanceTelemetreExtremeGauche < 20 &&
            robotState.distanceTelemetreExtremeDroit > 20 &&
            robotState.distanceTelemetreDroit < 30 &&
            robotState.distanceTelemetreCentre < 20 &&
            robotState.distanceTelemetreGauche < 30)
        positionObstacle = OBSTACLE_A_DROITE;

        //    // C1 C2 C3 C4 C5
    else if (robotState.distanceTelemetreExtremeGauche < 20 &&
            robotState.distanceTelemetreExtremeDroit < 20 &&
            robotState.distanceTelemetreDroit < 30 &&
            robotState.distanceTelemetreCentre < 20 &&
            robotState.distanceTelemetreGauche < 30)
        positionObstacle = OBSTACLE_EN_FACE;

    //éDtermination de lé?tat àvenir du robot
    if (positionObstacle == PAS_D_OBSTACLE)
        nextStateRobot = STATE_AVANCE;
    else if (positionObstacle == OBSTACLE_A_DROITE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    else if (positionObstacle == OBSTACLE_A_GAUCHE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
    else if (positionObstacle == OBSTACLE_EN_FACE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
    //Si l?on n?est pas dans la transition de lé?tape en cours
    if (nextStateRobot != stateRobot - 1) {
        stateRobot = nextStateRobot;
        if (stateRobot != '\n')
            robotStateChange(nextStateRobot);
    }
}