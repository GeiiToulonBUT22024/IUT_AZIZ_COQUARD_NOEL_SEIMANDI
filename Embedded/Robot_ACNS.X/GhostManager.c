#include <math.h>
//#include <stdlib.h>
#include "GhostManager.h"
#include "timer.h"
#include "Robot.h"
#include "utilities.h"
#include "UART_Protocol.h"
#include "QEI.h"

extern unsigned long timestamp;

volatile GhostPosition ghostPosition;
extern volatile int waypoint_received;
volatile int mooving;

double maxAngularSpeed = 1.5; // rad/s
double angularAccel = 2.5; // rad/s^2
double maxLinearSpeed = 1; // m/s
double minMaxLinenearSpeed = 0.5; // m/s
double linearAccel = 1; // m/s^2

int current_state = IDLE;
int waypoint_index = 0;

// Waypoint_t waypoints[MAX_POS] = {{0, 0, 0}, {0, 0.5, 0}, {-1, 0.5, 0}, {-1, -0.5, 0}, {0, -0.5, 0}, {0, 0, 0}, {1.3, 0, 1}};
Waypoint_t waypoints[MAX_POS];



void InitTrajectoryGenerator(void) {
    // ghostPosition.x = 0.0;
    ghostPosition.x = 0.0;
    ghostPosition.y = 0.0;
    ghostPosition.theta = 0;
    ghostPosition.linearSpeed = 0.0;
    ghostPosition.angularSpeed = 0.0;
    ghostPosition.targetX = 0.0;
    ghostPosition.targetY = 0.0;
    ghostPosition.angleToTarget = 0.0;
    ghostPosition.distanceToTarget = 0.0;
}

void UpdateTrajectory() // Mise a jour de la trajectoire en fonction de l'etat actuel par rapport au waypoint
{
    // Target -> le waypoint : c'est où on veut aller
    double thetaTarget = atan2(ghostPosition.targetY - ghostPosition.y, ghostPosition.targetX - ghostPosition.x);
    // Theta entre le robot et où on veut aller
    double thetaRestant = ModuloByAngle(ghostPosition.theta, thetaTarget) - ghostPosition.theta;
    ghostPosition.angleToTarget = thetaRestant;
    // Theta à partir duquel on considère que c'est good
    double thetaArret = ghostPosition.angularSpeed * ghostPosition.angularSpeed / (2 * angularAccel);
    // Pas d'angle à ajouter
    double incrementAng = ghostPosition.angularSpeed / FREQ_ECH_QEI;
    double incremntLin = ghostPosition.linearSpeed / FREQ_ECH_QEI;

    double distanceArret = ghostPosition.linearSpeed * ghostPosition.linearSpeed / (2 * linearAccel);

    double distanceRestante = sqrt((ghostPosition.targetX - ghostPosition.x) * (ghostPosition.targetX - ghostPosition.x)
            + (ghostPosition.targetY - ghostPosition.y) * (ghostPosition.targetY - ghostPosition.y));
    ghostPosition.distanceToTarget = distanceRestante;

    /* ################## IDLE ################## */
    if (current_state == IDLE) {
        if(waypoint_received == 1) {
            Waypoint_t nextWay = waypoints[0];
            ghostPosition.targetX = nextWay.x;
            ghostPosition.targetY = nextWay.y;
            current_state = ROTATING;
            mooving = 1;
            waypoint_received = 0 ;
            //}
        }
        else {
            waypoint_index = 0;
        }
        
    /* ################## ROTATIONING ################## */
    } else if (current_state == ROTATING || current_state == LASTROTATE) {

        if (ghostPosition.angularSpeed < 0) thetaArret = -thetaArret;

        if (((thetaArret >= 0 && thetaRestant >= 0) || (thetaArret <= 0 && thetaRestant <= 0)) && (Abs(thetaRestant) >= Abs(thetaArret))) {
            // on accélère en rampe saturée 
            if (thetaRestant > 0) {
                ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, maxAngularSpeed);
            } else if (thetaRestant < 0) {
                ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, -maxAngularSpeed);
            }
            //        else {
            //            ghostPosition.angularSpeed = 0;
            //        }

        } else {
            //on freine en rampe saturée
            if (thetaRestant >= 0 && ghostPosition.angularSpeed > 0) {
                ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, 0);
            } else if (thetaRestant >= 0 && ghostPosition.angularSpeed < 0) {
                ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, 0);
            } else if (thetaRestant <= 0 && ghostPosition.angularSpeed > 0) {
                ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, 0);
            } else if (thetaRestant <= 0 && ghostPosition.angularSpeed < 0) {
                ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, 0);
            }

            if (Abs(thetaRestant) < Abs(incrementAng)) {
                incrementAng = thetaRestant;
            }
        }

        ghostPosition.theta += incrementAng;
        robotState.consigneVitesseAngulaire = ghostPosition.angularSpeed;

        if (ghostPosition.angularSpeed == 0 && (Abs(thetaRestant) < 0.01)) {
            ghostPosition.theta = thetaTarget;
            robotState.angleRadianFromOdometry = thetaTarget;
            robotState.PidTheta.epsilon_1 = 0;
            // if(robotState.angleRadianFromOdometry == )
            if(current_state != LASTROTATE) 
                current_state = ADVANCING;
            else
                current_state = IDLE;
        }

        
    /* ################## AVANCING ################## */
    } else if (current_state == ADVANCING) {

        if ((distanceRestante != 0) && (Modulo2PIAngleRadian(thetaRestant) < 0.01)) {
            if (((distanceArret >= 0 && distanceRestante >= 0) || (distanceArret <= 0 && distanceRestante <= 0)) && Abs(distanceRestante) >= Abs(distanceArret)) {
                if (distanceRestante > 0) {
                    ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, maxLinearSpeed);
                } else if (distanceRestante < 0) {
                    ghostPosition.linearSpeed = Max(ghostPosition.linearSpeed - linearAccel / FREQ_ECH_QEI, -maxLinearSpeed);
                }
            } else {

                if (distanceRestante >= 0 && ghostPosition.linearSpeed > 0) {
                    ghostPosition.linearSpeed = Max(ghostPosition.linearSpeed - linearAccel / FREQ_ECH_QEI, 0);
                } else if (distanceRestante >= 0 && ghostPosition.linearSpeed < 0) {
                    ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, 0);
                } else if (distanceRestante <= 0 && ghostPosition.linearSpeed > 0) {
                    ghostPosition.linearSpeed = Max(ghostPosition.linearSpeed - linearAccel / FREQ_ECH_QEI, 0);
                } else if (distanceRestante <= 0 && ghostPosition.linearSpeed < 0) {
                    ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, 0);
                }

                if (Abs(distanceRestante) < Abs(incremntLin)) {
                    incremntLin = distanceRestante;
                }
            }


        }

        if ((Abs(distanceRestante) < 0.0001)) {
            ghostPosition.linearSpeed = 0;
            ghostPosition.x = ghostPosition.targetX;
            ghostPosition.y = ghostPosition.targetY;
            
//            ghostPosition.theta = robotState.angleRadianFromOdometry;
//            ghostPosition.x = robotState.xPosFromOdometry;
//            ghostPosition.y = robotState.yPosFromOdometry;
                    
            current_state = IDLE;
            mooving = 0;
        }
        
        ghostPosition.x += incremntLin * cos(ghostPosition.theta);
        ghostPosition.y += incremntLin * sin(ghostPosition.theta);
        robotState.consigneVitesseLineaire = ghostPosition.linearSpeed;
    }
    SendGhostData();
}

void SendGhostData() {
    unsigned char ghostPayload[32];
    getBytesFromInt32(ghostPayload, 0, timestamp);
    getBytesFromFloat(ghostPayload, 4, (float) ghostPosition.angleToTarget);
    getBytesFromFloat(ghostPayload, 8, (float) ghostPosition.distanceToTarget);
    getBytesFromFloat(ghostPayload, 12, (float) ghostPosition.theta);
    getBytesFromFloat(ghostPayload, 16, (float) ghostPosition.angularSpeed);
    getBytesFromFloat(ghostPayload, 20, (float) ghostPosition.x);
    getBytesFromFloat(ghostPayload, 24, (float) ghostPosition.y);
    getBytesFromFloat(ghostPayload, 28, (float) ghostPosition.linearSpeed);
    UartEncodeAndSendMessage(GHOST_DATA, 32, ghostPayload);
}