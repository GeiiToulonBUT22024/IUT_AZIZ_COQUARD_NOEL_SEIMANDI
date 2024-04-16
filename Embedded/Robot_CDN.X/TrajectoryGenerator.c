#include <math.h>
#include <stdlib.h>
#include "trajectoryGenerator.h"
#include "timer.h"
#include "Robot.h"
#include "utilities.h"
#include "UART_Protocol.h"
#include "QEI.h"


volatile GhostPosition ghostPosition;
static unsigned long lastUpdateTime = 0;
double thetaRobot;
int newPos = 0;
int lin = 0;

double maxAngularSpeed = 2 * PI;
double angularAccel = 2 * PI;

double maxLinearSpeed = 1;
double minMaxLinenearSpeed = 0.2;
double linearAccel = 1;

void InitTrajectoryGenerator(void) {
    ghostPosition.x = 0.0;
    ghostPosition.y = 0.0;
    ghostPosition.theta = 0.0;
    ghostPosition.linearSpeed = 0.0;
    ghostPosition.angularSpeed = 0.0;
    ghostPosition.targetX = 0.0;
    ghostPosition.targetY = 0.0;
    ghostPosition.angleToTarget = 0.0;
    ghostPosition.distanceToTarget = 0.0;
    ghostPosition.state = IDLE;
}

void UpdateTrajectory() // Mise a jour de la trajectoire en fonction de l'etat actuel
{
    ///Calcul de quelques variables interm��diaires qui pourraient servir...
    // Angle de la cible

    double targetAngle = atan2(ghostPosition.targetY - ghostPosition.y, ghostPosition.targetX - ghostPosition.x);
    double angleAParcourir = ModuloByAngle(ghostPosition.theta, targetAngle - ghostPosition.theta);
    double angleArret = ghostPosition.angularSpeed * ghostPosition.angularSpeed / (2 * angularAccel);
    double distanceAParcourir = sqrt((ghostPosition.targetX - ghostPosition.x)*(ghostPosition.targetX - ghostPosition.x)
            +(ghostPosition.targetY - ghostPosition.y)*(ghostPosition.targetY - ghostPosition.y));
    double distanceArret = ghostPosition.linearSpeed * ghostPosition.linearSpeed / (2 * linearAccel);
    double vitesseLinMax = 0.5 * ((maxLinearSpeed + minMaxLinenearSpeed) + (maxLinearSpeed - minMaxLinenearSpeed) * cos(angleAParcourir));
    double rayonArretMax = 0.5 * (maxLinearSpeed + minMaxLinenearSpeed) / maxAngularSpeed;

    if (angleAParcourir != 0 && distanceAParcourir > 0.01) ///On doit tourner
    {
        //Soit l'angle � parcourir est positif 
        if (angleAParcourir > 0) {
            //Soit l'angle � parcourir est sup�rieur � la distance d'arr�t
            if (angleAParcourir > angleArret) {
                //Soit on a d�j� atteint la Vmax angulaire
                if (ghostPosition.angularSpeed >= maxAngularSpeed) {
                    //On maintient la vitesse
                } else {
                    //Soit on ne l'a pas atteint
                    //On acc�l�re avec saturation � VMax
                    ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, maxAngularSpeed);
                }
            }//Soit l'angle � parcourir est inf�rieur
            else {
                //On freine
                ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, 0);
            }
        }//Soit l'angle � parcourir est n�gatif 
        else {
            //Soit l'angle � parcourir est sup�rieur � la distance d'arr�t
            if (abs(angleAParcourir) > angleArret) {
                //Soit on a d�j� atteint la Vmax angulaire
                if (ghostPosition.angularSpeed <= -maxAngularSpeed) {
                    //On maintient la vitesse
                } else {
                    //Soit on ne l'a pas atteint
                    //On acc�l�re avec saturation � VMax
                    ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, -maxAngularSpeed);
                }
            }//Soit l'angle � parcourir est inf�rieur
            else {
                //On freine
                ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, 0);
            }
        }

        ghostPosition.theta += ghostPosition.angularSpeed / FREQ_ECH_QEI;
        //Si la nouvelle vitesse angulaire est nulle ici
        //On a termin� la rotation, l'angle du ghost est donc l'angle de la cible (on casse les erreurs d'arrondi d'int�gration)
        if (ghostPosition.angularSpeed == 0) {
            ghostPosition.theta = targetAngle;
        }
    }

    if (distanceAParcourir != 0 && abs(angleAParcourir) < 0.5) {
        //Soit la distance � parcourir est sup�rieure � la distance d'arret
        if (distanceAParcourir > (distanceArret + ghostPosition.linearSpeed / FREQ_ECH_QEI)) // Savoir si 
        {
            //Soit on a d�j� atteint la Vmax
            if (ghostPosition.linearSpeed >= vitesseLinMax) {
                ghostPosition.linearSpeed = Max(ghostPosition.linearSpeed - linearAccel / FREQ_ECH_QEI, vitesseLinMax);
                //On fait rien
            } else {
                //On acc�l�re (en saturant � Vmax)
                ghostPosition.linearSpeed = Min(ghostPosition.linearSpeed + linearAccel / FREQ_ECH_QEI, vitesseLinMax);
            }
        }//Soit la distane � parcourir est inf�rieure � la distance d'arret
        else {
            //On freine
            ghostPosition.linearSpeed = Max(ghostPosition.linearSpeed - linearAccel / FREQ_ECH_QEI, 0);
        }

        //Si la nouvelle vitesse lin�aire est nulle ici
        //On a termin� le parcours, la position du ghost est donc la position de la cible (on casse les erreurs d'arrondi d'int�gration)
        if (distanceAParcourir <= rayonArretMax) {
            ghostPosition.x = ghostPosition.targetX;
            ghostPosition.y = ghostPosition.targetY;
            ghostPosition.linearSpeed = 0 ;
        }


                robotState.consigneLin = ghostPosition.linearSpeed;
                robotState.consigneAng = ghostPosition.angularSpeed;
    }

    double deltaParcouru = ghostPosition.linearSpeed / FREQ_ECH_QEI;
    ghostPosition.x += deltaParcouru * cos(ghostPosition.theta);
    ghostPosition.y += deltaParcouru * sin(ghostPosition.theta);
}

//void RotateTowardsTarget(double currentTime) // Orientation du Ghost vers le waypoint
//{
//    double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
//    double thetaWaypoint = atan2(ghostPosition.targetY - ghostPosition.y, ghostPosition.targetX - ghostPosition.x);
//    thetaRobot = ModuloByAngle(thetaWaypoint, ghostPosition.theta);
//    double thetaRestant = thetaWaypoint - thetaRobot;
//    double thetaRestant = thetaWaypoint - thetaRobot;
//    double thetaArret = pow(ghostPosition.angularSpeed, 2) / (2 * MAX_ANGULAR_ACCEL);
//
//    int shouldAccelerate = 0;
//    int isDirectionPositive = thetaRestant > 0 ? 1 : 0;
//
//    if (fabs(thetaRestant) < ANGLE_TOLERANCE) {
//        ghostPosition.state = ADVANCING;
//        ghostPosition.angularSpeed = 0;
//        return;
//    }
//
//    if (isDirectionPositive) {
//        shouldAccelerate = (ghostPosition.angularSpeed < MAX_ANGULAR_SPEED && thetaRestant > thetaArret) ? 1 : 0;
//    } else {
//        shouldAccelerate = (ghostPosition.angularSpeed > -MAX_ANGULAR_SPEED && fabs(thetaRestant) > thetaArret) ? 1 : 0;
//    }
//
//    if (shouldAccelerate) {
//        ghostPosition.angularSpeed += (isDirectionPositive ? 1 : -1) * MAX_ANGULAR_ACCEL * deltaTime;
//    } else if (fabs(thetaRestant) <= thetaArret || (!isDirectionPositive && ghostPosition.angularSpeed > 0) || (isDirectionPositive && ghostPosition.angularSpeed < 0)) {
//        ghostPosition.angularSpeed -= (isDirectionPositive ? 1 : -1) * MAX_ANGULAR_ACCEL * deltaTime;
//    }
//
//    if (ghostPosition.angularSpeed > MAX_ANGULAR_SPEED) ghostPosition.angularSpeed = MAX_ANGULAR_SPEED;
//
//    ghostPosition.angularSpeed = fmin(fmax(ghostPosition.angularSpeed, -MAX_ANGULAR_SPEED), MAX_ANGULAR_SPEED);
//    ghostPosition.theta += ghostPosition.angularSpeed * deltaTime;
//    ghostPosition.theta = ModuloByAngle(0, ghostPosition.theta);
//
//    ghostPosition.angleToTarget = thetaRestant;
//}
//
//void AdvanceTowardsTarget(double currentTime) // Avancement lineaire du Ghost vers le waypoint
//{
//    ghostPosition.theta = thetaRobot;
//    double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
//    double distance = DistanceProjete(ghostPosition.x, ghostPosition.y,
//            ghostPosition.x + cos(ghostPosition.theta), ghostPosition.y + sin(ghostPosition.theta),
//            ghostPosition.targetX, ghostPosition.targetY); // distance par rapport au projete sur la droite de direction
//
//    if (distance < DISTANCE_TOLERANCE) {
//        ghostPosition.state = IDLE;
//        ghostPosition.linearSpeed = 0.0;
//        ghostPosition.targetX = ghostPosition.x;
//        ghostPosition.targetY = ghostPosition.y;
//
//        return;
//    }
//
//    double accelDistance = (MAX_LINEAR_SPEED * MAX_LINEAR_SPEED - ghostPosition.linearSpeed * ghostPosition.linearSpeed) / (2 * MAX_LINEAR_ACCEL);
//    double decelDistance = (ghostPosition.linearSpeed * ghostPosition.linearSpeed) / (2 * MAX_LINEAR_ACCEL);
//
//    if (distance <= (decelDistance + DISTANCE_TOLERANCE)) {
//        // Phase de deceleration
//        ghostPosition.linearSpeed = (MAX_LINEAR_SPEED * distance) / decelDistance;
//    } else if (distance > decelDistance + accelDistance) {
//        ghostPosition.linearSpeed += MAX_LINEAR_ACCEL * deltaTime;
//        ghostPosition.linearSpeed = fmin(ghostPosition.linearSpeed, MAX_LINEAR_SPEED);
//    } else {
//        double vMedian = sqrt(MAX_LINEAR_ACCEL * distance + ghostPosition.linearSpeed / 2);
//        ghostPosition.linearSpeed += MAX_LINEAR_ACCEL * deltaTime;
//        ghostPosition.linearSpeed = fmin(ghostPosition.linearSpeed, vMedian);
//    }
//
//    if (ghostPosition.linearSpeed > MAX_LINEAR_SPEED) ghostPosition.linearSpeed = MAX_LINEAR_SPEED;
//
//
//    ghostPosition.distanceToTarget = distance;
//
//    // Deplacer le robot vers le waypoint
//    ghostPosition.x += ghostPosition.linearSpeed * cos(ghostPosition.theta) * deltaTime;
//    ghostPosition.y += ghostPosition.linearSpeed * sin(ghostPosition.theta) * deltaTime;
//    
//    newPos = 0 ;
//}

void SendGhostData() {
    unsigned char ghostPayload[24];
    getBytesFromFloat(ghostPayload, 0, (float) ghostPosition.angleToTarget);
    getBytesFromFloat(ghostPayload, 4, (float) ghostPosition.distanceToTarget);
    getBytesFromFloat(ghostPayload, 8, (float) ghostPosition.theta);
    getBytesFromFloat(ghostPayload, 12, (float) ghostPosition.angularSpeed);
    getBytesFromFloat(ghostPayload, 16, (float) ghostPosition.x);
    getBytesFromFloat(ghostPayload, 20, (float) ghostPosition.y);

    UartEncodeAndSendMessage(GHOST_DATA, 24, ghostPayload);
}