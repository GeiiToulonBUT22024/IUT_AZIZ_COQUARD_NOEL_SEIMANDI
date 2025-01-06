/*
 * File:   IO.c
 */

#include <xc.h>
#include "IO.h"
#include "main.h"

void InitIO()
{
    // IMPORTANT : désactiver les entrées analogiques, sinon on perd les entrées numériques
    ANSELA = 0; // 0 desactive
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    ANSELG = 0;

    // Configuration des sorties
    
    
    _TRISE3 = 0;
    _TRISE2 = 0;
    _TRISE1 = 0;
    _TRISE0 = 0;

    //******* LED ***************************
    _TRISJ6 = 0;  // LED Orange
    _TRISJ5 = 0; //LED Blanche
    _TRISJ4 = 0; // LED Bleue
    _TRISJ11 = 0; // LED Rouge
    _TRISH10 = 0; // LED Verte 

    
    //****** Moteurs ************************

    // Configuration des entrées
    _TRISH0 = 1; // Chargement de l'input sur BTN 1
    _TRISH1 = 1;
    _TRISH2 = 1;

    // Configuration des pins remappables    
    //*************************************************************
    // Unlock Registers
    //*************************************************************
    //__builtin_write_OSCCONL(OSCCON & ~(1<<6)); 
    
    UnlockIO(); // On unlock les registres d'entrées/sorties, ainsi que les registres des PPS
    
    //Assignation des remappable pins
    
    //******************** QEI *****************
    _QEA2R = 97; //assign QEI A to pin RP97
    _QEB2R = 113; //assign QEI B to pin RP96
    _QEA1R = 126; //assign QEI A to pin RP70
    _QEB1R = 124; //assign QEI B to pin RP69
    
    _U1RXR = 78; //Remappe la RP... sur l?éentre Rx1
    _RP79R = 0b00001; //Remappe la sortie Tx1 vers RP...
    _U2RXR = 98; //Remappe la RP... sur l?éentre Rx1   
    //_RP98R = 0b00001; //Remappe la sortie Tx1 vers RP...
    LockIO(); // On lock les registres d'entrées/sorties, ainsi que les registres des PPS
    
    //Assignation des remappable pins
    
    //*************************************************************
    // Lock Registers
    //*************************************************************
    //__builtin_write_OSCCONL(OSCCON | (1<<6));
    
}

void LockIO() {
    asm volatile ("mov #OSCCON,w1 \n"
                "mov #0x46, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2,[w1] \n"
                "mov.b w3,[w1] \n"
                "bset OSCCON, #6":: : "w1", "w2", "w3");
}

void UnlockIO() {
    asm volatile ("mov #OSCCON,w1 \n"
                "mov #0x46, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2,[w1] \n"
                "mov.b w3,[w1] \n"
                "bclr OSCCON, #6":: : "w1", "w2", "w3");
}