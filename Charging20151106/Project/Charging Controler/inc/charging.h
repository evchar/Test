#ifndef __CHARGING_H__
#define __CHARGING_H__

/* encapsulated delcaration of the Ship active object ----------------------*/
/* @(/1/1) .................................................................*/
/**
* Ship Active Object
*/
typedef struct tag_stateTag{
/* protected: */
    QHsm super;
}Tag;


enum Signals {                              /* signals used in the game */
    Q_TOGGLE_SIG = Q_USER_SIG,                  /* published from tick ISR */
    Q_CAR_DETECTED_SIG, /* published by Player (ISR) to trigger the Missile */
    Q_CAR_REDAY_SIG,          /* published by Player (ISR) to quit the game */
    Q_START_SIG,          /* published by Ship when it finishes exploding */
    Q_STOP_SIG,  /* posted by Player (ISR) to the Ship to move it */
    Q_STANDBY_SIG,    /* from Tunnel to Ship to grant permission to take off */
    Q_FAULT_SIG,            /* from Tunnel to Ship when Ship hits the wall */
    Q_CHECK_SERVER_SIG,     /* from Mine to Ship or Missile when it hits the mine */
    Q_SENSOR_DATA_SIG,
    Q_TIMERB_SIG,
    Q_LOSS_SYNC_SIG,
    Q_NANO_LP_SIG,
    Q_NANO_WKUP_SIG
};


extern Tag Tag_;

void QHsmTagInit(void);
void QHsmMsgProcess(void);

#endif