#ifndef GAITSCHEDULER_HPP
#define GAITSCHEDULER_HPP

#include <array>

class GaitScheduler {
public:
    GaitScheduler(double gaitPeriod=1, double switchingPhase=0.5, double phaseOffset=0, double initialPhase=0, double dt_=0.001);

    // Run one scheduler step
    void run();

    // Accessors
    double getPhase() const { return phase; }
    double getGaitPeriod() const { return gaitPeriod; }
    int getContactState() const { return contactState; }
    int getContactStatePrev() const { return contactStatePrev; }
    double getPhaseSwing() const { return phaseSwing; }
    double getSwingPeriod() const { return Tsw; }
    double getTimeSwingRemaining() const { return timeSwingRemaining; }
    double getPhaseStance() const { return phaseStance; }
    double getStancePeriod() const { return Tc; }
    double getTimeStanceRemaining() const { return timeStanceRemaining; }
    bool getTouchDown() const { return touchDown; }
    bool getLiftOff() const { return liftOff; }
    double getSwitchingPhase() const { return switchingPhase; }

private:
    // Timing parameters
    double Tsw;
    double Tc;
    double dt;
    double gaitPeriod;
    double phaseOffset;
    double switchingPhase;

    // State variables
    double phase;
    double initialPhase;
    double dphase;
    int contactStatePrev;
    int contactState;

    // Embedded gait state
    double phaseSwing;
    double timeSwingRemaining;
    double phaseStance;
    double timeStanceRemaining;
    bool touchDown;
    bool liftOff;
};

class Gait {
public:
    Gait(double dt_=0.001);
    void run();
    void switchGait(int newGait);

    double getGaitPeriod() const { return _gaitPeriod; }
    double getSwitchingPhase() const { return _switchingPhase; }
    double getSwingPeriod() const { return _swingPeriod; }
    double getStancePeriod() const { return _stancePeriod; }
    double getPhase(int leg) const { return (*currentGait)[leg].getPhase(); }
    int getContactState(int leg) const { return (*currentGait)[leg].getContactState(); }
    int getContactStatePrev(int leg) const { return (*currentGait)[leg].getContactStatePrev(); }
    double getPhaseSwing(int leg) const { return (*currentGait)[leg].getPhaseSwing(); }
    double getTimeSwingRemaining(int leg) const { return (*currentGait)[leg].getTimeSwingRemaining(); }
    double getPhaseStance(int leg) const { return (*currentGait)[leg].getPhaseStance(); }
    double getTimeStanceRemaining(int leg) const { return (*currentGait)[leg].getTimeStanceRemaining(); }
    bool getTouchDown(int leg) const { return (*currentGait)[leg].getTouchDown(); }
    bool getLiftOff(int leg) const { return (*currentGait)[leg].getLiftOff(); }

private:
    double dt, _gaitPeriod, _switchingPhase;
    double _stancePeriod, _swingPeriod;
    bool canGaitChange = true;
    std::array<GaitScheduler, 4> stand, trotWalk, trot, trotRun;
    std::array<GaitScheduler, 4>* currentGait;
};

#endif 