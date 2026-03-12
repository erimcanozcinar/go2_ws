#include "legged_trajectory/GaitScheduler.hpp"
#include <cmath>  // fmod, abs
#include <iostream>

GaitScheduler::GaitScheduler(double duration, double switchingPhaseNominal, double phase_offset, double initialPhase, double dt_)
    : dt(dt_), contactStatePrev(1), contactState(1),
      phaseSwing(0.0), timeSwingRemaining(0.0), phaseStance(0.0), timeStanceRemaining(0.0),
      touchDown(0), liftOff(0), phaseOffset(phase_offset), gaitPeriod(duration), switchingPhase(switchingPhaseNominal)
{
    Tc = switchingPhase*gaitPeriod;
    Tsw = gaitPeriod - Tc;

    // gaitPeriod = Tsw + Tc;
    // switchingPhase = Tc / gaitPeriod;
    dphase = dt / gaitPeriod;
    // initialPhase = 0.0; //(Tsw * 0.5) / gaitPeriod - dphase;
    phase = initialPhase + phaseOffset - dphase;
}

void GaitScheduler::run()
{
    // phase = std::fmod(phase + dphase, 1.0 + dphase); // This method cause 0.001 sec drift in each phase
    phase = phase + dphase;
    contactStatePrev = contactState;
    if(phase > 0.999) phase = phase - 1.0;
    // Stance phase
    if (phase <= switchingPhase) {
        contactState = 1;
        phaseStance = (phase) / switchingPhase;
        timeStanceRemaining = gaitPeriod * (switchingPhase - phase);
        liftOff = 0;
        touchDown = (contactStatePrev == 0) ? 1 : 0;
    } else {
        phaseStance = 0.0;
        timeStanceRemaining = 0.0;
    }

    // Swing phase
    if (phase >= switchingPhase) {
        contactState = 0;
        phaseSwing = (phase - switchingPhase) / (1.0 - switchingPhase);
        timeSwingRemaining = gaitPeriod * (1.0 - phase);
        liftOff = (contactStatePrev == 1) ? 1 : 0;
        touchDown = 0;
    } else {
        phaseSwing = 0.0;
        timeSwingRemaining = 0.0;
    }   
    
}


Gait::Gait(double dt_)
    : dt(dt_) {
    std::cout << "Sampling rate of Gait Scheduler: " << 1/dt << " Hz" << std::endl;
    
    stand = {GaitScheduler(0.5, 1.0, 0.5, 0.0, dt),
             GaitScheduler(0.5, 1.0, 0.5, 0.0, dt),
             GaitScheduler(0.5, 1.0, 0.5, 0.0, dt),
             GaitScheduler(0.5, 1.0, 0.5, 0.0, dt)};
    trotWalk = {GaitScheduler(0.5, 0.6, 0.0, 0.0, dt),
                GaitScheduler(0.5, 0.6, 0.5, 0.0, dt),
                GaitScheduler(0.5, 0.6, 0.5, 0.0, dt),
                GaitScheduler(0.5, 0.6, 0.0, 0.0, dt)};
    trot = {GaitScheduler(0.5, 0.5, 0.0, 0.0, dt),
            GaitScheduler(0.5, 0.5, 0.5, 0.0, dt),
            GaitScheduler(0.5, 0.5, 0.5, 0.0, dt),
            GaitScheduler(0.5, 0.5, 0.0, 0.0, dt)};
    trotRun = {GaitScheduler(0.5, 0.4, 0.0, 0.0, dt),
                GaitScheduler(0.5, 0.4, 0.5, 0.0, dt),
                GaitScheduler(0.5, 0.4, 0.5, 0.0, dt),
                GaitScheduler(0.5, 0.4, 0.0, 0.0, dt)};
    
    currentGait = &trotWalk;
}

void Gait::run() {

    for(int i = 0; i < 4; i++) {
        (*currentGait)[i].run();
    }
    _gaitPeriod = (*currentGait)[0].getGaitPeriod();
    _switchingPhase = (*currentGait)[0].getSwitchingPhase();
    _stancePeriod = (*currentGait)[0].getStancePeriod();
    _swingPeriod = (*currentGait)[0].getSwingPeriod();
}

void Gait::switchGait(int newGait) {
    switch(newGait) {
        case 0:
            currentGait = &stand;
            std::cout << "Standing" << std::endl;
            break;
        case 1:
            currentGait = &trot;
            std::cout << "Trotting" << std::endl;
            break;
    }
}