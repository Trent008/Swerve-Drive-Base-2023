#pragma once
#include "frc/Joystick.h"

class XKeysPad{
private:
    frc::Joystick *joy;
    double a;

public:
    XKeysPad(frc::Joystick *joy)
    {
        this->joy = joy;
    }

    bool getSuctionCupState() {
        return joy->GetRawButton(7);
    }

    bool getHiRowPressed() {
        return joy->GetRawButtonPressed(3);
    }
    
    bool getMidRowPressed() {
        return joy->GetRawButtonPressed(2);
    }
    
    bool getLowRowPressed() {
        return joy->GetRawButtonPressed(1);
    }
    
    bool getFeederStationPressed() {
        return joy->GetRawButtonPressed(5);
    }

    bool getFloorPressed() {
        return joy->GetRawButtonPressed(4);
    }

    bool getHomePressed() {
        return joy->GetRawButtonPressed(6);
    }
    
    bool getIsCone() {
        return joy->GetRawButton(8);
    }
};