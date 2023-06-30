#pragma once
#include "Parameters.h"
#include "ctre/phoenixpro/TalonFX.hpp"
using namespace ctre::phoenixpro;

class Falcon500 {
private:
    hardware::TalonFX *motor;
    controls::TorqueCurrentFOC accelerationController{0_A};
    float const maxRotationsPerSecond = parameters.falconMaxRotationsPerSecond;

public:
    Falcon500(int canID) {
        motor = new hardware::TalonFX(canID, "rio");
    }

    void initialize()
    {
        motor->SetRotorPosition(0_tr);
    }

    void SetAcceleration(float amperage) {
        motor->SetControl(accelerationController.WithOutput(amperage * 1_A));
    }

    float getPosition() {
        return motor->GetPosition().GetValue().value();
    }

    float getPercentOfMaxVelocity() {
        return motor->GetVelocity().GetValue().value() / maxRotationsPerSecond;
    }
};