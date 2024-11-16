#include "robot_class.hpp"

int main() {
    RobotNamespace::PhysicalAttributes alpha_specs{15.0, 2.5, 4};
    RobotNamespace::RobotClass AlphaBot("AlphaBot", 1.5, alpha_specs);

    RobotNamespace::PhysicalAttributes beta_specs{10.0, 2.0, 3};
    RobotNamespace::RobotClass BetaBot("BetaBot", 2.0, beta_specs);

    // Operations with AlphaBot
    AlphaBot.moveForward(3.0);
    AlphaBot.stop();

    // Operations with BetaBot
    BetaBot.moveBackward(2.0);
    BetaBot.stop();

    return 0;
}
