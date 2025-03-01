package frc.robot.utils

import com.pathplanner.lib.config.PIDConstants
import edu.wpi.first.math.controller.PIDController

public fun PIDController(constants: PIDConstants): PIDController {
    return PIDController(constants.kP, constants.kI, constants.kD)
}