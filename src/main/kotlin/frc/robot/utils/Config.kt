package frc.robot.utils
import edu.wpi.first.hal.HALUtil

class Config<T>(testBot:T,realBot:T) {

    companion object {
        val teamNumber = HALUtil.getTeamNumber();
    }

    val config:T;

    init {
        if (teamNumber == 9999) {
            config = testBot
        } else {
            config = realBot
        }
    }
}