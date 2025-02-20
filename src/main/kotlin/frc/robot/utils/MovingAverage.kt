package frc.robot.utils

import edu.wpi.first.wpilibj.DriverStation

class MovingAverage(private val numberValues: Int) {
    private var values: Array<Double>
    private var currentSum = 0.0
    private var index = 0

    init {
        values = Array(numberValues) { 0.0 }
    }

    fun addValue(value: Double): Double {
        currentSum += value
        index++
        index = index % numberValues
        currentSum -= values[index]
        values[index] = value
        return getAverage()
    }

    fun setAverage(value: Double) {
        currentSum = numberValues * value
        for (i in 0..(numberValues - 1)) {
            values.set(i, value)
        }
    }

    fun getAverage(): Double {
        return currentSum / numberValues
    }
}
