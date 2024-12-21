// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.Accelerometer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Gforce extends SubsystemBase {
    @SuppressWarnings("unused")
    private GyroIO gyro;

    private double MaxGForce = 0;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    /** Creates a new Accelerometer. */
    public Gforce(GyroIO gyro) {
        this.gyro = gyro;
    }

    public double getOverallGForce() {
        double[] xyzDps = new double[3];
        xyzDps = gyroInputs.accelerations;
        // Convert accelerometer values from m/s^2 to G (1 G = 9.81 m/s^2)
        double[] gForces = new double[3];
        for (int i = 0; i < xyzDps.length; i++) {
            gForces[i] = xyzDps[i] / 9.81;
        }

        // Calculate and return the magnitude of the G-force vector
        return Math.sqrt(Math.pow(gForces[0], 2) + Math.pow(gForces[1], 2) + Math.pow(gForces[2], 2));
    }

    @Override
    public void periodic() {
        double Gforce = getOverallGForce();
        if (Gforce > MaxGForce) {
            MaxGForce = Gforce;
            Logger.recordOutput("Accelerometer/Gforce", MaxGForce);
        }

        // This method will be called once per scheduler run
    }
}
