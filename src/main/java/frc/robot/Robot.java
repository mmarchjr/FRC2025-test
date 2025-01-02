// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.ironmaple.simulation.SimulatedArena;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private final Field2d m_field = new Field2d();
    private String autoName, newAutoName;

    @SuppressWarnings("resource")
    public Robot() {

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                new PowerDistribution(1, ModuleType.kRev);
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Initialize URCL
        Logger.registerURCL(URCL.startExternal());

        // Start AdvantageKit logger
        Logger.start();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        Shuffleboard.selectTab("Autonomous");
        SmartDashboard.putData("Auto Field", m_field);

        if (isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        m_field.setRobotPose(robotContainer.getDrive().getPose());
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        robotContainer.resetSimulationField();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        newAutoName = robotContainer.getAutonomousCommand().getName();
        if (autoName != newAutoName) {
            autoName = newAutoName;
            if (AutoBuilder.getAllAutoNames().contains(autoName)) {
                System.out.println("Displaying " + autoName);
                List<PathPlannerPath> pathPlannerPaths;
                try {
                    pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);

                    List<Pose2d> poses = new ArrayList<>();
                    for (PathPlannerPath path : pathPlannerPaths) {
                        poses.addAll(path.getAllPathPoints().stream()
                                .map(point ->
                                        new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d()))
                                .collect(Collectors.toList()));
                    }
                    m_field.getObject("path").setPoses(poses);
                } catch (IOException | ParseException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        Shuffleboard.selectTab("Autonomous");
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        Shuffleboard.selectTab("Teleoperated");
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
        robotContainer.displaySimFieldToAdvantageScope();
    }

    public Field2d getField() {
        return m_field;
    }
}
