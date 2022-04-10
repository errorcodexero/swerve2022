// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.automodes.SwerveDriveRobotAutoController;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.SimArgs;
import org.xero1425.misc.XeroPathType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Swerve2021 extends XeroRobot {
    public Swerve2021() {
        super(0.02);
    }

    public String getName() {
        return "swerve2021";
    }

    public String getSimulationFileName() {
        String ret = SimArgs.InputFileName;
        if (ret != null)
            return ret;

        return "testmode";
    }

    public AutoController createAutoController() {
        AutoController ctrl;

        try {
            ctrl = new SwerveDriveRobotAutoController(this);
        } catch (Exception ex) {
            ctrl = null;
        }

        return ctrl;
    }

    protected void hardwareInit() throws Exception {
        SwerveDriveRobotSubsystem robot = new SwerveDriveRobotSubsystem(this);
        setRobotSubsystem(robot);
    }

    protected XeroPathType getPathType() {
        return XeroPathType.SwervePathFollowing ;
    }
}
