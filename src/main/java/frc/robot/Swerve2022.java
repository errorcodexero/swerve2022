// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.automodes.SwerveDriveRobotAutoController;
import frc.robot.subsystems.Swerve2022RobotSubsystem;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.misc.SimArgs;
import org.xero1425.misc.XeroPathType;
import org.xero1425.simulator.engine.ModelFactory;
import org.xero1425.simulator.engine.SimulationEngine;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Swerve2022 extends XeroRobot {
    public Swerve2022() {
        super(0.02);
    }

    protected void addRobotSimulationModels() {
        ModelFactory factory = SimulationEngine.getInstance().getModelFactory();
        factory.registerModel("conveyor", "frc.models.ConveyorModel");
        factory.registerModel("swerve2022oi", "frc.models.Swerve2022OIModel") ;
        factory.registerModel("turret", "frc.models.TurretModel");
        factory.registerModel("hood", "frc.models.HoodModel") ;
        factory.registerModel("shooter", "frc.models.ShooterModel") ;
    }    

    public String getName() {
        return "swerve2022";
    }

    public String getSimulationFileName() {
        String ret = SimArgs.InputFileName;
        if (ret != null)
            return ret;

        return "teleop-collect-one-one-fire";
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
        Swerve2022RobotSubsystem robot = new Swerve2022RobotSubsystem(this);
        setRobotSubsystem(robot);
    }

    protected XeroPathType getPathType() {
        return XeroPathType.SwerveHolonomic ;
    }
}
