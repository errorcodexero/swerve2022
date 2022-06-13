package org.xero1425.base.swerve.common;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.Subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class SwerveBaseSubsystem extends DriveBaseSubsystem {
    private int plotid_ ;
    private double plotstart_ ;
    private Double[] plotdata_ ;
    private static final String [] columns_ = {
        "time",
        "fl-ang-t (deg)", "fl-ang-a (deg)","fl-drv-t (m/s)","fl-drv-a (m/s)",
        "fr-ang-t (deg)", "fr-ang-a (deg)","fr-drv-t (m/s)","fr-drv-a (m/s)",
        "bl-ang-t (deg)", "bl-ang-a (deg)","bl-drv-t (m/s)","bl-drv-a (m/s)",
        "br-ang-t (deg)", "br-ang-a (deg)","br-drv-t (m/s)","br-drv-a (m/s)",
    } ;

    private boolean pathing_ ;
    private NetworkTableEntry robot_loc_ ;
    private NetworkTableEntry path_loc_ ;
    private double [] path_values_ ;
    private int index_ ;

    private SwerveDriveKinematics kinematics_ ;
    private SwerveDriveOdometry odometry_ ;

    private double width_ ;
    private double length_ ;
    private double rotate_angle_ ;
   
    static public final int FL = 0;                                                             // Index of the front left module
    static public final int FR = 1;                                                             // Index of the front right module
    static public final int BL = 2;                                                             // Index of the back left module
    static public final int BR = 3;                                                             // Index of the back right module

    public SwerveBaseSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name) ;

        plotdata_ = new Double[columns_.length] ;
        plotid_ = -1 ;
       
        pathing_ = false ;
        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;
        NetworkTable table = inst.getTable("XeroLocation") ;
        robot_loc_ = table.getEntry("Robot") ;
        path_loc_ = table.getEntry("Path") ;
        path_values_ = new double[3] ;

        width_ = getSettingsValue("physical:width").getDouble() ;
        length_ = getSettingsValue("physical:length").getDouble() ;

        //
        // Calculate the angle for the velocity vector to rotate the robot
        //
        rotate_angle_ = Math.toDegrees(Math.atan2(length_, width_)) ;

        
        kinematics_ = new SwerveDriveKinematics(new Translation2d(getWidth() / 2.0, getLength() / 2.0), new Translation2d(getWidth() / 2.0, -getLength() / 2.0), 
                        new Translation2d(-getWidth() / 2.0, getLength() / 2.0), new Translation2d(-getWidth() / 2.0, -getLength() / 2.0)) ;

        odometry_ = new SwerveDriveOdometry(kinematics_, getHeading()) ;

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        shuffleboardTab.addNumber("Heading", () -> getHeading().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> getPose().getX());
        shuffleboardTab.addNumber("Pose Y", () -> getPose().getY());
    }

    // Control the swerve drive by settings a ChassisSppeds object
    public abstract void drive(ChassisSpeeds speeds) ;

    // Control the swerve drive by supplying raw targets.  If power is true then the values
    // supplied by the speeds_power array are power numbers to go directly to the drive motors.
    // If power is false, these are speed numbers to feed into the drive motor PID controller 
    // for the modules.
    public abstract void setRawTargets(boolean power, double [] angles, double [] speeds_powers) ;

    public abstract SwerveModuleState getModuleState(int which) ;

    public abstract SwerveModuleState getModuleTarget(int which) ;

    @Override
    public void computeMyState() throws Exception {
        odometry_.update(getHeading(), getModuleState(FL), getModuleState(FR), getModuleState(BL), getModuleState(BR));
    }

    @Override
    public void run() throws Exception {
    }

    protected SwerveDriveKinematics getKinematics() {
        return kinematics_ ;
    }

    protected SwerveDriveOdometry getOdometry() {
        return odometry_ ;
    }
    
    public Pose2d getPose() {
        return odometry_.getPoseMeters() ;
    }

    public void zeroGyro() {
        odometry_.resetPosition(
                new Pose2d(odometry_.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)), getHeading()) ;
    }

    public void resetOdometry(Pose2d pose) {
        odometry_.resetPosition(pose, getHeading()) ;
    }

    public int getModuleCount() {
        return 4 ;
    }
    
    /// \brief return the base angle for a vector to rotate the robot.
    /// \returns the base angle for a vector to rotate the robot
    public double getPHI() {
        return rotate_angle_ ;
    }

    public double getWidth() {
        return width_ ;
    }

    public double getLength() {
        return length_ ;
    }
   
    public void startPathing() {
        if (!DriverStation.isFMSAttached())
        {
            pathing_ = true ;
        }
    }

    public void setPathLocation(Pose2d loc) {
        if (pathing_) {
            path_values_[0] = loc.getX() ;
            path_values_[1] = loc.getY() ;
            path_values_[2] = loc.getRotation().getDegrees() ;
            path_loc_.setDoubleArray(path_values_) ;

            Pose2d here = getPose() ;
            System.out.print("Swerve:") ;
            System.out.print(" OX " + here.getX()) ;
            System.out.print(" OY " + here.getY()) ;
            System.out.print(" PX " + loc.getX()) ;
            System.out.print(" PY " + loc.getY()) ;
            System.out.println() ;
        }
    }

    public void setRobotLocation(Pose2d loc) {
        if (pathing_) {
            path_values_[0] = loc.getX() ;
            path_values_[1] = loc.getY() ;
            path_values_[2] = loc.getRotation().getDegrees() ;
            robot_loc_.setDoubleArray(path_values_) ;
        }
    }

    public void endPathing() {
        pathing_ = false ;
    }

    public void startSwervePlot(String name) {
        if (plotid_ == -1) {
            plotid_ = initPlot(name) ;
        }

        startPlot(plotid_, columns_);
        plotstart_ = getRobot().getTime();
    }

    public void endSwervePlot() {
        endPlot(plotid_);
        plotid_ = -1 ;
    }    

    protected void newPlotData() {
        index_ = 0 ;
        
        putData(getRobot().getTime() - plotstart_) ;

        putData(getModuleTarget(FL).angle.getDegrees()) ;
        putData(getModuleState(FL).angle.getDegrees()) ;
        putData(getModuleTarget(FL).speedMetersPerSecond) ;
        putData(getModuleState(FL).speedMetersPerSecond) ;

        putData(getModuleTarget(FR).angle.getDegrees()) ;
        putData(getModuleState(FR).angle.getDegrees()) ;
        putData(getModuleTarget(FR).speedMetersPerSecond) ;
        putData(getModuleState(FR).speedMetersPerSecond) ;

        putData(getModuleTarget(BL).angle.getDegrees()) ;
        putData(getModuleState(BL).angle.getDegrees()) ;
        putData(getModuleTarget(BL).speedMetersPerSecond) ;
        putData(getModuleState(BL).speedMetersPerSecond) ;
        
        putData(getModuleTarget(BR).angle.getDegrees()) ;
        putData(getModuleState(BR).angle.getDegrees()) ;
        putData(getModuleTarget(BR).speedMetersPerSecond) ;
        putData(getModuleState(BR).speedMetersPerSecond) ;

        addPlotData(plotid_, plotdata_);
    }

    private void putData(double d) {
        plotdata_[index_++] = d ;
    }
}
