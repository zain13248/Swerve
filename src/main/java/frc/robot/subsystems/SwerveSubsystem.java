package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
            );

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

 
    public SwerveModulePosition[] getModulePositions(){

        return( new SwerveModulePosition[]{
          frontLeft.getPosition(),
          frontRight.getPosition(), 
          backLeft.getPosition(),
          backRight.getPosition()});
    
      } 

      
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer;  
    
    public SwerveSubsystem() {


        odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getOdometryAngle(), getModulePositions());




    try {
            zeroHeading();
    } catch (Exception e) {
            }
    }
    
    


public Rotation2d getInitRotation2d(SwerveModule swrvMod) { 
    return Rotation2d.fromRadians(swrvMod.getAbsoluteEncoderRad()); }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());

    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getOdometryAngle(), getModulePositions(), pose);
       }

       public Rotation2d getOdometryAngle(){
        return(Rotation2d.fromDegrees(gyro.getYaw()));
      }   
    
    @Override
     public void periodic() {
        // Used for Odometry purposes only, does not affect Teleop
    
        SwerveModulePosition lf = new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition()));
        SwerveModulePosition rf = new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition()));
        SwerveModulePosition lb = new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition()));
        SwerveModulePosition rb = new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition()));
    
        odometer.update(getRotation2d(),
                new SwerveModulePosition[]{
                        lf, rf, lb, rb
                });
    
        SmartDashboard.putNumber("Gyro:", getHeading());
        SmartDashboard.putNumber("Left Front Swerve: ", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("Right Front Swerve: ", frontRight.getTurningPosition());
        SmartDashboard.putNumber("Left Back Swerve: ", backLeft.getTurningPosition());
        SmartDashboard.putNumber("Right Back Swerve: ", backRight.getTurningPosition());
    
        SmartDashboard.putNumber("Left Front Swerve Encoder: ", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Right Front Swerve Encoder: ", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Left Back Swerve Encoder: ", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Right Back Swerve Encoder: ", backRight.getAbsoluteEncoderRad());
    
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }    



    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    public void setWheelState( ){

    }
}