/**
 * This class controls the entire swerve drive train
 */

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;

/**
 * Represents a swerve drive style drivetrain.
 */
public class Drivetrain {
  // set max speed of the wheels for safety
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // these values are based on the size of the robot and the position of the wheels
  // mechanically, swerve drive is ideal on a square chassis with wheels 
  // equidistant from the center
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  // set up SwerveModules for each of the four wheels
  // swerve modules include the turning motor and encoding as well as the drive motor and encoder (6 total DIO)
  private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 1, 2, 3, 4);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4, 5, 6, 7, 8);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6, 9, 10, 11, 12);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 13, 14, 15, 16);

  // Gyro is used to oreniate the robot based on field position (i.e. the goal is always forward)
  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  // set up odometry, a powerful class to track the state of the robot
  // very useful for future autonomous code (path following, camera tracking, etc.)
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

  // make sure to reset the gyro on startup
  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // find the desired states for each of the four swerve modules based on the correct speeds, controller input, and encoder values
    // the states are the desired orentiation of the wheel, the speed of the wheel.
    // the states of the other modules are taken into consideration as well 
    // i.e. if strafing, the current speed of each module is accounted to ensure the wheels don't go in opposing directions, potentially damaging the robot
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );

    // normalize the wheel speeds based on the desired states and the max speed (i.e. don't allow the wheels to go faster than the max speed)
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);

    // set the states of the four swerve modules accordinly
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(
        getAngle(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    );
  }
}
