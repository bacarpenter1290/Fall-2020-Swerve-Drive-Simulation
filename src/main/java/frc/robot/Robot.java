/**
 * This program is for FRC team 2197 to simulate the 2020 robot with a swerve drive
 * It is a heavily modified version of the code planned for use at the 2020 St. Joe. compeition
 */

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  // without limiters, the robot can feel jerky when driving
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // all of the channels for the various motors and sensors
  private static final int kIntakeChannel = 3;

  private static final int kConveyerChannel = 2;
  private static final int kElevatorChannel = 6;
  private static final int kElevatorHelperChannel = 4;

  private static final int kClimberChannel = 7;
  private static final int kTransverseChannel = 5;

  private static final int kShooterChannel = 1;

  // all of the motors and sensors
  private CANSparkMax intakeMotor;

  private CANSparkMax conveyerMotor;
  private CANSparkMax elevatorMotor;
  private CANSparkMax elevatorHelperMotor;

  private CANSparkMax climberMotor;
  private CANSparkMax transverseMotor;

  private CANSparkMax shooterMotor;

  private Joystick m_operatorStick;

  private DigitalInput intakeLimit;
  private DigitalInput shooterLimit;

  private Elevator elevator;

  // initialize the robot
  @Override
  public void robotInit() {
    // initalize the motors
    intakeMotor = new CANSparkMax(kIntakeChannel, MotorType.kBrushed);

    conveyerMotor = new CANSparkMax(kConveyerChannel, MotorType.kBrushed);
    elevatorMotor = new CANSparkMax(kElevatorChannel, MotorType.kBrushed);
    elevatorHelperMotor = new CANSparkMax(kElevatorHelperChannel, MotorType.kBrushed);

    climberMotor = new CANSparkMax(kClimberChannel, MotorType.kBrushless);
    transverseMotor = new CANSparkMax(kTransverseChannel, MotorType.kBrushed);

    // shooterMotor = new WPI_TalonFX(kShooterChannel);

    shooterMotor = new CANSparkMax(kShooterChannel, MotorType.kBrushed);

    // initalize ports for limit switches
    intakeLimit = new DigitalInput(20);
    shooterLimit = new DigitalInput(21);

    m_operatorStick = new Joystick(1);

    elevator = new Elevator(intakeLimit, shooterLimit, conveyerMotor,
                       elevatorMotor, elevatorHelperMotor, m_operatorStick, shooterMotor);
  }

  @Override
  public void autonomousPeriodic() {
    //
    // TODO: add automomous code once robot is accessible again.
    //
  }

  @Override
  public void teleopPeriodic() {
    // keep field relative driving off when simulating
    // it can be disorienting when there's no physical device
    driveWithJoystick(false);

    // all of these values would come from the dashboard but
    // it won't work during simulation
    boolean manualMode = false;

    double conveyerSpeed = 1;
    double shooterSpeed = 1;
    double elevatorSpeed = 1;
    double intakeSpeed = 1;
    double transverseSpeed = 1;
    double climberSpeed = 1;
    double elevatorHelperSpeed = 1;

    // in manual, operator controls each motor individually
    // otherwise, the shooter and elevator are controlled by sensor events
    if (manualMode) {
      if (m_operatorStick.getRawButton(1)) {
        shooterMotor.set(shooterSpeed);
      } else {
        shooterMotor.set(0);
      }
  
      if(m_operatorStick.getRawButton(3)) {
        conveyerMotor.set(conveyerSpeed);
      } else {
        conveyerMotor.set(0);
      }

      if(m_operatorStick.getRawButton(4)) {
        elevatorMotor.set(elevatorSpeed);
      } else {
        elevatorMotor.set(0);
      }

      if(m_operatorStick.getRawButton(5)) {
        elevatorHelperMotor.set(elevatorHelperSpeed);
      } else {
        elevatorHelperMotor.set(0);
      }
    } else {
      if(m_operatorStick.getRawButton(1)) {
        elevator.startShooter(elevatorSpeed, elevatorHelperSpeed, conveyerSpeed, shooterSpeed);
      }

      elevator.runElevator(conveyerSpeed, elevatorSpeed, elevatorHelperSpeed);
    }

    // these motors are always controlled manually as there are no sensors
    // placed to automate their control
    if(m_operatorStick.getRawButton(2)) {
      intakeMotor.set(intakeSpeed);
    } else {
      intakeMotor.set(0);
    }

    if(m_operatorStick.getRawButton(8)) {
      transverseMotor.set(transverseSpeed);
    } else {
      transverseMotor.set(0);
    }

    if(m_operatorStick.getRawButton(9)) {
      transverseMotor.set(-1 * transverseSpeed);
    } else {
      transverseMotor.set(0);
    }

    if(m_operatorStick.getRawButton(6)) {
      climberMotor.set(climberSpeed);
    } else {
      climberMotor.set(0);
    }

    if(m_operatorStick.getRawButton(7)) {
      climberMotor.set(-1 * climberSpeed);
    } else {
      climberMotor.set(0);
    }
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(m_controller.getY(GenericHID.Hand.kLeft))
            * frc.robot.Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(m_controller.getX(GenericHID.Hand.kLeft))
            * frc.robot.Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(m_controller.getX(GenericHID.Hand.kRight))
            * frc.robot.Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
