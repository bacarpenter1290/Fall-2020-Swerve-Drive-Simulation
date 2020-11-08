/**
 * This class controls the elevator
 */

package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class Elevator {
    // int ballCount = 0;
    DigitalInput intakeLimit;
    DigitalInput shooterLimit;
    CANSparkMax elevator;
    CANSparkMax elevatorHelper;
    CANSparkMax conveyer;
    Joystick m_OperatorStick;

    long startTime;
    long elapsedTime;

    double elevatorSpeed;
    double elevatorHelperSpeed;
    double conveyerSpeed;
    CANSparkMax shooter;
    double shooterSpeed;
    int shooterStage = 0;
    int elevatorStage = 0;

    public Elevator(DigitalInput intakeLimit, DigitalInput shooterLimit,
                    CANSparkMax conveyer, CANSparkMax elevator, CANSparkMax elevatorHelper, 
                    Joystick m_OperatorStick, CANSparkMax shooter) {
        this.intakeLimit = intakeLimit;
        this.shooterLimit = shooterLimit;
        this.elevator = elevator;
        this.shooter = shooter;
        this.conveyer = conveyer;
        this.elevatorHelper = elevatorHelper;
        this.m_OperatorStick = m_OperatorStick;
    }

    // start the elevator to shoot 
    public void runElevator(double conveyerSpeed, double elevatorSpeed, double elevatorHelperSpeed) {

        elapsedTime = System.currentTimeMillis() - startTime;

        if(intakeLimit.get() && !shooterLimit.get() && elevatorStage == 0 && shooterStage == 0) {
            startElevator(elevatorSpeed, elevatorHelperSpeed, conveyerSpeed);
        }

        shoot();
        runElevatorOneBall();

    }

    // reset the timer to 0 ms
    public void resetTimer() {
        startTime = System.currentTimeMillis();
    }

    // run the conveyer for 2 seconds
    public void runConveyer(double conveyerSpeed, double elevatorHelperSpeed) {
        conveyer.set(conveyerSpeed);
        elevatorHelper.set(elevatorHelperSpeed);

        Timer.waitMilli(2000);

        conveyer.set(0);
        elevatorHelper.set(0);
    }

    // initalize the elevator and set the stage to 1
    public void startElevator(double elevatorSpeed, double elevatorHelperSpeed, double conveyerSpeed) {
        this.elevatorSpeed = elevatorSpeed;
        this.elevatorHelperSpeed = elevatorHelperSpeed;
        this.conveyerSpeed = conveyerSpeed;
        elevatorStage = 1;
        resetTimer();
    }

    // run the elevator up one ball
    public void runElevatorOneBall() {
        if (elevatorStage == 1) {
            elevator.set(elevatorSpeed);
            elevatorHelper.set(elevatorHelperSpeed);
            conveyer.set(conveyerSpeed);
            elevatorStage = 2;
            
        } else if (elevatorStage == 2) {
            elevator.set(elevatorSpeed);
            elevatorHelper.set(elevatorHelperSpeed);
            conveyer.set(conveyerSpeed);
            if(elapsedTime > 2500 || shooterLimit.get())
                elevatorStage = 0;
        } else if (elevatorStage == 0 && shooterStage == 0) {
            elevator.set(0);
            elevatorHelper.set(0);
            conveyer.set(0);
        }
    }

    // run the elevator all the way up until a ball hits the limit switch below the shooter
    public void runElevatorUp (double elevatorSpeed, double elevatorHelperSpeed, double conveyerSpeed) {
        boolean stop = false;
        while(!stop) {
            elevator.set(elevatorSpeed);
            elevatorHelper.set(elevatorHelperSpeed);
            conveyer.set(conveyerSpeed);
            if(shooterLimit.get()) {
                stop = true;
            }

            if(m_OperatorStick.getRawButton(11)) {
                stop = true;
            }
        }

        elevator.set(0);
        elevatorHelper.set(0);
        conveyer.set(0);

    }

    // run the shooter
    public void startShooter (double elevatorSpeed, double elevatorHelperSpeed, double conveyerSpeed, double shooterSpeed) {
        shooterStage = 1;
        resetTimer();
        this.elevatorSpeed = elevatorSpeed;
        this.elevatorHelperSpeed = elevatorHelperSpeed;
        this.conveyerSpeed = conveyerSpeed;
        this.shooterSpeed = shooterSpeed;
    }

    // automatically run the elevator to shoot the top ball
    public void shoot () {
        if(shooterStage == 1) {
            elevatorHelper.set(elevatorHelperSpeed);
            elevator.set(elevatorSpeed);
            conveyer.set(conveyerSpeed);
            shooterStage = 2;
        } else if (shooterStage == 2) {
            elevatorHelper.set(elevatorHelperSpeed);
            elevator.set(elevatorSpeed);
            conveyer.set(conveyerSpeed);
            if(elapsedTime > 5000) {
                shooterStage = 0;
            }
            if (shooterLimit.get())
                shooterStage++;
        } else if (shooterStage == 3) {
            elevator.set(0);
            elevatorHelper.set(0);
            conveyer.set(0);
            
            shooter.set(shooterSpeed);
            resetTimer();
            shooterStage = 4;
        } else if (shooterStage == 4) {
            shooter.set(shooterSpeed);
            // System.out.println(shooterSpeed);
            if (elapsedTime > 2000) {
                shooterStage = 5;
                resetTimer();
            }
        } else if (shooterStage == 5) {
            elevatorHelper.set(elevatorHelperSpeed);
            elevator.set(elevatorSpeed);
            conveyer.set(conveyerSpeed);
            shooter.set(shooterSpeed);
            // System.out.println(elapsedTime);
            if(elapsedTime > 2000) {
                shooterStage = 6;
                resetTimer();
            }
        } else if (shooterStage == 6) {
            elevator.set(0);
            elevatorHelper.set(0);
            conveyer.set(0);
            shooter.set(0);
            if (elapsedTime > 2000)
                shooterStage = 0;
        } else if (shooterStage == 0 && elevatorStage == 0) {
            elevator.set(0);
            elevatorHelper.set(0);
            conveyer.set(0);
            shooter.set(0);
            // System.out.println(shooter.get());
        }
        // System.out.println(shooter.get());
        // System.out.println("Shooter Stage " + shooterStage);
    }

 }
