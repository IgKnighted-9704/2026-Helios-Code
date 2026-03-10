package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public class SubsystemConstants {

        public static class LightSensor{
            public static final int INTAKE_SENSOR_ID = 0;
            public static final int INDEXER_SENSOR_ID = 1;
            public static final int HOPPER_MAX_LIMIT_SENSOR_ID = 2;
        }

        public static class Vision{
            public static final String CAM_LIMELIGHT = "limeLight-knight";
                public static final double LL_X_MULTIPLIER = 1;
                public static final double LL_Y__MULTIPLIER = 1;
                public static final double LL_Z_MULTIPLIER = 1;
            
            public static final double PP_MAX_VELOCITY = 0;
            public static final double PP_MAX_ACCELERATION = 0;
            public static final double PP_MAX_ANGULAR_VELOCITY = 0;
            public static final double PP_MAX_ANGULAR_ACCELERATION = 0;

        }

        public static class ClimbSubsystemConstants{
            public static final int CLIMBMOTOR_A = 0;
            public static final int CLIMBMOTOR_B = 0;

            //SPEED CONSTANTS
                public static final double CLIMB_MANUAL_SPEED = 0;
                public static final double CLIMB_MAX_VELOCITY = 0;
                public static final double CLIMB_MAX_ACCELERATION = 0;
            //PID - Climb
                public static double CLIMB_kP = 0;
                public static double CLIMB_kI = 0;
                public static double CLIMB_kD = 0;
            //FEEDFORWARD - Climb
                public static double CLIMB_kS = 0;
                public static double CLIMB_kG = 0;
                public static double CLIMB_kV = 0;
                public static double Climb_kA = 0;

            //HARDWARE CONSTANTS
                public static double CLIMB_WHEEL_RADIUS_INCHES = 0;
            
            //CLIMB HEIGHT CONSTRAINTS
                public static double MAX_INCHES = 0;
                public static double MIN_INCHES = 0;
        
        }

        public static class IntakeSubsystemConstants{
            public static final int INTAKE_MOTOR_ID = 18;
            public static final int INTAKE_PIVOT_MOTOR_ID = 22;

            //HARDWARE CONSTANTS
                public static final double SLIDER_OFFSET = 0;
                public static final double ABS_ENC_2_INCHES_RATIO = 0;

            //SPEED CONSTANTS
                public static final double INTAKE_SPEED = 0;
                public static final double OUTTAKE_SPEED = 0;

            //PID - Slider
                public static double INTAKE_SLIDER_kP = 0;
                public static double INTAKE_SLIDER_kI = 0;
                public static double INTAKE_SLIDER_kD = 0;
            
            //Slider Macros
                public static double INTAKE_SLIDER_INCHES = 0;
                public static double OUTTAKE_SLIDER_INCHES = 0;
                public static double STOW_SLIDER_INCHES = 0;
        }

        public static class HopperSubsystemConstants{
            public static final int HOPPER_ID_A = 20;
            public static final int HOPPER_ID_B = 21;
            public static final int KICKER_MOTOR_ID = 17;
            
            //SPEED CONSTANTS
                public static final double INDEXER_SPEED = 0;
                public static final double HOPPER_SPEED = 0;

        }

        public static class ShooterSubsystemConstants{
            public static final int APRILTAG_TARGET_FIDUCIALID = 0;
            public static final int SHOOTER_ANGLE_ID = 23;
            public static final int SHOOTER_ID_A = 13;
            public static final int SHOOTER_ID_B = 14;
            public static final int SHOOTER_ID_C = 15;
            public static final int SHOOTER_ID_D = 16;
            //PID - Angle
                public static double SHOOTER_ANGLE_kP = 0;
                public static double SHOOTER_ANGLE_kI = 0;
                public static double SHOOTER_ANGLE_kD = 0;
            //PID - Speed
                public static double SHOOTER_SPEED_kP = 0;
                public static double SHOOTER_SPEED_kI = 0;
                public static double SHOOTER_SPEED_kD = 0;
            //FEEDFORWARD - Speed
                public static double SHOOTER_SPEED_kS = 0;
                public static double SHOOTER_SPEED_kV = 0;
                public static double SHOOTER_SPEED_kA = 0;
            //HARDWARE CONSTANTS
                public static double KRAKEN2FLYWHEEL_GEAR_RATIO = 1.5; // 3 : 2, 3 rotations of the flywheel = 2 rotations of the kraken
                public static double FLYWHEEL_RADIUS_METERS = Units.inchesToMeters(2);
                public static double SHOOTER_ANGLE_OFFSET = 0;
            //SHOOTER CONSTRAINTS
                public static double MAX_ANGLE = 0;
                public static double MIN_ANGLE = 0;
                public static double SHOOTER_LOW_SPEED = 45;
                public static double SHOOTER_HIGH_SPEED = 45;
                public static double DISTANCE_SHORT = 0;
                public static double DISTANCE_FAR = 0;
                public static double SPEED_TOLERANCE = 0;
                public static double ANGLE_TOLERANCE = 0;
            //PERIODIC CONSTANTS
                public static boolean desiredVelReached = false;
                public static boolean desiredAngleReached = false;
        }
}