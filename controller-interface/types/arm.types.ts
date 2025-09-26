// Denavit-Hartenberg parameters for robot kinematics
export interface DHParameters {
  theta: number;  // Joint angle
  d: number;      // Link offset
  a: number;      // Link length
  alpha: number;  // Link twist
}

// Individual joint data structure
export interface JointData {
  isCalibrated: boolean;
  currentAngle: number;
  targetAngle: number;
  dHParameters: DHParameters;
  minAngle: number;
  maxAngle: number;
}

// Complete arm state
export interface ArmState {
  joints: JointData[];
  isDriverActive: boolean;
  debugMode: boolean;
}

// Robot configuration from config file
export interface RobotConfig {
  axes: number;
  joints: JointData[];
}
