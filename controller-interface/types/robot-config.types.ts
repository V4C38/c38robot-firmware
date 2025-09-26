export interface RobotJointConfig {
  id: number;
  name: string;
  theta: number;
  d: number;
  a: number;
  alpha: number;
  minAngle: number;
  maxAngle: number;
}

export interface RobotArmConfig {
  axes: number;
}

export interface RobotConfig {
  armConfig: RobotArmConfig;
  joints: RobotJointConfig[];
}
