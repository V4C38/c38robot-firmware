// Base command interface
export interface BaseCommand {
  type: string;
  uuid: string;
  command: string;
}

// Command types
export interface HomingSequenceCommand extends BaseCommand {
  command: 'homingSequence';
  axis: number;
}

export interface SetAxisAngleCommand extends BaseCommand {
  command: 'setAxisAngle';
  axis: number;
  angle: number;
}

export interface GetStateCommand extends BaseCommand {
  command: 'getState';
}

export interface EmergencyStopCommand extends BaseCommand {
  command: 'emergencyStop';
}

export interface SetArmStateCommand extends BaseCommand {
  command: 'setArmState';
  state: any; // Will be defined as ArmState when implemented
}

export interface RunTestCommand extends BaseCommand {
  command: 'runTest';
  testIndex: number;
}

// Union type for all commands
export type Command = 
  | HomingSequenceCommand 
  | SetAxisAngleCommand 
  | GetStateCommand 
  | EmergencyStopCommand 
  | SetArmStateCommand 
  | RunTestCommand;

// Response interfaces
export interface BaseResponse {
  type: 'response';
  uuid: string;
  command: string;
  status: 'success' | 'error';
  error?: string;
}

export interface StateUpdateResponse extends BaseResponse {
  stateUpdate?: {
    axes: Record<string, number>;
  };
}

// Command configuration from JSON
export interface CommandConfig {
  commands: Record<string, {
    type: string;
    description: string;
    parameters?: Record<string, {
      type: string;
      description: string;
      min?: number;
      max?: number;
      required: boolean;
    }>;
  }>;
  testSequences: Array<{
    id: number;
    name: string;
    description: string;
  }>;
}
