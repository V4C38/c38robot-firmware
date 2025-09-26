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
  message?: string;
  error?: string;
}

export interface StateUpdateResponse extends BaseResponse {
  stateUpdate?: {
    axes: Record<string, number>;
    limits?: Record<string, { isAtLimit: boolean; limitIndex: number | null }>;
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

// Outgoing on-wire command envelope (host -> firmware)
export type CommandName =
  | 'homingSequence'
  | 'setAxisAngle'
  | 'getState'
  | 'emergencyStop'
  | 'setArmState'
  | 'runTest';

interface WireCommandBase {
  type: 'command';
  uuid: string;
  command: CommandName;
}

export interface WireSetAxisAngleCommand extends WireCommandBase {
  command: 'setAxisAngle';
  parameters: { axis: number; angle: number };
}

export interface WireHomingSequenceCommand extends WireCommandBase {
  command: 'homingSequence';
  parameters: { axis: number };
}

export interface WireRunTestCommand extends WireCommandBase {
  command: 'runTest';
  parameters: { testIndex: number };
}

export interface WireEmergencyStopCommand extends WireCommandBase {
  command: 'emergencyStop';
  parameters: {};
}

export interface WireGetStateCommand extends WireCommandBase {
  command: 'getState';
  parameters: {};
}

export interface WireSetArmStateCommand extends WireCommandBase {
  command: 'setArmState';
  parameters: { state: unknown };
}

export type WireCommand =
  | WireSetAxisAngleCommand
  | WireHomingSequenceCommand
  | WireRunTestCommand
  | WireEmergencyStopCommand
  | WireGetStateCommand
  | WireSetArmStateCommand;
