import type {
  Command,
  BaseResponse,
  SetAxisAngleCommand,
  HomingSequenceCommand,
  RunTestCommand
} from '@/types/command.types';
import type { RobotConfig } from '@/types/robot-config.types';

export interface DebugSettings {
  delays: {
    getStateMs: number;
    homingSequenceMsPerAxis: number;
    runTestMs: number;
    emergencyStopMs: number;
    baseOverheadMs?: number;
  };
  movement: {
    degPerSecond: number[];
  };
}

type ResponseCallback = (message: BaseResponse) => void;

class SimulatedMicrocontroller {
  private static instance: SimulatedMicrocontroller | null = null;

  private robotConfig: RobotConfig | null = null;
  private settings: DebugSettings | null = null;
  private onResponse: ResponseCallback | null = null;
  private loopTimer: NodeJS.Timeout | null = null;
  private publishTimer: NodeJS.Timeout | null = null;
  private isRunning: boolean = false;
  private publishEnabled: boolean = true;
  private publishIntervalMs: number = 200;

  private currentAngles: number[] = [];
  private targetAngles: number[] = [];
  private isAtLimit: boolean[] = [];
  private limitIndex: Array<number | null> = [];

  public static getInstance(): SimulatedMicrocontroller {
    if (!SimulatedMicrocontroller.instance) {
      SimulatedMicrocontroller.instance = new SimulatedMicrocontroller();
    }
    return SimulatedMicrocontroller.instance;
  }

  public connect(robotConfig: RobotConfig, settings: DebugSettings, onResponse: ResponseCallback): void {
    this.robotConfig = robotConfig;
    this.settings = settings;
    this.onResponse = onResponse;

    const axes = robotConfig.armConfig.axes;
    this.currentAngles = new Array(axes).fill(0);
    this.targetAngles = new Array(axes).fill(0);
    this.isAtLimit = new Array(axes).fill(false);
    this.limitIndex = new Array(axes).fill(null);

    this.startLoop();
    this.startPublisher();
    this.isRunning = true;
  }

  public disconnect(): void {
    if (this.loopTimer) {
      clearInterval(this.loopTimer);
      this.loopTimer = null;
    }
    if (this.publishTimer) {
      clearInterval(this.publishTimer);
      this.publishTimer = null;
    }
    this.isRunning = false;
    this.onResponse = null;
    this.robotConfig = null;
    this.settings = null;
  }

  public isActive(): boolean {
    return this.isRunning;
  }

  public sendCommand(command: Command): void {
    if (!this.robotConfig || !this.settings || !this.onResponse) {
      return;
    }

    const baseOverhead = this.settings.delays.baseOverheadMs ?? 0;
    const callback = this.onResponse;

    const name = command.command;
    switch (name) {
      case 'getState': {
        const delay = baseOverhead + this.settings.delays.getStateMs;
        setTimeout(() => callback(this.buildStateResponse(command.uuid, 'getState', 'success', 'State retrieved successfully.')), delay);
        break;
      }
      case 'setAxisAngle': {
        const { axis, angle } = command as SetAxisAngleCommand;
        const clamped = this.clampToJointLimits(axis, angle);
        this.targetAngles[axis] = clamped;
        // Respond without stateUpdate to avoid visually jumping current angles
        setTimeout(() => callback({
          type: 'response',
          uuid: command.uuid,
          command: 'setAxisAngle',
          status: 'success',
          message: `Axis ${axis} target set to ${clamped} degrees.`
        } as BaseResponse), baseOverhead);
        break;
      }
      case 'homingSequence': {
        const { axis } = command as HomingSequenceCommand;
        const perAxis = this.settings.delays.homingSequenceMsPerAxis;
        if (axis === -1) {
          // Sequential homing order (3,4,2,1,0) similar to firmware
          const order = [3, 4, 2, 1, 0].filter(i => i < this.targetAngles.length);
          order.forEach((ax, idx) => {
            setTimeout(() => { this.targetAngles[ax] = 0; }, baseOverhead + perAxis * idx);
          });
          const total = baseOverhead + perAxis * order.length;
          setTimeout(() => callback(this.buildStateResponse(command.uuid, 'homingSequence', 'success', 'Homing all axes.')), total);
        } else {
          this.targetAngles[axis] = 0;
          const delay = baseOverhead + perAxis;
          setTimeout(() => callback(this.buildStateResponse(command.uuid, 'homingSequence', 'success', `Homing axis ${axis}`)), delay);
        }
        break;
      }
      case 'runTest': {
        // Emulate sequences loosely similar to firmware jog patterns
        const testId = (command as RunTestCommand).testIndex;
        const phases: Array<() => void> = [];

        const setTargets = (values: number[]) => {
          for (let i = 0; i < Math.min(values.length, this.targetAngles.length); i++) {
            this.targetAngles[i] = this.clampToJointLimits(i, values[i]);
          }
        };

        if (testId === 6) {
          phases.push(() => setTargets([25, 25, 25, 0, 0, 45]));
          phases.push(() => setTargets([-25, -25, -25, 0, 0, -45]));
          phases.push(() => setTargets([45, -35, 45, 0, 0, -30]));
          phases.push(() => setTargets([0, 0, 0, 0, 0, 0]));
        } else if (testId === 7) {
          phases.push(() => setTargets([5, -25, -40, 25, -15, 15]));
          phases.push(() => setTargets([15, -45, -65, 45, -35, -15]));
          phases.push(() => setTargets([-10, -15, 35, -45, 35, 50]));
          phases.push(() => setTargets([0, 0, 0, 0, 0, 0]));
        } else if (testId >= 0 && testId <= 5) {
          const axis = testId;
          phases.push(() => setTargets(this.targetAngles.map((_, i) => (i === axis ? 45 : 0))));
          phases.push(() => setTargets(this.targetAngles.map((_, i) => 0)));
        } else {
          phases.push(() => setTargets(this.targetAngles.map(() => 0)));
        }

        const stepDelay = Math.max(500, this.settings.delays.runTestMs / Math.max(1, phases.length));
        phases.forEach((fn, idx) => setTimeout(fn, baseOverhead + stepDelay * idx));
        const totalDelay = baseOverhead + stepDelay * phases.length;
        setTimeout(() => callback(this.buildStateResponse(command.uuid, 'runTest', 'success', 'Test completed.')), totalDelay);
        break;
      }
      case 'setStateInterval': {
        const ms = (command as unknown as { ms: number }).ms;
        this.setStateInterval(ms);
        setTimeout(() => callback({
          type: 'response',
          uuid: command.uuid,
          command: 'setStateInterval',
          status: 'success',
          message: `State interval set to ${this.publishIntervalMs} ms`
        } as BaseResponse), baseOverhead);
        break;
      }
      case 'enableStateStream': {
        const enabled = (command as unknown as { enabled: boolean }).enabled;
        this.enableStateStream(enabled);
        setTimeout(() => callback({
          type: 'response',
          uuid: command.uuid,
          command: 'enableStateStream',
          status: 'success',
          message: `State stream ${enabled ? 'enabled' : 'disabled'}`
        } as BaseResponse), baseOverhead);
        break;
      }
      case 'emergencyStop': {
        const delay = baseOverhead + this.settings.delays.emergencyStopMs;
        // Stop where we are
        for (let i = 0; i < this.targetAngles.length; i++) {
          this.targetAngles[i] = this.currentAngles[i];
        }
        setTimeout(() => callback(this.buildStateResponse(command.uuid, 'emergencyStop', 'success', 'Emergency stop executed.')), delay);
        break;
      }
      default: {
        setTimeout(() => callback(this.buildStateResponse(command.uuid, name, 'success', 'OK')), baseOverhead);
      }
    }
  }

  public setStateInterval(ms: number): void {
    const clamped = Math.max(20, Math.min(5000, Math.floor(ms)));
    this.publishIntervalMs = clamped;
    this.startPublisher();
  }

  public enableStateStream(enabled: boolean): void {
    this.publishEnabled = enabled;
    this.startPublisher();
  }

  private startLoop(): void {
    const tickMs = 20;
    let last = Date.now();
    this.loopTimer = setInterval(() => {
      if (!this.robotConfig || !this.settings) return;
      const now = Date.now();
      const dt = Math.min(0.1, Math.max(0, (now - last) / 1000));
      last = now;
      const joints = this.robotConfig.joints;
      for (let i = 0; i < this.currentAngles.length; i++) {
        const current = this.currentAngles[i];
        const target = this.targetAngles[i];
        const speed = this.settings.movement.degPerSecond[i] ?? 90;

        const remaining = target - current;
        if (Math.abs(remaining) < 0.001) {
          this.currentAngles[i] = target;
          continue;
        }
        const step = Math.sign(remaining) * speed * dt;
        let next = current + step;
        if (Math.sign(remaining) !== Math.sign(target - next)) {
          next = target;
        }
        // Clamp to joint limits
        const min = joints[i]?.minAngle ?? -180;
        const max = joints[i]?.maxAngle ?? 180;
        if (next <= min) {
          next = min;
          this.isAtLimit[i] = true;
          this.limitIndex[i] = 0;
        } else if (next >= max) {
          next = max;
          this.isAtLimit[i] = true;
          this.limitIndex[i] = 1;
        } else {
          this.isAtLimit[i] = false;
          this.limitIndex[i] = null;
        }
        this.currentAngles[i] = next;
      }
    }, tickMs);
  }

  private startPublisher(): void {
    if (this.publishTimer) {
      clearInterval(this.publishTimer);
      this.publishTimer = null;
    }
    if (!this.publishEnabled) return;
    this.publishTimer = setInterval(() => {
      // Build bracketed compact state line with integer format for consistency with firmware
      const ts = Date.now();
      const parts: string[] = [`[${ts}]`];
      for (let i = 0; i < this.currentAngles.length; i++) {
        // limit: -1 left (index 0), 0 none, 1 right (index 1)
        const li = this.limitIndex[i];
        const lim = li == null ? 0 : (li === 0 ? -1 : 1);
        // Use integer values to match firmware optimization
        const curInt = Math.round(this.currentAngles[i]);
        const tgtInt = Math.round(this.targetAngles[i]);
        parts.push(`[${i},${curInt},${tgtInt},${lim}]`);
      }
      const line = `state-update: ${parts.join('')}`;
      // Feed into SerialManager ingest path
      const g = globalThis as any;
      const mgr = g.__serialManagerInstance as { ingestRawLine?: (l: string) => void } | undefined;
      if (mgr?.ingestRawLine) {
        mgr.ingestRawLine(line);
      }
    }, this.publishIntervalMs);
  }

  private clampToJointLimits(axis: number, angle: number): number {
    const min = this.robotConfig?.joints[axis]?.minAngle ?? -180;
    const max = this.robotConfig?.joints[axis]?.maxAngle ?? 180;
    return Math.max(min, Math.min(max, angle));
  }

  private buildStateResponse(uuid: string, command: string, status: 'success' | 'error', message: string): BaseResponse {
    const axes: Record<string, number> = {};
    const limits: Record<string, { isAtLimit: boolean; limitIndex: number | null }> = {};
    for (let i = 0; i < this.currentAngles.length; i++) {
      axes[String(i)] = this.currentAngles[i];
      limits[String(i)] = { isAtLimit: this.isAtLimit[i] || false, limitIndex: this.limitIndex[i] ?? null };
    }
    return {
      type: 'response',
      uuid,
      command,
      status,
      message,
      stateUpdate: { axes, limits }
    } as BaseResponse;
  }

  private buildPartialAxisResponse(uuid: string, command: string, status: 'success' | 'error', message: string, axis: number, angle: number): BaseResponse {
    const limitsEntry = { isAtLimit: false, limitIndex: null as number | null };
    return {
      type: 'response',
      uuid,
      command,
      status,
      message,
      stateUpdate: { axes: { [String(axis)]: angle }, limits: { [String(axis)]: limitsEntry } }
    } as BaseResponse;
  }
}

export default SimulatedMicrocontroller;


