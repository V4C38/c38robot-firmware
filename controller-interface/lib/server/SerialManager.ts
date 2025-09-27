import { SerialPort } from 'serialport';
import { EventEmitter } from 'events';
import { v4 as uuidv4 } from 'uuid';
import type { Command, BaseResponse, CommandConfig } from '@/types/command.types';
import type { RobotConfig } from '@/types/robot-config.types';
import * as fs from 'fs/promises';
import * as path from 'path';
import SimulatedMicrocontroller from '@/lib/server/SimulatedMicrocontroller';

// Singleton instance for server-side serial management
class SerialManager extends EventEmitter {
  private static instance: SerialManager;
  private serialPort: SerialPort | null = null;
  private serialBuffer: string = '';
  private pendingCommands: Map<string, Command> = new Map();
  private pendingResponseResolvers: Map<string, (message: BaseResponse) => void> = new Map();
  private commandConfig: CommandConfig | null = null;
  private robotConfig: RobotConfig | null = null;
  private isConnected: boolean = false;
  private logFile: string;
  private logs: string[] = [];
  // Debug simulation
  private debugSimulated: boolean = false;
  private debugSettings: DebugSettings | null = null;
  private simAngles: number[] = Array(6).fill(0);
  // getState request coalescing
  private getStateInFlight: boolean = false;
  private currentGetStateCommand: Command | null = null;
  private getStateWaiters: Array<(message: BaseResponse) => void> = [];
  private lastStateResponse: BaseResponse | null = null;

  private constructor() {
    super();
    this.logFile = path.join(process.cwd(), 'logs', 'serial.log');
    this.initializeLogging();
    // Load configs asynchronously (don't await in constructor)
    this.loadCommandConfig().catch(console.error);
    this.loadRobotConfig().catch(console.error);
    this.loadDebugSettings().catch(() => {});
    this.setupCleanupHandlers();
  }

  private setupCleanupHandlers() {
    // Handle process exit
    process.on('exit', () => {
      this.forceClosePort();
    });

    // Handle graceful shutdown before exit
    process.on('beforeExit', () => {
      this.forceClosePort();
    });

    // Handle SIGINT (Ctrl+C)
    process.on('SIGINT', () => {
      this.forceClosePort();
      process.exit(0);
    });

    // Handle SIGTERM
    process.on('SIGTERM', () => {
      this.forceClosePort();
      process.exit(0);
    });

    // Handle common HMR signals
    process.on('SIGUSR1', () => {
      this.forceClosePort();
    });
    process.on('SIGUSR2', () => {
      this.forceClosePort();
    });

    // Handle uncaught exceptions
    process.on('uncaughtException', (error) => {
      console.error('Uncaught exception:', error);
      this.forceClosePort();
    });

    // Handle unhandled rejections
    process.on('unhandledRejection', (reason, promise) => {
      console.error('Unhandled rejection at:', promise, 'reason:', reason);
      this.forceClosePort();
    });
  }

  private forceClosePort() {
    // Clear simulator state as well
    if (this.debugSimulated) {
      this.debugSimulated = false;
    }
    if (this.serialPort) {
      try {
        this.serialPort.removeAllListeners();
        if (this.serialPort.isOpen) {
          this.serialPort.close();
        }
        this.serialPort.destroy();
      } catch (error) {
        console.error('Error in force close:', error);
      }
      this.serialPort = null;
      this.isConnected = false;
    }
  }

  // Public hard disconnect to guarantee the port is released
  public async forceDisconnect(): Promise<void> {
    await this.log('Force disconnect requested');
    this.forceClosePort();
    // Give the OS a brief moment to release the device handle
    await new Promise(resolve => setTimeout(resolve, 1000));
  }

  private async initializeLogging() {
    try {
      // Create logs directory if it doesn't exist
      const logsDir = path.dirname(this.logFile);
      await fs.mkdir(logsDir, { recursive: true });
      
      // Clear log file on server restart
      await fs.writeFile(this.logFile, '');
      this.logs = [];
      this.log('Serial communication log started');
    } catch (error) {
      console.error('Failed to initialize logging:', error);
    }
  }

  // Clear in-memory and on-disk logs
  public async clearLogs(): Promise<void> {
    try {
      this.logs = [];
      await fs.writeFile(this.logFile, '');
    } catch (error) {
      console.error('Failed to clear logs:', error);
    }
  }

  private async log(message: string, type: 'INFO' | 'COMMAND' | 'RESPONSE' | 'ERROR' = 'INFO') {
    const timestamp = new Date().toISOString();
    const logEntry = `[${timestamp}] ${type}: ${message}`;
    
    try {
      // Add to in-memory logs
      this.logs.push(logEntry);
      
      // Keep only last 1000 entries in memory
      if (this.logs.length > 1000) {
        this.logs = this.logs.slice(-1000);
      }
      
      // Write to file
      await fs.appendFile(this.logFile, logEntry + '\n');
      
      // Emit log event for real-time updates
      this.emit('logUpdated', logEntry);
    } catch (error) {
      console.error('Failed to write log:', error);
    }
  }

  // Public logging entrypoint for API routes and other callers
  public async appendLog(message: string, type: 'INFO' | 'COMMAND' | 'RESPONSE' | 'ERROR' = 'INFO') {
    await this.log(message, type);
  }

  public static getInstance(): SerialManager {
    const g = globalThis as any;
    if (!g.__serialManagerInstance) {
      g.__serialManagerInstance = new SerialManager();
    }
    return g.__serialManagerInstance as SerialManager;
  }

  // Publicly update robot config in-memory (e.g., after saving file via API)
  public setRobotConfig(config: RobotConfig): void {
    this.robotConfig = config;
    void this.log(`Robot config updated in-memory (axes: ${config.armConfig?.axes ?? 'n/a'})`);
  }

  // Publicly update command config in-memory
  public setCommandConfig(config: CommandConfig): void {
    this.commandConfig = config;
    void this.log('Command config updated in-memory');
  }

  // Reload robot config from disk
  public async reloadRobotConfigFromDisk(): Promise<void> {
    try {
      const fs = await import('fs/promises');
      const path = await import('path');
      const configPath = path.join(process.cwd(), 'config', 'robot_arm_config.json');
      const configData = await fs.readFile(configPath, 'utf-8');
      const parsed = JSON.parse(configData) as RobotConfig;
      this.robotConfig = parsed;
      await this.log('Robot config reloaded from disk');
    } catch (error) {
      console.error('Failed to reload robot configuration:', error);
      await this.log('Failed to reload robot configuration', 'ERROR');
    }
  }

  // Wait for configs to be loaded (useful for API endpoints)
  public async waitForInitialization(timeout: number = 5000): Promise<void> {
    const startTime = Date.now();
    
    while (!this.robotConfig && (Date.now() - startTime) < timeout) {
      await new Promise(resolve => setTimeout(resolve, 100));
    }
    
    if (!this.robotConfig) {
      throw new Error('Robot configuration failed to load within timeout');
    }
  }

  // Load command configuration
  private async loadCommandConfig(): Promise<void> {
    try {
      const fs = await import('fs/promises');
      const path = await import('path');
      const configPath = path.join(process.cwd(), 'config', 'CommandConfig.json');
      const configData = await fs.readFile(configPath, 'utf-8');
      this.commandConfig = JSON.parse(configData);
    } catch (error) {
      console.error('Failed to load command configuration:', error);
    }
  }

  // Load robot configuration
  private async loadRobotConfig(): Promise<void> {
    try {
      const fs = await import('fs/promises');
      const path = await import('path');
      const configPath = path.join(process.cwd(), 'config', 'robot_arm_config.json');
      const configData = await fs.readFile(configPath, 'utf-8');
      const parsedConfig = JSON.parse(configData) as RobotConfig;
      this.robotConfig = parsedConfig;
      await this.log(`Loaded robot config with ${parsedConfig.armConfig.axes} axes`);
    } catch (error) {
      console.error('Failed to load robot configuration:', error);
      await this.log(`Failed to load robot configuration: ${error instanceof Error ? error.message : 'Unknown error'}`, 'ERROR');
    }
  }

  private async loadDebugSettings(): Promise<void> {
    try {
      const fs = await import('fs/promises');
      const path = await import('path');
      const settingsPath = path.join(process.cwd(), 'config', 'DebugSettings.json');
      const settingsData = await fs.readFile(settingsPath, 'utf-8');
      this.debugSettings = JSON.parse(settingsData) as DebugSettings;
      await this.log('Loaded debug settings');
    } catch (error) {
      // Optional file; use defaults if missing
      this.debugSettings = {
        delays: {
          getStateMs: 40,
          homingSequenceMsPerAxis: 1500,
          runTestMs: 3000,
          emergencyStopMs: 20,
          baseOverheadMs: 5
        },
        movement: {
          degPerSecond: Array(this.robotConfig?.armConfig?.axes ?? 6).fill(90)
        }
      };
      await this.log('Using default debug settings');
    }
  }

  private simulateDebugResponse(fullCommand: Command): void {
    if (!this.debugSettings) return;
    const commandName = (fullCommand as unknown as { command: string }).command;
    const params = Object.fromEntries(
      Object.entries(fullCommand as unknown as Record<string, unknown>)
        .filter(([key]) => key !== 'type' && key !== 'uuid' && key !== 'command')
    ) as Record<string, unknown>;

    const baseOverhead = this.debugSettings.delays.baseOverheadMs ?? 0;
    let delay = baseOverhead;

    if (commandName === 'getState') {
      delay += this.debugSettings.delays.getStateMs;
      setTimeout(() => {
        const response = this.buildStateResponse(fullCommand.uuid, 'getState', 'success', 'State retrieved successfully.');
        this.dispatchSimulatedResponse(response);
      }, delay);
      return;
    }

    if (commandName === 'setAxisAngle') {
      const axis = Number(params.axis ?? 0);
      const target = Number(params.angle ?? 0);
      const current = this.simAngles[axis] ?? 0;
      const axes = this.robotConfig?.armConfig?.axes ?? this.simAngles.length;
      const degPerSec = (this.debugSettings.movement.degPerSecond[axis] ?? 90);
      const travel = Math.abs(target - current);
      const travelMs = Math.round((travel / degPerSec) * 1000);
      delay += travelMs;

      // Schedule movement simulation and then respond
      setTimeout(() => {
        this.simAngles[axis] = target;
        const response = this.buildStateResponse(fullCommand.uuid, 'setAxisAngle', 'success', `Axis ${axis} angle set to ${target} degrees.`);
        this.dispatchSimulatedResponse(response);
      }, delay);
      return;
    }

    if (commandName === 'homingSequence') {
      const axis = Number(params.axis ?? -1);
      const axes = this.robotConfig?.armConfig?.axes ?? this.simAngles.length;
      const count = axis === -1 ? axes : 1;
      delay += this.debugSettings.delays.homingSequenceMsPerAxis * count;
      setTimeout(() => {
        if (axis === -1) {
          this.simAngles = this.simAngles.map(() => 0);
        } else if (axis >= 0 && axis < this.simAngles.length) {
          this.simAngles[axis] = 0;
        }
        const response = this.buildStateResponse(fullCommand.uuid, 'homingSequence', 'success', axis === -1 ? 'Homing all axes.' : `Homing axis ${axis}`);
        this.dispatchSimulatedResponse(response);
      }, delay);
      return;
    }

    if (commandName === 'runTest') {
      delay += this.debugSettings.delays.runTestMs;
      setTimeout(() => {
        const response = this.buildStateResponse(fullCommand.uuid, 'runTest', 'success', 'Test completed.');
        this.dispatchSimulatedResponse(response);
      }, delay);
      return;
    }

    if (commandName === 'emergencyStop') {
      delay += this.debugSettings.delays.emergencyStopMs;
      setTimeout(() => {
        const response = this.buildStateResponse(fullCommand.uuid, 'emergencyStop', 'success', 'Emergency stop executed.');
        this.dispatchSimulatedResponse(response);
      }, delay);
      return;
    }

    // Default immediate success
    setTimeout(() => {
      const response = this.buildStateResponse(fullCommand.uuid, commandName, 'success', 'OK');
      this.dispatchSimulatedResponse(response);
    }, delay);
  }

  private buildStateResponse(uuid: string, command: string, status: string, message: string): BaseResponse {
    const axesState: Record<string, number> = {};
    const limits: Record<string, { isAtLimit: boolean; limitIndex: number }> = {};
    const axes = this.robotConfig?.armConfig?.axes ?? this.simAngles.length;
    for (let i = 0; i < axes; i++) {
      axesState[String(i)] = this.simAngles[i] ?? 0;
      limits[String(i)] = { isAtLimit: false, limitIndex: -1 };
    }
    const response = {
      type: 'response',
      uuid,
      command,
      status,
      message,
      stateUpdate: {
        axes: axesState,
        limits
      }
    } as unknown as BaseResponse;
    return response;
  }

  private dispatchSimulatedResponse(message: BaseResponse) {
    const json = JSON.stringify(message);
    void this.log(`RX: ${json}`);
    // Resolve waiting maps like a real message
    this.processMessage(message as unknown as Record<string, unknown>).catch(() => {});
  }

  // Get available serial ports
  public async getAvailablePorts() {
    try {
      const ports = await SerialPort.list();
      // Always include a debug simulator port option
      const debugPort = {
        path: 'debug://simulated',
        manufacturer: 'C38Robot Simulator',
        serialNumber: 'SIM-0001',
        pnpId: 'SIMULATED',
        locationId: 'SIM',
        productId: '0000',
        vendorId: '0000'
      };
      return [...ports, debugPort];
    } catch (error) {
      console.error('Failed to list serial ports:', error);
      throw error;
    }
  }

  // Open serial connection
  public async openPort(path: string, baudRate: number = 115200): Promise<boolean> {
    try {
      // Debug simulator connection
      if (path.startsWith('debug://')) {
        // Close any existing connection
        if (this.serialPort) {
          await this.closePort();
        }
        await this.log(`Connecting to debug simulator at ${path}`);
        this.debugSimulated = true;
        await this.loadDebugSettings();
        // Start simulator engine
        if (this.robotConfig && this.debugSettings) {
          SimulatedMicrocontroller.getInstance().connect(
            this.robotConfig,
            this.debugSettings,
            (message) => {
              void this.log(`RX: ${JSON.stringify(message)}`);
              void this.processMessage(message as unknown as Record<string, unknown>);
            }
          );
        }
        this.isConnected = true;
        this.emit('connectionChanged', true);
        return true;
      }

      // Always close any existing connection first
      if (this.serialPort) {
        await this.closePort();
        // Wait a bit for the port to be fully released
        await new Promise(resolve => setTimeout(resolve, 2000));
      }

      await this.log(`Attempting to connect to ${path} at ${baudRate} baud`);

      this.serialPort = new SerialPort({
        path,
        baudRate,
        autoOpen: false,
        lock: false
      });

      // Set up event handlers
      this.serialPort.on('data', this.handleData.bind(this));
      this.serialPort.on('error', (err: Error) => {
        console.error('Serial port error:', err);
        this.log(`Serial port error: ${err.message}`, 'ERROR');
        this.isConnected = false;
        this.emit('connectionChanged', false);
        this.emit('error', err);
        // Force cleanup on error
        this.forceClosePort();
      });
      this.serialPort.on('close', () => {
        this.isConnected = false;
        this.log('Serial port connection closed');
        this.emit('connectionChanged', false);
      });
      this.serialPort.on('disconnect', () => {
        this.isConnected = false;
        this.log('Serial port disconnected');
        this.emit('connectionChanged', false);
      });

      // Open the port
      await new Promise<void>((resolve, reject) => {
        this.serialPort!.open((err?: Error | null) => {
          if (err) {
            reject(err);
          } else {
            resolve();
          }
        });
      });

      // Give the MCU a brief moment to finish USB CDC setup after port open
      await new Promise(resolve => setTimeout(resolve, 300));

      this.isConnected = true;
      await this.log(`Successfully connected to ${path}`);
      this.emit('connectionChanged', true);
      return true;
    } catch (error) {
      console.error('Failed to open serial port:', error);
      await this.log(`Failed to connect to ${path}: ${error instanceof Error ? error.message : 'Unknown error'}`, 'ERROR');
      throw error;
    }
  }

  // Close serial connection
  public async closePort(): Promise<void> {
    if (this.debugSimulated) {
      await this.log('Closing debug simulator connection');
      SimulatedMicrocontroller.getInstance().disconnect();
      this.debugSimulated = false;
      this.isConnected = false;
      this.emit('connectionChanged', false);
      return;
    }

    if (this.serialPort) {
      await this.log('Closing serial port connection');
      
      try {
        // Remove all listeners to prevent memory leaks
        this.serialPort.removeAllListeners();
        
        // Attempt to flush/drain IO before closing
        try {
          await new Promise<void>((resolve, reject) => {
            this.serialPort!.flush((err?: Error | null) => (err ? reject(err) : resolve()));
          });
        } catch {}
        try {
          await new Promise<void>((resolve, reject) => {
            this.serialPort!.drain((err?: Error | null) => (err ? reject(err) : resolve()));
          });
        } catch {}

        if (this.serialPort.isOpen) {
          await new Promise<void>((resolve, reject) => {
            this.serialPort!.close((err) => {
              if (err) {
                console.error('Error closing serial port:', err);
                reject(err);
              } else {
                resolve();
              }
            });
          });
        }
        
        // Force destroy the port if still not closed
        if (this.serialPort.isOpen) {
          this.serialPort.destroy();
        }
        
      } catch (error) {
        console.error('Error during port closure:', error);
        await this.log(`Error during port closure: ${error instanceof Error ? error.message : 'Unknown error'}`, 'ERROR');
        
        // Force destroy even if close failed
        try {
          this.serialPort.destroy();
        } catch (destroyError) {
          console.error('Error destroying port:', destroyError);
        }
      }
      
      this.serialPort = null;
      this.isConnected = false;
      await this.log('Serial port connection closed');
      this.emit('connectionChanged', false);
    }
  }

  // Send command through serial port
  public async sendCommand(command: Omit<Command, 'uuid' | 'type'>): Promise<Command> {
    if (!this.debugSimulated && (!this.serialPort?.isOpen || !this.isConnected)) {
      await this.log('Attempted to send command but port is not open', 'ERROR');
      throw new Error('Serial port is not open');
    }

    // Build UI-level command object with uuid (kept for pending map and events)
    const fullCommand: Command = {
      ...(command as unknown as Record<string, unknown>),
      type: 'command',
      uuid: uuidv4()
    } as Command;

    // Store pending command
    this.pendingCommands.set(fullCommand.uuid, fullCommand);

    // Build standardized on-wire envelope { type, uuid, command, parameters }
    const commandName = (fullCommand as unknown as { command: string }).command;
    const parameters = Object.fromEntries(
      Object.entries(fullCommand as unknown as Record<string, unknown>)
        .filter(([key]) => key !== 'type' && key !== 'uuid' && key !== 'command')
    );
    const wirePayload = {
      type: 'command',
      uuid: fullCommand.uuid,
      command: commandName,
      parameters
    };

    // Convert to JSON and send
    const jsonString = JSON.stringify(wirePayload) + '\r\n';
    
    // Log the command
    await this.log(`Sending command: ${fullCommand.command} ${JSON.stringify(command)}`, 'COMMAND');

    if (this.debugSimulated) {
      // Simulate command delivery and schedule a fake response
      this.emit('commandSent', fullCommand);
      SimulatedMicrocontroller.getInstance().sendCommand(fullCommand as Command);

      return new Promise((resolve, reject) => {
        // Simulate immediate write success
        setTimeout(() => resolve(fullCommand), 0);
        // Keep standard timeout to resolve waiting callers
        const TIMEOUT_MS = 60000;
        setTimeout(() => {
          if (this.pendingCommands.has(fullCommand.uuid)) {
            this.pendingCommands.delete(fullCommand.uuid);
            void this.log(`Command timeout (debug): ${fullCommand.command}`, 'ERROR');
            reject(new Error(`Command timeout: ${fullCommand.command}`));
          }
        }, TIMEOUT_MS);
      });
    }

    return new Promise((resolve, reject) => {
      this.serialPort!.write(jsonString, (err?: Error | null) => {
        if (err) {
          console.error('Failed to send command:', err);
          this.log(`Failed to send command: ${err.message}`, 'ERROR');
          this.pendingCommands.delete(fullCommand.uuid);
          reject(err);
        } else {
          this.emit('commandSent', fullCommand);
          resolve(fullCommand);
        }
      });

      // Timeout for pending commands (allow long-running firmware ops like homing)
      const TIMEOUT_MS = 60000;
      setTimeout(() => {
        if (this.pendingCommands.has(fullCommand.uuid)) {
          this.pendingCommands.delete(fullCommand.uuid);
          this.log(`Command timeout: ${fullCommand.command}`, 'ERROR');
          reject(new Error(`Command timeout: ${fullCommand.command}`));
        }
      }, TIMEOUT_MS);
    });
  }

  // Send command and wait for MCU response for the same UUID
  public async sendCommandAndWait(
    command: Omit<Command, 'uuid' | 'type'>,
    timeoutMs: number = 60000
  ): Promise<{ command: Command; response: BaseResponse }> {
    const sent = await this.sendCommand(command);
    const response = await new Promise<BaseResponse>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pendingResponseResolvers.delete(sent.uuid);
        void this.log(`Response timeout: ${sent.command}`, 'ERROR');
        reject(new Error(`Response timeout: ${sent.command}`));
      }, timeoutMs);

      this.pendingResponseResolvers.set(sent.uuid, (message: BaseResponse) => {
        clearTimeout(timer);
        resolve(message);
      });
    });

    return { command: sent, response };
  }

  // Handle incoming serial data
  private handleData(data: Buffer): void {
    this.serialBuffer += data.toString();
    
    // Process complete messages (ended with newline)
    let newlineIndex;
    while ((newlineIndex = this.serialBuffer.indexOf('\n')) !== -1) {
      let message = this.serialBuffer.substring(0, newlineIndex);
      this.serialBuffer = this.serialBuffer.substring(newlineIndex + 1);
      
      try {
        // Trim possible CR from Arduino's println ("\r\n") and any surrounding whitespace
        message = message.replace(/\r+$/, '').trim();
        if (message.length === 0) {
          continue;
        }
        void this.log(`RX: ${message}`);
        const parsed = JSON.parse(message);
        this.processMessage(parsed);
      } catch (error) {
        console.error('Failed to parse message:', message, error);
        void this.log(`Parse error for line: ${message}`, 'ERROR');
      }
    }
  }

  // Process parsed message
  private async processMessage(message: any): Promise<void> {
    if (message.type === 'response' && message.uuid) {
      const pendingCommand = this.pendingCommands.get(message.uuid);
      if (pendingCommand) {
        this.pendingCommands.delete(message.uuid);
        const details = typeof message.message === 'string' && message.message.length > 0
          ? ` - Message: ${message.message}`
          : '';
        await this.log(`Received response for command: ${pendingCommand.command} - Status: ${message.status}${details}`, 'RESPONSE');
      } else {
        await this.log(`Received response: ${JSON.stringify(message)}`, 'RESPONSE');
      }

      // Cache last state response for getState
      if ((message as { command?: string }).command === 'getState' && message.status === 'success') {
        this.lastStateResponse = message as BaseResponse;
        // Resolve any coalesced waiters
        const waiters = [...this.getStateWaiters];
        this.getStateWaiters = [];
        for (const waiter of waiters) {
          try {
            waiter(this.lastStateResponse);
          } catch {}
        }
        this.getStateInFlight = false;
        this.currentGetStateCommand = null;
      }

      // Resolve any waiter for this response
      const resolver = this.pendingResponseResolvers.get(message.uuid as string);
      if (resolver) {
        this.pendingResponseResolvers.delete(message.uuid as string);
        resolver(message as BaseResponse);
      }
      this.emit('messageReceived', message as BaseResponse);
    }
  }

  // Coalesce concurrent getState requests to a single on-wire request
  public async sendGetStateCoalesced(
    payload: Omit<Command, 'uuid' | 'type'>,
    timeoutMs: number = 5000
  ): Promise<{ command: Command; response: BaseResponse }> {
    // If there's already a getState in-flight, join it
    if (this.getStateInFlight && this.currentGetStateCommand) {
      return new Promise((resolve, reject) => {
        const timer = setTimeout(() => {
          reject(new Error('Response timeout: getState'));
        }, timeoutMs);

        this.getStateWaiters.push((message: BaseResponse) => {
          clearTimeout(timer);
          resolve({ command: this.currentGetStateCommand as Command, response: message });
        });
      });
    }

    this.getStateInFlight = true;

    // Send without waiting so we can set up our own resolver
    const sent = await this.sendCommand(payload);
    this.currentGetStateCommand = sent;

    const response = await new Promise<BaseResponse>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pendingResponseResolvers.delete(sent.uuid);
        this.getStateInFlight = false;
        this.currentGetStateCommand = null;
        reject(new Error('Response timeout: getState'));
      }, timeoutMs);

      this.pendingResponseResolvers.set(sent.uuid, (message: BaseResponse) => {
        clearTimeout(timer);
        resolve(message);
      });
    });

    this.lastStateResponse = response;

    // Resolve any waiters (if any remain)
    const waiters = [...this.getStateWaiters];
    this.getStateWaiters = [];
    for (const waiter of waiters) {
      try {
        waiter(response);
      } catch {}
    }

    this.getStateInFlight = false;
    this.currentGetStateCommand = null;

    return { command: sent, response };
  }

  // Get logs
  public getLogs(): string[] {
    return [...this.logs];
  }

  // Get connection status
  public getConnectionStatus(): boolean {
    return this.isConnected;
  }

  // Get command configuration
  public getCommandConfig(): CommandConfig | null {
    return this.commandConfig;
  }

  public getRobotConfig(): RobotConfig | null {
    return this.robotConfig;
  }

  // Whether the underlying serialport instance is present and open
  public isPortOpen(): boolean {
    if (this.debugSimulated) {
      return true;
    }
    return !!this.serialPort && this.serialPort.isOpen === true;
  }
}

export default SerialManager;

// Types
interface DebugSettings {
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

