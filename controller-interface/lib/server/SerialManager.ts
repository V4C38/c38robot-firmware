import { SerialPort } from 'serialport';
import { EventEmitter } from 'events';
import { v4 as uuidv4 } from 'uuid';
import type { Command, BaseResponse, CommandConfig } from '@/types/command.types';
import type { RobotConfig } from '@/types/robot-config.types';
import * as fs from 'fs/promises';
import * as path from 'path';

// Singleton instance for server-side serial management
class SerialManager extends EventEmitter {
  private static instance: SerialManager;
  private serialPort: SerialPort | null = null;
  private serialBuffer: string = '';
  private pendingCommands: Map<string, Command> = new Map();
  private commandConfig: CommandConfig | null = null;
  private robotConfig: RobotConfig | null = null;
  private isConnected: boolean = false;
  private logFile: string;
  private logs: string[] = [];

  private constructor() {
    super();
    this.logFile = path.join(process.cwd(), 'logs', 'serial.log');
    this.initializeLogging();
    // Load configs asynchronously (don't await in constructor)
    this.loadCommandConfig().catch(console.error);
    this.loadRobotConfig().catch(console.error);
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

  // Get available serial ports
  public async getAvailablePorts() {
    try {
      const ports = await SerialPort.list();
      return ports;
    } catch (error) {
      console.error('Failed to list serial ports:', error);
      throw error;
    }
  }

  // Open serial connection
  public async openPort(path: string, baudRate: number = 115200): Promise<boolean> {
    try {
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
    if (!this.serialPort?.isOpen || !this.isConnected) {
      await this.log('Attempted to send command but port is not open', 'ERROR');
      throw new Error('Serial port is not open');
    }

    const fullCommand: Command = {
      ...command,
      type: 'command',
      uuid: uuidv4()
    } as Command;

    // Store pending command
    this.pendingCommands.set(fullCommand.uuid, fullCommand);

    // Convert to JSON and send
    const jsonString = JSON.stringify(fullCommand) + '\n';
    
    // Log the command
    await this.log(`Sending command: ${fullCommand.command} ${JSON.stringify(command)}`, 'COMMAND');
    
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

      // Timeout for pending commands
      setTimeout(() => {
        if (this.pendingCommands.has(fullCommand.uuid)) {
          this.pendingCommands.delete(fullCommand.uuid);
          this.log(`Command timeout: ${fullCommand.command}`, 'ERROR');
          reject(new Error(`Command timeout: ${fullCommand.command}`));
        }
      }, 5000);
    });
  }

  // Handle incoming serial data
  private handleData(data: Buffer): void {
    this.serialBuffer += data.toString();
    
    // Process complete messages (ended with newline)
    let newlineIndex;
    while ((newlineIndex = this.serialBuffer.indexOf('\n')) !== -1) {
      const message = this.serialBuffer.substring(0, newlineIndex);
      this.serialBuffer = this.serialBuffer.substring(newlineIndex + 1);
      
      try {
        const parsed = JSON.parse(message);
        this.processMessage(parsed);
      } catch (error) {
        console.error('Failed to parse message:', message, error);
      }
    }
  }

  // Process parsed message
  private async processMessage(message: any): Promise<void> {
    if (message.type === 'response' && message.uuid) {
      const pendingCommand = this.pendingCommands.get(message.uuid);
      if (pendingCommand) {
        this.pendingCommands.delete(message.uuid);
        await this.log(`Received response for command: ${pendingCommand.command} - Status: ${message.status}`, 'RESPONSE');
      } else {
        await this.log(`Received response: ${JSON.stringify(message)}`, 'RESPONSE');
      }
      this.emit('messageReceived', message as BaseResponse);
    }
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
    return !!this.serialPort && this.serialPort.isOpen === true;
  }
}

export default SerialManager;
