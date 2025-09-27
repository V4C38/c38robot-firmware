import type { Command, BaseResponse, CommandConfig } from '@/types/command.types';
import type { RobotConfig } from '@/types/robot-config.types';

// Type definitions for serial port info
export interface PortInfo {
  path: string;
  manufacturer?: string;
  serialNumber?: string;
  pnpId?: string;
  locationId?: string;
  productId?: string;
  vendorId?: string;
}

export interface SerialClientEvents {
  commandSent: (command: Command) => void;
  messageReceived: (response: BaseResponse) => void;
  portsUpdated: (ports: PortInfo[]) => void;
  connectionChanged: (connected: boolean) => void;
  error: (error: Error) => void;
}

// Simple browser-compatible event emitter
class EventEmitter {
  private events: { [key: string]: Function[] } = {};

  on(event: string, listener: Function): this {
    if (!this.events[event]) {
      this.events[event] = [];
    }
    this.events[event].push(listener);
    return this;
  }

  emit(event: string, ...args: any[]): boolean {
    if (!this.events[event]) {
      return false;
    }
    this.events[event].forEach(listener => listener(...args));
    return true;
  }

  removeListener(event: string, listenerToRemove: Function): this {
    if (!this.events[event]) {
      return this;
    }
    this.events[event] = this.events[event].filter(listener => listener !== listenerToRemove);
    return this;
  }

  removeAllListeners(): this {
    this.events = {};
    return this;
  }
}

export declare interface SerialClient {
  on<U extends keyof SerialClientEvents>(
    event: U, listener: SerialClientEvents[U]
  ): this;
  emit<U extends keyof SerialClientEvents>(
    event: U, ...args: Parameters<SerialClientEvents[U]>
  ): boolean;
}

export class SerialClient extends EventEmitter {
  private isConnected: boolean = false;
  private commandConfig: CommandConfig | null = null;
  private robotConfig: RobotConfig | null = null;
  private pollInterval: number | null = null;

  constructor() {
    super();
    // Only initialize if we're in the browser
    if (typeof window !== 'undefined') {
      this.initialize();
    }
  }

  private async initialize() {
    // Only initialize in browser environment
    if (typeof window !== 'undefined') {
      await this.updateStatus();
      // No automatic polling - status will be updated on-demand
    }
  }

  private startPolling() {
    // Only start polling in browser environment
    if (typeof window !== 'undefined') {
      // Poll for connection status every 5 seconds (less frequent)
      this.pollInterval = window.setInterval(async () => {
        await this.updateStatus();
      }, 5000);
    }
  }

  private stopPolling() {
    if (this.pollInterval && typeof window !== 'undefined') {
      window.clearInterval(this.pollInterval);
      this.pollInterval = null;
    }
  }

  private async updateStatus() {
    try {
      const response = await fetch('/api/serial/status');
      const data = await response.json();
      
      if (data.connected !== this.isConnected) {
        this.isConnected = data.connected;
        this.emit('connectionChanged', this.isConnected);
      }
      
      if (data.config) {
        this.commandConfig = data.config;
      }
      
      if (data.robotConfig) {
        this.robotConfig = data.robotConfig;
      }
    } catch (error) {
      console.error('Failed to update status:', error);
    }
  }

  // Get available serial ports
  public async getAvailablePorts(): Promise<PortInfo[]> {
    try {
      const response = await fetch('/api/serial/ports');
      const data = await response.json();
      
      if (data.error) {
        // Emit error without throwing to avoid breaking UI flows
        this.emit('error', new Error(data.message || data.error));
        return;
      }
      
      const ports = data.ports || [];
      this.emit('portsUpdated', ports);
      return ports;
    } catch (error) {
      console.error('Failed to list serial ports:', error);
      this.emit('error', error as Error);
      return [];
    }
  }

  // Open serial connection
  public async openPort(path: string, baudRate: number = 115200): Promise<boolean> {
    try {
      const response = await fetch('/api/serial/connect', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ port: path, baudRate }),
      });
      
      const data = await response.json();
      
      if (data.error) {
        // Do not throw to avoid breaking UI; emit error for any listeners (e.g., log tab)
        this.emit('error', new Error(data.message || data.error));
        return;
      }
      
      this.isConnected = data.connected;
      this.emit('connectionChanged', this.isConnected);
      
      // Update status after connection to get configs
      await this.updateStatus();
      
      return data.success;
    } catch (error) {
      console.error('Failed to open serial port:', error);
      this.emit('error', error as Error);
      return false;
    }
  }

  // Close serial connection
  public async closePort(): Promise<void> {
    try {
      if (!this.isConnected) {
        return;
      }
      const response = await fetch('/api/serial/disconnect', {
        method: 'POST',
      });
      
      const data = await response.json();
      
      if (data.error) {
        throw new Error(data.error);
      }
      
      this.isConnected = false;
      this.emit('connectionChanged', false);
      
      // Update status after disconnection to clear configs
      await this.updateStatus();
    } catch (error) {
      console.error('Failed to close serial port:', error);
      this.emit('error', error as Error);
    }
  }

  // Send command through serial port
  public async sendCommand(command: Omit<Command, 'uuid' | 'type'>): Promise<void> {
    try {
      if (!this.isConnected) {
        // Silently ignore when not connected to avoid console errors during mode flips
        return;
      }
      const response = await fetch('/api/serial/command', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(command),
      });
      
      const data = await response.json();
      
      if (data.error) {
        // Emit error without throwing to avoid breaking UI flows
        this.emit('error', new Error(data.message || data.error));
        return;
      }
      
      if (data.command) {
        this.emit('commandSent', data.command);
      }

      // If server returned the MCU response, emit it to subscribers immediately
      if (data.response) {
        this.emit('messageReceived', data.response as BaseResponse);
      }
    } catch (error) {
      console.error('Failed to send command:', error);
      this.emit('error', error as Error);
    }
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

  // Reload configurations from server
  public async loadConfigs(): Promise<void> {
    await this.updateStatus();
  }

  // Reload configurations from server (alias for compatibility)
  public async reloadConfigs(): Promise<void> {
    await this.loadConfigs();
  }

  // Cleanup
  public destroy() {
    this.stopPolling();
    this.removeAllListeners();
  }
}

// Create and export singleton instance only in browser
let serialClient: SerialClient;

if (typeof window !== 'undefined') {
  serialClient = new SerialClient();
} else {
  // Create a comprehensive mock instance for SSR
  const mockClient = {
    getAvailablePorts: async () => [],
    openPort: async () => false,
    closePort: async () => {},
    sendCommand: async () => {},
    getConnectionStatus: () => false,
    getCommandConfig: () => null,
    getRobotConfig: () => null,
    loadConfigs: async () => {},
    reloadConfigs: async () => {},
    destroy: () => {},
    on: (event: any, listener: any) => mockClient,
    emit: (event: any, ...args: any[]) => false,
    removeListener: (event: any, listener: any) => mockClient,
    removeAllListeners: (event?: any) => mockClient,
    updateStatus: async () => {},
    off: (event: any, listener: any) => mockClient,
    once: (event: any, listener: any) => mockClient,
    addListener: (event: any, listener: any) => mockClient,
    prependListener: (event: any, listener: any) => mockClient,
    prependOnceListener: (event: any, listener: any) => mockClient,
    eventNames: () => [],
    listeners: (event: any) => [],
    listenerCount: (event: any) => 0,
    getMaxListeners: () => 0,
    setMaxListeners: (n: number) => mockClient,
    rawListeners: (event: any) => []
  };
  serialClient = mockClient as any;
}

export default serialClient;