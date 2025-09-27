'use client';

import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import serialClient from '@/lib/client/serialClient';
import type { PortInfo } from '@/lib/client/serialClient';
import type { ArmState, JointData } from '@/types/arm.types';
import type { BaseResponse, StateUpdateResponse, CommandConfig } from '@/types/command.types';
import type { RobotConfig } from '@/types/robot-config.types';

interface RobotContextType {
  // Connection state
  isConnected: boolean;
  availablePorts: PortInfo[];
  selectedPort: string | null;
  baudRate: number;
  
  // Arm state
  armState: ArmState;
  
  // Target angles (persistent across tab switches)
  targetAngles: number[];
  
  // Command configuration
  commandConfig: CommandConfig | null;
  
  // Robot configuration
  robotConfig: RobotConfig | null;
  
  // Actions
  refreshPorts: () => Promise<void>;
  connectToPort: (port: string, baudRate?: number) => Promise<boolean>;
  disconnect: () => Promise<void>;
  sendCommand: (command: any) => Promise<void>;
  updateArmState: (updates: Partial<ArmState>) => void;
  setSelectedPort: (port: string | null) => void;
  setBaudRate: (baud: number) => void;
  updateTargetAngle: (axis: number, angle: number) => void;
  reloadConfig: () => Promise<void>;
  // Streaming state helpers
  pullLatestState: () => Promise<void>;
  isDraggingSlider: boolean[];
  setSliderDragging: (axis: number, dragging: boolean) => void;
}

const RobotContext = createContext<RobotContextType | undefined>(undefined);

// Default arm state with 6 axes
const defaultArmState: ArmState = {
  joints: Array(6).fill(null).map((_, index) => ({
    isCalibrated: false,
    currentAngle: 0,
    targetAngle: 0,
    dHParameters: {
      theta: 0,
      d: 0,
      a: 0,
      alpha: 0
    },
    minAngle: -180,
    maxAngle: 180
  })),
  isDriverActive: false,
  debugMode: false
};

export const RobotProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [isConnected, setIsConnected] = useState(false);
  const [availablePorts, setAvailablePorts] = useState<PortInfo[]>([]);
  const [selectedPort, setSelectedPort] = useState<string | null>(null);
  const [baudRate, setBaudRate] = useState<number>(115200);
  const [armState, setArmState] = useState<ArmState>(defaultArmState);
  const [commandConfig, setCommandConfig] = useState<CommandConfig | null>(null);
  const [robotConfig, setRobotConfig] = useState<RobotConfig | null>(null);
  const [targetAngles, setTargetAngles] = useState<number[]>(
    defaultArmState.joints.map(joint => joint.targetAngle)
  );
  const [isDraggingSlider, setIsDraggingSlider] = useState<boolean[]>(
    defaultArmState.joints.map(() => false)
  );

  // Initialize serial interface listeners
  useEffect(() => {
    const handleConnectionChanged = (connected: boolean) => {
      setIsConnected(connected);
      if (!connected) {
        setSelectedPort(null);
      }
    };

    const handlePortsUpdated = (ports: PortInfo[]) => {
      setAvailablePorts(ports);
    };

    const handleMessageReceived = (response: BaseResponse) => {
      if (response.status === 'success' && 'stateUpdate' in response) {
        const responseCommand = (response as unknown as { command?: string }).command;
        // Ignore immediate echo updates from setAxisAngle; rely on periodic getState updates instead
        if (responseCommand === 'setAxisAngle') {
          return;
        }
        const stateUpdate = response as StateUpdateResponse;
        if (stateUpdate.stateUpdate?.axes) {
          setArmState(prev => {
            const newState = { ...prev };
            Object.entries(stateUpdate.stateUpdate!.axes).forEach(([axis, angle]) => {
              const axisIndex = parseInt(axis);
              if (axisIndex >= 0 && axisIndex < newState.joints.length) {
                newState.joints[axisIndex].currentAngle = angle;
              }
            });
            if (stateUpdate.stateUpdate?.limits) {
              Object.entries(stateUpdate.stateUpdate.limits).forEach(([axis, info]) => {
                const axisIndex = parseInt(axis);
                if (axisIndex >= 0 && axisIndex < newState.joints.length) {
                  newState.joints[axisIndex].isAtLimit = info.isAtLimit;
                  newState.joints[axisIndex].limitIndex = info.limitIndex;
                }
              });
            }
            return newState;
          });
        }
      }
    };

    const handleError = (error: Error) => {
      console.error('Serial interface error:', error);
    };

    // Set up event listeners
    serialClient.on('connectionChanged', handleConnectionChanged);
    serialClient.on('portsUpdated', handlePortsUpdated);
    serialClient.on('messageReceived', handleMessageReceived);
    serialClient.on('error', handleError);

    // Only load configs in browser environment
    if (typeof window !== 'undefined') {
      // Ensure port is closed when window is closing/reloading
      const handleBeforeUnload = () => {
        // Fire-and-forget request to close the port on page close
        navigator.sendBeacon?.('/api/serial/disconnect');
      };
      window.addEventListener('beforeunload', handleBeforeUnload);

      // Function to load configs and set state
      const loadConfigs = () => {
        // Load command config
        const config = serialClient.getCommandConfig();
        if (config) {
          setCommandConfig(config);
        }
        
        // Load robot config
        const robotConf = serialClient.getRobotConfig();
        if (robotConf) {
          setRobotConfig(robotConf);
          // Update arm state with robot config joints
          if (robotConf.joints) {
            setArmState(prev => ({
              ...prev,
              joints: robotConf.joints.map((joint, index) => ({
                id: index,
                currentAngle: prev.joints[index]?.currentAngle || 0,
                targetAngle: prev.joints[index]?.targetAngle || 0,
                minAngle: joint.minAngle,
                maxAngle: joint.maxAngle,
                isCalibrated: prev.joints[index]?.isCalibrated || false,
                dHParameters: prev.joints[index]?.dHParameters || {
                  theta: joint.theta,
                  d: joint.d,
                  a: joint.a,
                  alpha: joint.alpha
                }
              }))
            }));
            // Update target angles to respect new limits
            setTargetAngles(
              robotConf.joints.map((joint, index) => 
                Math.max(joint.minAngle, Math.min(joint.maxAngle, targetAngles[index] || 0))
              )
            );
          }
        }
      };

      // Try to load configs immediately
      loadConfigs();

      // If robot config is still null, wait for it to be loaded by polling
      if (!serialClient.getRobotConfig()) {
        const configPollInterval = setInterval(() => {
          if (serialClient.getRobotConfig()) {
            loadConfigs();
            clearInterval(configPollInterval);
          }
        }, 500);

        // Clear interval after 10 seconds to prevent infinite polling
        setTimeout(() => {
          clearInterval(configPollInterval);
        }, 10000);
      }
    }

    // Initial port refresh (only in browser)
    if (typeof window !== 'undefined') {
      refreshPorts();
    }

    // Cleanup
    return () => {
      serialClient.removeListener('connectionChanged', handleConnectionChanged);
      serialClient.removeListener('portsUpdated', handlePortsUpdated);
      serialClient.removeListener('messageReceived', handleMessageReceived);
      serialClient.removeListener('error', handleError);
      serialClient.destroy();
      if (typeof window !== 'undefined') {
        window.removeEventListener('beforeunload', () => {});
      }
    };
  }, []);

  const refreshPorts = useCallback(async () => {
    await serialClient.getAvailablePorts();
  }, []);

  const connectToPort = useCallback(async (port: string, requestedBaudRate?: number) => {
    const baud = requestedBaudRate ?? baudRate;
    const success = await serialClient.openPort(port, baud);
    if (success) {
      setSelectedPort(port);
    }
    return success;
  }, [baudRate]);

  const disconnect = useCallback(async () => {
    await serialClient.closePort();
  }, []);

  const sendCommand = useCallback(async (command: any) => {
    try {
      await serialClient.sendCommand(command);
    } catch (error) {
      console.error('Failed to send command:', error);
      // Optionally emit an error event or show user notification
    }
  }, []);

  const updateArmState = useCallback((updates: Partial<ArmState>) => {
    setArmState(prev => ({ ...prev, ...updates }));
  }, []);

  const setSelectedPortHandler = useCallback((port: string | null) => {
    setSelectedPort(port);
  }, []);

  const updateTargetAngle = useCallback((axis: number, angle: number) => {
    setTargetAngles(prev => {
      const newAngles = [...prev];
      newAngles[axis] = angle;
      return newAngles;
    });
  }, []);

  // Public helpers to mark slider drag state
  const setSliderDragging = useCallback((axis: number, dragging: boolean) => {
    setIsDraggingSlider(prev => {
      const next = [...prev];
      next[axis] = dragging;
      return next;
    });
  }, []);

  const pullLatestState = useCallback(async () => {
    try {
      const res = await fetch('/api/serial/state');
      const data = await res.json();
      const latest = data?.latestState as { timestamp: number; axes: Record<string, { current: number; target: number; limit: number }> } | null;
      if (!latest || !latest.axes) return;
      setArmState(prev => {
        const newState = { ...prev };
        const numJoints = newState.joints.length;
        for (let i = 0; i < numJoints; i++) {
          const axisKey = String(i);
          const axisData = latest.axes[axisKey];
          if (!axisData) continue;
          // Current angle always from stream
          newState.joints[i].currentAngle = axisData.current;
          // Target angle from stream unless user is dragging this axis
          if (!isDraggingSlider[i]) {
            newState.joints[i].targetAngle = axisData.target;
          }
          // Limit mapping: -1 left -> index 0, 1 right -> index 1
          if (axisData.limit === 0) {
            newState.joints[i].isAtLimit = false;
            newState.joints[i].limitIndex = null;
          } else {
            newState.joints[i].isAtLimit = true;
            newState.joints[i].limitIndex = axisData.limit < 0 ? 0 : 1;
          }
        }
        return newState;
      });
      // Keep local targetAngles in sync with stream for non-dragging axes
      setTargetAngles(prev => {
        const next = [...prev];
        for (let i = 0; i < next.length; i++) {
          const axisKey = String(i);
          const axisData = latest.axes[axisKey];
          if (!axisData) continue;
          if (!isDraggingSlider[i]) {
            next[i] = axisData.target;
          }
        }
        return next;
      });
    } catch (e) {
      console.error('Failed to fetch latest state:', e);
    }
  }, [isDraggingSlider]);

  const reloadConfig = useCallback(async () => {
    try {
      // Reload configs from the server by calling the API endpoint
      const response = await fetch('/api/config/reload', {
        method: 'POST',
      });
      
      if (!response.ok) {
        throw new Error('Failed to reload config from server');
      }
      
      // Now reload configs from the server into the client
      await serialClient.loadConfigs();
      
      // Load command config
      const config = serialClient.getCommandConfig();
      if (config) {
        setCommandConfig(config);
      }
      
      // Load robot config and force re-render by creating new object reference
      const robotConf = serialClient.getRobotConfig();
      if (robotConf) {
        // Force new object reference to trigger React re-renders
        setRobotConfig({ ...robotConf });
        
        // Update arm state with robot config joints
        if (robotConf.joints) {
          setArmState(prev => ({
            ...prev,
            joints: robotConf.joints.map((joint, index) => ({
              id: index,
              currentAngle: prev.joints[index]?.currentAngle || 0,
              targetAngle: prev.joints[index]?.targetAngle || 0,
              minAngle: joint.minAngle,
              maxAngle: joint.maxAngle,
              isCalibrated: prev.joints[index]?.isCalibrated || false,
              dHParameters: {
                theta: joint.theta,
                d: joint.d,
                a: joint.a,
                alpha: joint.alpha
              }
            }))
          }));
          // Update target angles to respect new limits
          setTargetAngles(
            robotConf.joints.map((joint, index) => 
              Math.max(joint.minAngle, Math.min(joint.maxAngle, targetAngles[index] || 0))
            )
          );
        }
      }
    } catch (error) {
      console.error('Failed to reload config:', error);
      throw error; // Re-throw so the UI can handle the error
    }
  }, [targetAngles]);

  // Auto-connect/disconnect debug simulator based on debugMode
  useEffect(() => {
    const debugPortPath = 'debug://simulated';
    if (armState.debugMode) {
      if (selectedPort !== debugPortPath || !isConnected) {
        setSelectedPort(debugPortPath);
        void connectToPort(debugPortPath, baudRate);
      }
    } else {
      if (selectedPort === debugPortPath && isConnected) {
        void disconnect();
        setSelectedPort(null);
      }
    }
  }, [armState.debugMode, selectedPort, isConnected, connectToPort, disconnect, baudRate]);

  const value: RobotContextType = {
    isConnected,
    availablePorts,
    selectedPort,
    baudRate,
    armState,
    targetAngles,
    commandConfig,
    robotConfig,
    refreshPorts,
    connectToPort,
    disconnect,
    sendCommand,
    updateArmState,
    setSelectedPort: setSelectedPortHandler,
    setBaudRate,
    updateTargetAngle,
    reloadConfig,
    pullLatestState,
    isDraggingSlider,
    setSliderDragging
  };

  return (
    <RobotContext.Provider value={value}>
      {children}
    </RobotContext.Provider>
  );
};

export const useRobot = () => {
  const context = useContext(RobotContext);
  if (!context) {
    throw new Error('useRobot must be used within a RobotProvider');
  }
  return context;
};
