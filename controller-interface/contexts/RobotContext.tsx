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
  connectToPort: (port: string) => Promise<boolean>;
  disconnect: () => Promise<void>;
  sendCommand: (command: any) => Promise<void>;
  updateArmState: (updates: Partial<ArmState>) => void;
  setSelectedPort: (port: string | null) => void;
  updateTargetAngle: (axis: number, angle: number) => void;
  reloadConfig: () => Promise<void>;
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
  const [armState, setArmState] = useState<ArmState>(defaultArmState);
  const [commandConfig, setCommandConfig] = useState<CommandConfig | null>(null);
  const [robotConfig, setRobotConfig] = useState<RobotConfig | null>(null);
  const [targetAngles, setTargetAngles] = useState<number[]>(
    defaultArmState.joints.map(joint => joint.targetAngle)
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
        const stateUpdate = response as StateUpdateResponse;
        if (stateUpdate.stateUpdate?.axes) {
          // Update arm state based on response
          setArmState(prev => {
            const newState = { ...prev };
            Object.entries(stateUpdate.stateUpdate!.axes).forEach(([axis, angle]) => {
              const axisIndex = parseInt(axis);
              if (axisIndex >= 0 && axisIndex < newState.joints.length) {
                newState.joints[axisIndex].currentAngle = angle;
              }
            });
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

  const connectToPort = useCallback(async (port: string) => {
    const success = await serialClient.openPort(port);
    if (success) {
      setSelectedPort(port);
    }
    return success;
  }, []);

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

  const reloadConfig = useCallback(async () => {
    try {
      // Force reload configs from the server
      await serialClient.reloadConfigs();
      
      // Get the updated configs
      const updatedRobotConfig = serialClient.getRobotConfig();
      const updatedCommandConfig = serialClient.getCommandConfig();
      
      if (updatedRobotConfig) {
        setRobotConfig(updatedRobotConfig);
        // Update arm state with new robot config
        if (updatedRobotConfig.joints) {
          setArmState(prev => ({
            ...prev,
            joints: updatedRobotConfig.joints.map((joint, index) => ({
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
            updatedRobotConfig.joints.map((joint, index) => 
              Math.max(joint.minAngle, Math.min(joint.maxAngle, targetAngles[index] || 0))
            )
          );
        }
      }
      
      if (updatedCommandConfig) {
        setCommandConfig(updatedCommandConfig);
      }
    } catch (error) {
      console.error('Failed to reload config:', error);
      throw error;
    }
  }, [targetAngles]);

  const value: RobotContextType = {
    isConnected,
    availablePorts,
    selectedPort,
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
    updateTargetAngle,
    reloadConfig
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
