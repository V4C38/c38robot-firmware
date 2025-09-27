'use client';

import React, { useState, useCallback, useEffect, useRef } from 'react';
import { useRobot } from '@/contexts/RobotContext';
import { AxisSlider } from '@/components/AxisSlider';
import { Home, AlertTriangle, RotateCcw, ChevronDown, ChevronUp, Maximize2, Minimize2, Activity } from 'lucide-react';

export const AxisControlTab: React.FC = () => {
  const { armState, isConnected, sendCommand, targetAngles, updateTargetAngle, robotConfig } = useRobot();
  const [collapsedAxes, setCollapsedAxes] = useState<Set<number>>(new Set());
  const [allCollapsed, setAllCollapsed] = useState(false);
  const [autoUpdate, setAutoUpdate] = useState(true);
  const [updateInterval, setUpdateInterval] = useState(1.0);
  const intervalRef = useRef<NodeJS.Timeout | null>(null);
  const isPollingRef = useRef(false);

  const sendAxisCommand = useCallback(async (axis: number, angle: number) => {
    if (!isConnected) {
      return;
    }

    try {
      await sendCommand({
        command: 'setAxisAngle',
        axis: axis,
        angle: angle
      });
    } catch (error) {
      console.error('Failed to send axis command:', error);
    }
  }, [isConnected, sendCommand]);

  const handleSliderChange = (axis: number, value: number[]) => {
    updateTargetAngle(axis, value[0]);
  };

  const handleSliderCommit = (axis: number, value: number[]) => {
    // Send command when slider drag ends
    sendAxisCommand(axis, value[0]);
  };

  const handleTargetChange = (axis: number, value: string) => {
    const angle = parseFloat(value) || 0;
    const joint = armState.joints[axis];
    
    // Clamp value to joint limits
    const clampedAngle = Math.max(joint.minAngle, Math.min(joint.maxAngle, angle));
    
    updateTargetAngle(axis, clampedAngle);
    
    // Send command immediately
    sendAxisCommand(axis, clampedAngle);
  };

  const handleResetAxis = (axis: number) => {
    updateTargetAngle(axis, 0);
    
    // Send command immediately
    sendAxisCommand(axis, 0);
  };

  const handleHomeAxis = async (axis: number) => {
    if (!isConnected) {
      alert('Please connect to a serial port first');
      return;
    }

    try {
      await sendCommand({
        command: 'homingSequence',
        axis: axis
      });
    } catch (error) {
      console.error('Failed to send home command:', error);
    }
  };

  const handleEmergencyStop = async () => {
    if (!isConnected) {
      alert('Please connect to a serial port first');
      return;
    }

    try {
      await sendCommand({
        command: 'emergencyStop'
      });
    } catch (error) {
      console.error('Failed to send emergency stop command:', error);
    }
  };

  const handleResetAllAngles = async () => {
    if (!isConnected) {
      alert('Please connect to a serial port first');
      return;
    }

    // Reset all angles to 0
    try {
      for (let axis = 0; axis < armState.joints.length; axis++) {
        updateTargetAngle(axis, 0);
        await sendCommand({
          command: 'setAxisAngle',
          axis: axis,
          angle: 0
        });
      }
    } catch (error) {
      console.error('Failed to send reset commands:', error);
    }
  };

  const toggleAxisCollapse = (axisIndex: number) => {
    setCollapsedAxes(prev => {
      const newSet = new Set(prev);
      if (newSet.has(axisIndex)) {
        newSet.delete(axisIndex);
      } else {
        newSet.add(axisIndex);
      }
      return newSet;
    });
  };

  const toggleAllAxes = () => {
    if (allCollapsed) {
      // Expand all
      setCollapsedAxes(new Set());
      setAllCollapsed(false);
    } else {
      // Collapse all
      const allIndices = new Set(armState.joints.map((_, index) => index));
      setCollapsedAxes(allIndices);
      setAllCollapsed(true);
    }
  };

  const handleHomeAllAxes = async () => {
    if (!isConnected) {
      alert('Please connect to a serial port first');
      return;
    }

    // Send homing commands for all axes
    try {
      for (let axis = 0; axis < armState.joints.length; axis++) {
        await sendCommand({
          command: 'homingSequence',
          axis: axis
        });
      }
    } catch (error) {
      console.error('Failed to send home all commands:', error);
    }
  };

  const handleGetState = async () => {
    if (!isConnected) {
      return;
    }
    if (isPollingRef.current) {
      return;
    }
    isPollingRef.current = true;
    try {
      await sendCommand({
        command: 'getState'
      });
    } catch (error) {
      console.error('Failed to send get state command:', error);
    } finally {
      isPollingRef.current = false;
    }
  };

  // Auto-update effect
  useEffect(() => {
    if (autoUpdate && isConnected && updateInterval > 0) {
      intervalRef.current = setInterval(() => {
        handleGetState();
      }, updateInterval * 1000);
    } else {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    }

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
        intervalRef.current = null;
      }
    };
  }, [autoUpdate, isConnected, updateInterval]);

  const handleAutoUpdateToggle = () => {
    setAutoUpdate(!autoUpdate);
  };

  const handleIntervalChange = (value: string) => {
    const interval = parseFloat(value);
    if (isNaN(interval)) return;
    // Clamp to reduce MCU load and avoid jitter during moves
    const clamped = Math.max(0.5, Math.min(60, interval));
    setUpdateInterval(clamped);
  };

  return (
    <div className="p-6 space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-2xl font-bold text-black">Axis Control</h2>
        </div>
        
        <div className="flex items-center gap-4">
          {/* Auto-update controls */}
          <div className="flex items-center gap-2 px-3 py-1 bg-gray-100 rounded-md">
            <label className="flex items-center gap-2 text-sm">
              <input
                type="checkbox"
                checked={autoUpdate}
                onChange={handleAutoUpdateToggle}
                className="w-4 h-4 text-blue-600 bg-gray-100 border-gray-300 rounded focus:ring-blue-500"
              />
              <span className="text-black">Auto Update</span>
            </label>
            <input
              type="number"
              value={updateInterval.toFixed(1)}
              onChange={(e) => handleIntervalChange(e.target.value)}
              min="0.1"
              max="60"
              step="0.1"
              disabled={!autoUpdate}
              className="w-16 px-1 py-0.5 text-xs border border-gray-300 rounded text-black bg-white disabled:bg-gray-200"
            />
            <span className="text-xs text-gray-600">s</span>
          </div>
          
          <button
            onClick={toggleAllAxes}
            className="px-3 py-1 bg-gray-100 hover:bg-gray-200 text-black font-medium rounded-md flex items-center gap-2 transition-colors text-sm"
          >
            {allCollapsed ? <Maximize2 className="w-4 h-4" /> : <Minimize2 className="w-4 h-4" />}
            {allCollapsed ? 'Expand All' : 'Collapse All'}
          </button>
          
          <button
            onClick={handleEmergencyStop}
            disabled={!isConnected}
            className={`px-3 py-1 font-medium rounded-md flex items-center gap-2 transition-colors text-sm ${
              !isConnected 
                ? 'bg-gray-300 text-black cursor-not-allowed' 
                : 'bg-red-600 hover:bg-red-700 text-white'
            }`}
          >
            <AlertTriangle className="w-4 h-4" />
            Emergency Stop
          </button>
        </div>
      </div>

      {/* Connection Warning */}
      {!isConnected && (
        <div className="p-4 bg-yellow-50 border border-yellow-200 rounded-lg flex items-center gap-3">
          <AlertTriangle className="w-5 h-5 text-yellow-600" />
          <p className="text-yellow-800">
            Serial port is not connected. Please connect to a port in the Serial Port tab first.
          </p>
        </div>
      )}

      {/* Axis Controls */}
      <div className="space-y-4">
        {armState.joints.map((joint, index) => {
          // Get config for this joint
          const robotJoint = robotConfig?.joints[index];
          const isCollapsed = collapsedAxes.has(index);
          
          return (
            <div key={index} className="bg-gray-50 rounded-lg p-4">
              {/* Header with collapse toggle */}
              <div className="flex justify-between items-center mb-3">
                <div className="flex items-center gap-4">
                  <button
                    onClick={() => toggleAxisCollapse(index)}
                    className="flex items-center gap-2 text-lg font-medium text-black hover:text-blue-600 transition-colors"
                  >
                    <h3>Axis {index}</h3>
                    {isCollapsed ? <ChevronDown className="w-4 h-4" /> : <ChevronUp className="w-4 h-4" />}
                  </button>
                  
                  <button
                    onClick={() => handleHomeAxis(index)}
                    disabled={!isConnected}
                    className={`px-3 py-1 rounded-md flex items-center gap-1 text-sm font-medium transition-colors text-black ${
                      !isConnected 
                        ? 'bg-gray-200 cursor-not-allowed' 
                        : 'bg-gray-200 hover:bg-gray-300'
                    }`}
                  >
                    <Home className="w-3 h-3" />
                    Home
                  </button>
                </div>
                
                <div className="flex items-center gap-3 text-sm">
                  <span className="text-gray-600">
                    Current: {joint.currentAngle.toFixed(1)}°
                  </span>
                  {!isCollapsed && (
                    <div className="flex items-center gap-2">
                      <span className="text-gray-600">Target:</span>
                      <input
                        type="number"
                        value={targetAngles[index].toFixed(1)}
                        onChange={(e) => handleTargetChange(index, e.target.value)}
                        min={robotJoint?.minAngle ?? joint.minAngle}
                        max={robotJoint?.maxAngle ?? joint.maxAngle}
                        step="0.1"
                        className="w-16 px-1 py-0.5 text-xs border border-gray-300 rounded text-black bg-white"
                      />
                      <span className="text-gray-600">°</span>
                      <button
                        onClick={() => handleResetAxis(index)}
                        className="px-1.5 py-0.5 text-xs bg-gray-200 hover:bg-gray-300 rounded text-black transition-colors"
                      >
                        <RotateCcw className="w-3 h-3" />
                      </button>
                    </div>
                  )}
                </div>
              </div>

              {/* Sliders - show current only when collapsed, both when expanded */}
              <AxisSlider
                value={targetAngles[index]}
                currentValue={joint.currentAngle}
                robotJoint={robotJoint}
                fallbackMin={joint.minAngle}
                fallbackMax={joint.maxAngle}
                onChange={(value) => handleSliderChange(index, value)}
                onCommit={(value) => handleSliderCommit(index, value)}
                collapsed={isCollapsed}
                isAtLimit={joint.isAtLimit}
                limitIndex={joint.limitIndex}
              />
            </div>
          );
        })}
      </div>

      {/* Control Buttons */}
      <div className="flex justify-center gap-4 pt-4">
        <button
          onClick={handleGetState}
          disabled={!isConnected}
          className={`px-6 py-3 font-medium rounded-md transition-colors flex items-center gap-2 ${
            !isConnected 
              ? 'bg-gray-300 text-black cursor-not-allowed' 
              : 'bg-purple-600 hover:bg-purple-700 text-white'
          }`}
        >
          <Activity className="w-4 h-4" />
          Get State
        </button>
        
        <button
          onClick={handleResetAllAngles}
          disabled={!isConnected}
          className={`px-6 py-3 font-medium rounded-md transition-colors flex items-center gap-2 ${
            !isConnected 
              ? 'bg-gray-300 text-black cursor-not-allowed' 
              : 'bg-blue-600 hover:bg-blue-700 text-white'
          }`}
        >
          <RotateCcw className="w-4 h-4" />
          Reset All Axes
        </button>
        
        <button
          onClick={handleHomeAllAxes}
          disabled={!isConnected}
          className={`px-6 py-3 font-medium rounded-md transition-colors flex items-center gap-2 ${
            !isConnected 
              ? 'bg-gray-300 text-black cursor-not-allowed' 
              : 'bg-green-600 hover:bg-green-700 text-white'
          }`}
        >
          <Home className="w-4 h-4" />
          Home All Axes
        </button>
      </div>
    </div>
  );
};
