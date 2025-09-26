'use client';

import React, { useState, useEffect, useMemo, useRef } from 'react';
import { useRobot } from '@/contexts/RobotContext';
import { Play, RefreshCw, Activity, Bug, Check, Download, Search, Filter } from 'lucide-react';
import * as Select from '@radix-ui/react-select';

export const TestingTab: React.FC = () => {
  const { isConnected, sendCommand, commandConfig, armState, updateArmState } = useRobot();
  const [selectedMode, setSelectedMode] = useState<'single' | 'jog1' | 'jog2'>('single');
  const [selectedAxis, setSelectedAxis] = useState<string>('0');
  const [isRunningTest, setIsRunningTest] = useState(false);
  const [serialLogs, setSerialLogs] = useState<string[]>([]);
  const [searchTerm, setSearchTerm] = useState('');
  const [logFilter, setLogFilter] = useState<'all' | 'info' | 'command' | 'response' | 'update' | 'error'>('all');
  const [isTabVisible, setIsTabVisible] = useState(false);
  const tabRef = useRef<HTMLDivElement>(null);

  // Fetch logs from server
  const fetchLogs = async () => {
    try {
      const response = await fetch('/api/serial/logs');
      const data = await response.json();
      if (data.logs) {
        setSerialLogs(data.logs);
      }
    } catch (error) {
      console.error('Failed to fetch logs:', error);
    }
  };

  // Track if the Testing tab is visible using Intersection Observer
  useEffect(() => {
    if (!tabRef.current) return;

    const observer = new IntersectionObserver(
      ([entry]) => {
        setIsTabVisible(entry.isIntersecting);
      },
      { threshold: 0.1 } // Consider visible if 10% of the tab is visible
    );

    observer.observe(tabRef.current);

    return () => {
      observer.disconnect();
    };
  }, []);

  // Fetch logs only when tab is visible
  useEffect(() => {
    if (!isTabVisible) return;

    // Fetch immediately when tab becomes visible
    fetchLogs();

    // Then poll every 5 seconds while visible
    const interval = setInterval(fetchLogs, 5000);
    return () => clearInterval(interval);
  }, [isTabVisible]);

  const handleRunTest = async () => {
    if (!isConnected) {
      alert('Please connect to a serial port first');
      return;
    }

    setIsRunningTest(true);
    let testIndex = 0;
    if (selectedMode === 'single') {
      testIndex = parseInt(selectedAxis);
    } else if (selectedMode === 'jog1') {
      testIndex = 6;
    } else {
      testIndex = 7;
    }

    try {
      await sendCommand({
        command: 'runTest',
        testIndex: testIndex
      });
    } catch (error) {
      console.error('Failed to send test command:', error);
    }

    // Simulate test completion (in real app, this would be based on response)
    setTimeout(() => {
      setIsRunningTest(false);
    }, 3000);
  };

  const handleGetState = async () => {
    if (!isConnected) {
      alert('Please connect to a serial port first');
      return;
    }

    try {
      await sendCommand({
        command: 'getState'
      });
    } catch (error) {
      console.error('Failed to send get state command:', error);
    }
  };

  const handleToggleDebugMode = () => {
    const newDebugMode = !armState.debugMode;
    updateArmState({ debugMode: newDebugMode });
  };

  const handleClearLog = async () => {
    try {
      const res = await fetch('/api/serial/logs/clear', { method: 'POST' });
      const data = await res.json();
      if (data.success) {
        setSerialLogs([]);
      } else {
        console.error('Failed to clear logs:', data.error);
      }
    } catch (error) {
      console.error('Failed to clear logs:', error);
    }
  };

  const handleDownloadLogs = () => {
    const logContent = serialLogs.join('\n');
    const blob = new Blob([logContent], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `serial-logs-${new Date().toISOString().split('T')[0]}.log`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  // Helper function to detect update responses
  const isUpdateResponse = (log: string) => {
    return (
      // RESPONSE logs with getState
      (log.includes('RESPONSE') && (
        log.includes('getState') || 
        log.includes('State retrieved successfully')
      )) ||
      // Raw RX logs containing getState response JSON
      (log.includes('RX:') && log.includes('"command":"getState"')) ||
      // COMMAND logs sending getState
      (log.includes('COMMAND') && log.includes('getState'))
    );
  };

  // Filter and search logs
  const filteredLogs = useMemo(() => {
    let filtered = serialLogs;

    // Apply type filter
    if (logFilter !== 'all') {
      filtered = filtered.filter(log => {
        switch (logFilter) {
          case 'command':
            return log.includes('COMMAND');
          case 'response':
            // Exclude update responses from regular response filter
            return log.includes('RESPONSE') && !isUpdateResponse(log);
          case 'update':
            // Show only update responses
            return isUpdateResponse(log);
          case 'error':
            return log.includes('ERROR');
          case 'info':
            return log.includes('INFO') && !log.includes('COMMAND') && !log.includes('RESPONSE') && !log.includes('ERROR');
          default:
            return true;
        }
      });
    } else {
      // For 'all' filter, exclude update responses to avoid clutter
      filtered = filtered.filter(log => !isUpdateResponse(log));
    }

    // Apply search filter
    if (searchTerm) {
      filtered = filtered.filter(log => 
        log.toLowerCase().includes(searchTerm.toLowerCase())
      );
    }

    return filtered;
  }, [serialLogs, logFilter, searchTerm]);

  return (
    <div ref={tabRef} className="p-6 space-y-6">
      <div>
        <h2 className="text-2xl font-bold mb-2 text-black">Testing & Diagnostics</h2>
        <p className="text-gray-600">
          Run predefined test sequences and access diagnostic tools.
        </p>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Test Controls */}
        <div className="space-y-4">
          <div className="bg-gray-50 rounded-lg p-4 space-y-4">
            <h3 className="font-medium text-lg text-black">Test Sequences</h3>

            <div className="space-y-4">
              <div className="space-y-2">
                <label className="block text-sm font-medium text-black">Test Mode</label>
                <Select.Root value={selectedMode} onValueChange={(v: any) => setSelectedMode(v)}>
                  <Select.Trigger className="w-full px-4 py-2 bg-white border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500 text-black">
                    <Select.Value />
                  </Select.Trigger>
                  <Select.Portal>
                    <Select.Content className="bg-white rounded-md shadow-lg border border-gray-200 mt-1">
                      <Select.Viewport>
                        <Select.Item value="single" className="px-4 py-2 hover:bg-gray-100 cursor-pointer text-black">
                          <Select.ItemText>Single Axis</Select.ItemText>
                        </Select.Item>
                        <Select.Item value="jog1" className="px-4 py-2 hover:bg-gray-100 cursor-pointer text-black">
                          <Select.ItemText>Jogging 1</Select.ItemText>
                        </Select.Item>
                        <Select.Item value="jog2" className="px-4 py-2 hover:bg-gray-100 cursor-pointer text-black">
                          <Select.ItemText>Jogging 2</Select.ItemText>
                        </Select.Item>
                      </Select.Viewport>
                    </Select.Content>
                  </Select.Portal>
                </Select.Root>
              </div>

              {selectedMode === 'single' && (
                <div className="space-y-2">
                  <label className="block text-sm font-medium text-black">Axis</label>
                  <Select.Root value={selectedAxis} onValueChange={setSelectedAxis}>
                    <Select.Trigger className="w-full px-4 py-2 bg-white border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500 text-black">
                      <Select.Value />
                    </Select.Trigger>
                    <Select.Portal>
                      <Select.Content className="bg-white rounded-md shadow-lg border border-gray-200 mt-1">
                        <Select.Viewport>
                          {[0,1,2,3,4,5].map((axis) => (
                            <Select.Item key={axis} value={String(axis)} className="px-4 py-2 hover:bg-gray-100 cursor-pointer text-black">
                              <Select.ItemText>Axis {axis}</Select.ItemText>
                            </Select.Item>
                          ))}
                        </Select.Viewport>
                      </Select.Content>
                    </Select.Portal>
                  </Select.Root>
                </div>
              )}
            </div>

            <button
              onClick={handleRunTest}
              disabled={!isConnected || isRunningTest}
              className={`w-full px-4 py-2 font-medium rounded-md flex items-center justify-center gap-2 transition-colors ${
                (!isConnected || isRunningTest) 
                  ? 'bg-gray-300 text-black cursor-not-allowed' 
                  : 'bg-green-600 hover:bg-green-700 text-white'
              }`}
            >
              {isRunningTest ? (
                <>
                  <RefreshCw className="w-4 h-4 animate-spin" />
                  Running Test...
                </>
              ) : (
                <>
                  <Play className="w-4 h-4" />
                  Run Test
                </>
              )}
            </button>
          </div>

          {/* Diagnostic Tools */}
          <div className="bg-gray-50 rounded-lg p-4 space-y-4">
            <h3 className="font-medium text-lg text-black">Diagnostic Tools</h3>
            
            <div className="space-y-2">
              <button
                onClick={handleGetState}
                disabled={!isConnected}
                className={`w-full px-4 py-2 font-medium rounded-md flex items-center justify-center gap-2 transition-colors ${
                  !isConnected 
                    ? 'bg-gray-300 text-black cursor-not-allowed' 
                    : 'bg-blue-500 hover:bg-blue-600 text-white'
                }`}
              >
                <Activity className="w-4 h-4" />
                Get Current State
              </button>

              <button
                onClick={handleToggleDebugMode}
                className={`w-full px-4 py-2 font-medium rounded-md flex items-center justify-center gap-2 transition-colors ${
                  armState.debugMode 
                    ? 'bg-yellow-500 hover:bg-yellow-600 text-white' 
                    : 'bg-gray-200 hover:bg-gray-300 text-gray-700'
                }`}
              >
                <Bug className="w-4 h-4" />
                Debug Mode: {armState.debugMode ? 'ON' : 'OFF'}
              </button>
            </div>
          </div>

          {/* System Status */}
          <div className="bg-gray-50 rounded-lg p-4 space-y-3">
            <h3 className="font-medium text-lg text-black">System Status</h3>
            
            <div className="space-y-2 text-sm">
              <div className="flex items-center justify-between">
                <span className="text-gray-600">Connection</span>
                <span className={`font-medium ${isConnected ? 'text-green-600' : 'text-red-600'}`}>
                  {isConnected ? 'Connected' : 'Disconnected'}
                </span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-gray-600">Driver Active</span>
                <span className={`font-medium ${armState.isDriverActive ? 'text-green-600' : 'text-gray-600'}`}>
                  {armState.isDriverActive ? 'Active' : 'Inactive'}
                </span>
              </div>
              <div className="flex items-center justify-between">
                <span className="text-gray-600">Calibrated Axes</span>
                <span className="font-medium">
                  {armState.joints.filter(j => j.isCalibrated).length} / {armState.joints.length}
                </span>
              </div>
            </div>
          </div>
        </div>

        {/* Log */}
        <div className="bg-gray-50 rounded-lg p-4 flex flex-col h-[600px]">
          <div className="flex items-center justify-between mb-4">
            <h3 className="font-medium text-lg text-black">Log</h3>
            <div className="flex items-center gap-2">
              <button
                onClick={handleDownloadLogs}
                disabled={serialLogs.length === 0}
                className={`px-3 py-1 text-sm rounded flex items-center gap-1 ${
                  serialLogs.length === 0 
                    ? 'bg-gray-300 text-black cursor-not-allowed' 
                    : 'bg-green-600 hover:bg-green-700 text-white'
                }`}
              >
                <Download className="w-3 h-3" />
                Download
              </button>
              <button
                onClick={handleClearLog}
                className="px-3 py-1 text-sm rounded bg-red-100 hover:bg-red-200 text-red-700"
              >
                Clear
              </button>
            </div>
          </div>

          {/* Search and Filter Controls */}
          <div className="flex items-center gap-3 mb-3">
            <div className="flex-1 relative">
              <Search className="w-4 h-4 absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400" />
              <input
                type="text"
                placeholder="Search logs..."
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                className="w-full pl-10 pr-4 py-2 text-sm border border-gray-300 rounded-md text-black bg-white focus:outline-none focus:ring-2 focus:ring-blue-500"
              />
            </div>
            
            <div className="flex items-center gap-1">
              <Filter className="w-4 h-4 text-gray-600" />
              <Select.Root value={logFilter} onValueChange={(value: any) => setLogFilter(value)}>
                <Select.Trigger className="px-3 py-2 text-sm bg-white border border-gray-300 rounded-md text-black hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-blue-500">
                  <Select.Value />
                </Select.Trigger>
                <Select.Portal>
                  <Select.Content className="bg-white rounded-md shadow-lg border border-gray-200 mt-1">
                    <Select.Viewport>
                      <Select.Item value="all" className="px-3 py-2 hover:bg-gray-100 cursor-pointer text-black">
                        <Select.ItemText>All</Select.ItemText>
                      </Select.Item>
                      <Select.Item value="info" className="px-3 py-2 hover:bg-gray-100 cursor-pointer text-black">
                        <Select.ItemText>Info</Select.ItemText>
                      </Select.Item>
                      <Select.Item value="command" className="px-3 py-2 hover:bg-gray-100 cursor-pointer text-black">
                        <Select.ItemText>Commands</Select.ItemText>
                      </Select.Item>
                      <Select.Item value="response" className="px-3 py-2 hover:bg-gray-100 cursor-pointer text-black">
                        <Select.ItemText>Responses</Select.ItemText>
                      </Select.Item>
                      <Select.Item value="update" className="px-3 py-2 hover:bg-gray-100 cursor-pointer text-black">
                        <Select.ItemText>Updates</Select.ItemText>
                      </Select.Item>
                      <Select.Item value="error" className="px-3 py-2 hover:bg-gray-100 cursor-pointer text-black">
                        <Select.ItemText>Errors</Select.ItemText>
                      </Select.Item>
                    </Select.Viewport>
                  </Select.Content>
                </Select.Portal>
              </Select.Root>
            </div>
          </div>
          
          <div className="flex-1 bg-black text-green-400 font-mono text-xs p-4 rounded overflow-y-auto">
            {filteredLogs.length === 0 ? (
              <div className="text-gray-600">
                {serialLogs.length === 0 ? 'No log entries yet...' : 'No entries match the current filter.'}
              </div>
            ) : (
              filteredLogs.slice().reverse().map((entry, index) => (
                <div key={index} className={`mb-1 ${
                  entry.includes('COMMAND') ? 'text-yellow-400' :
                  isUpdateResponse(entry) ? 'text-cyan-400' :
                  entry.includes('RESPONSE') ? 'text-blue-400' :
                  entry.includes('ERROR') ? 'text-red-400' :
                  'text-white'
                }`}>
                  {entry}
                </div>
              ))
            )}
          </div>

          {/* Log Stats */}
          <div className="flex justify-between items-center mt-2 text-xs text-gray-600">
            <span>
              Showing {filteredLogs.length} of {serialLogs.length} entries
            </span>
            <span>
              {logFilter !== 'all' && `Filter: ${logFilter}`}
              {searchTerm && ` | Search: "${searchTerm}"`}
            </span>
          </div>
        </div>
      </div>
    </div>
  );
};
