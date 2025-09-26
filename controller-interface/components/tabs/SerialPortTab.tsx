'use client';

import React, { useState } from 'react';
import { useRobot } from '@/contexts/RobotContext';
import { RefreshCw, Wifi, WifiOff } from 'lucide-react';
import * as Select from '@radix-ui/react-select';

export const SerialPortTab: React.FC = () => {
  const { 
    availablePorts, 
    selectedPort, 
    isConnected, 
    refreshPorts, 
    connectToPort, 
    disconnect,
    setSelectedPort,
    baudRate,
    setBaudRate
  } = useRobot();
  
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);

  const handleRefresh = async () => {
    setIsRefreshing(true);
    await refreshPorts();
    setTimeout(() => setIsRefreshing(false), 500);
  };

  const handleConnect = async () => {
    if (isConnected) {
      await disconnect();
    } else if (selectedPort) {
      setIsConnecting(true);
      const success = await connectToPort(selectedPort, baudRate);
      setIsConnecting(false);
      if (!success) {
        // Try force disconnect first, then retry
        try {
          await fetch('/api/serial/force-disconnect', { method: 'POST' });
          await new Promise(resolve => setTimeout(resolve, 1000));
          const retrySuccess = await connectToPort(selectedPort, baudRate);
          if (!retrySuccess) {
            alert('Failed to connect to serial port. The port may be locked by another application. Try unplugging and reconnecting the device.');
          }
        } catch {
          alert('Failed to connect to serial port. The port may be locked by another application. Try unplugging and reconnecting the device.');
        }
      }
    }
  };

  const handlePortSelect = (value: string) => {
    if (isConnected) {
      disconnect();
    }
    setSelectedPort(value);
  };

  return (
    <div className="p-6 space-y-6">
      <div>
        <h2 className="text-2xl font-bold mb-4 text-black">Serial Port Configuration</h2>
        <p className="text-gray-600">
          Select and connect to a serial port to communicate with the robot arm.
        </p>
      </div>

      <div className="space-y-4">
        {/* Port Selection */}
        <div className="space-y-2">
          <label className="block text-sm font-medium text-black">
            Available Serial Ports
          </label>
          <div className="flex gap-2">
            <Select.Root value={selectedPort || ''} onValueChange={handlePortSelect}>
              <Select.Trigger 
                className="flex-1 px-4 py-2 bg-white border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500 text-black"
              >
                <Select.Value placeholder="Select a serial port..." />
              </Select.Trigger>
              <Select.Portal>
                <Select.Content className="bg-white rounded-md shadow-lg border border-gray-200 mt-1">
                  <Select.Viewport>
                    {availablePorts.length === 0 ? (
                      <Select.Item value="_none" className="px-4 py-2 text-gray-500">
                        <Select.ItemText>No ports available</Select.ItemText>
                      </Select.Item>
                    ) : (
                      availablePorts.map((port) => (
                        <Select.Item
                          key={port.path}
                          value={port.path}
                          className="px-4 py-2 hover:bg-gray-100 cursor-pointer text-black"
                        >
                          <Select.ItemText>
                            {port.path} {port.manufacturer && `(${port.manufacturer})`}
                          </Select.ItemText>
                        </Select.Item>
                      ))
                    )}
                  </Select.Viewport>
                </Select.Content>
              </Select.Portal>
            </Select.Root>
            
            <button
              onClick={handleRefresh}
              disabled={isRefreshing}
              className="px-4 py-2 bg-gray-100 hover:bg-gray-200 text-gray-800 hover:text-gray-900 rounded-md transition-colors disabled:opacity-50"
            >
              <RefreshCw className={`w-5 h-5 ${isRefreshing ? 'animate-spin' : ''}`} />
            </button>
          </div>
        </div>

        {/* Connection Status */}
        <div className="flex items-center justify-between p-4 bg-gray-50 rounded-lg">
          <div className="flex items-center gap-3">
            {isConnected ? (
              <Wifi className="w-6 h-6 text-green-500" />
            ) : (
              <WifiOff className="w-6 h-6 text-gray-400" />
            )}
            <div>
              <p className="font-medium text-black">
                {isConnected ? 'Connected' : 'Disconnected'}
              </p>
              {isConnected && selectedPort && (
                <p className="text-sm text-gray-600">{selectedPort}</p>
              )}
            </div>
          </div>
          
          <button
            onClick={handleConnect}
            disabled={!selectedPort || isConnecting}
            className={`px-6 py-2 rounded-md font-medium transition-colors ${
              (!selectedPort || isConnecting) 
                ? 'bg-gray-300 text-black cursor-not-allowed' 
                : isConnected 
                  ? 'bg-red-500 hover:bg-red-600 text-white' 
                  : 'bg-blue-500 hover:bg-blue-600 text-white'
            }`}
          >
            {isConnecting ? 'Connecting...' : isConnected ? 'Disconnect' : 'Connect'}
          </button>
        </div>

        {/* Connection Info */}
        <div className="p-4 bg-blue-50 rounded-lg">
          <h3 className="font-medium text-black mb-2">Connection Settings</h3>
          <div className="space-y-3 text-sm text-blue-800">
            <div>
              <label className="block text-sm font-medium text-black mb-1">Baud Rate</label>
              <Select.Root value={String(baudRate)} onValueChange={(v) => setBaudRate(Number(v))}>
                <Select.Trigger 
                  className="px-4 py-2 bg-white border border-gray-300 rounded-md shadow-sm focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500 text-black"
                >
                  <Select.Value />
                </Select.Trigger>
                <Select.Portal>
                  <Select.Content className="bg-white rounded-md shadow-lg border border-gray-200 mt-1">
                    <Select.Viewport>
                      {[9600, 19200, 38400, 57600, 115200, 230400, 250000].map((rate) => (
                        <Select.Item key={rate} value={String(rate)} className="px-4 py-2 hover:bg-gray-100 cursor-pointer text-black">
                          <Select.ItemText>{rate}</Select.ItemText>
                        </Select.Item>
                      ))}
                    </Select.Viewport>
                  </Select.Content>
                </Select.Portal>
              </Select.Root>
            </div>
            <p>Data Bits: 8</p>
            <p>Stop Bits: 1</p>
            <p>Parity: None</p>
          </div>
        </div>
      </div>
    </div>
  );
};
