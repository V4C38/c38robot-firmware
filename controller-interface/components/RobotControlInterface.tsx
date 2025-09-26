'use client';

import React, { useState, useRef, useCallback } from 'react';
import * as Tabs from '@radix-ui/react-tabs';
import { Wifi, Sliders, TestTube, Settings } from 'lucide-react';
import { SerialPortTab } from './tabs/SerialPortTab';
import { AxisControlTab } from './tabs/AxisControlTab';
import { TestingTab } from './tabs/TestingTab';
import { ConfigTab } from './tabs/ConfigTab';
import dynamic from 'next/dynamic';
// Load 3D viewer purely on the client to avoid SSR hydration ID drift
const RobotArm3D = dynamic(() => import('./RobotArm3D'), { ssr: false });

export const RobotControlInterface: React.FC = () => {
  const [leftWidth, setLeftWidth] = useState(60); // percentage - 60% for tabs, 40% for 3D viewer
  const [isResizing, setIsResizing] = useState(false);
  const [containerSize, setContainerSize] = useState({ width: 800, height: 600 });
  const containerRef = useRef<HTMLDivElement>(null);

  const handleMouseDown = useCallback(() => {
    setIsResizing(true);
  }, []);

  const handleMouseMove = useCallback((e: MouseEvent) => {
    if (!isResizing || !containerRef.current) return;

    const containerRect = containerRef.current.getBoundingClientRect();
    const newLeftWidth = ((e.clientX - containerRect.left) / containerRect.width) * 100;
    
    // Constrain between 20% and 80%
    const constrainedWidth = Math.min(Math.max(newLeftWidth, 20), 80);
    setLeftWidth(constrainedWidth);
  }, [isResizing]);

  const handleMouseUp = useCallback(() => {
    setIsResizing(false);
  }, []);

  // Update container size on resize
  React.useEffect(() => {
    const updateSize = () => {
      if (containerRef.current) {
        const rect = containerRef.current.getBoundingClientRect();
        const newSize = { width: rect.width, height: rect.height };
        setContainerSize(newSize);
      }
    };

    // Use a small delay to ensure the DOM is fully rendered
    setTimeout(updateSize, 100);
    window.addEventListener('resize', updateSize);

    return () => window.removeEventListener('resize', updateSize);
  }, [leftWidth]);

  React.useEffect(() => {
    if (isResizing) {
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
      document.body.style.cursor = 'col-resize';
      document.body.style.userSelect = 'none';
    } else {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
    }

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
    };
  }, [isResizing, handleMouseMove, handleMouseUp]);

  return (
    <div ref={containerRef} className="h-screen bg-gray-100 flex overflow-hidden">
      {/* Left Panel - Tabs */}
      <div 
        className="bg-white shadow-lg" 
        style={{ width: `${leftWidth}%` }}
      >
        <Tabs.Root defaultValue="serial" className="h-full flex flex-col">
          {/* Tab List */}
          <Tabs.List className="flex border-b border-gray-200 bg-gray-50">
            <Tabs.Trigger
              value="serial"
              className="flex-1 px-6 py-3 font-medium text-gray-700 hover:text-gray-900 border-b-2 border-transparent data-[state=active]:border-blue-500 data-[state=active]:text-blue-600 transition-colors flex items-center justify-center gap-2"
            >
              <Wifi className="w-4 h-4" />
              Serial Port
            </Tabs.Trigger>
            <Tabs.Trigger
              value="axis"
              className="flex-1 px-6 py-3 font-medium text-gray-700 hover:text-gray-900 border-b-2 border-transparent data-[state=active]:border-blue-500 data-[state=active]:text-blue-600 transition-colors flex items-center justify-center gap-2"
            >
              <Sliders className="w-4 h-4" />
              Axis Control
            </Tabs.Trigger>
              <Tabs.Trigger
                value="testing"
                className="flex-1 px-6 py-3 font-medium text-gray-700 hover:text-gray-900 border-b-2 border-transparent data-[state=active]:border-blue-500 data-[state=active]:text-blue-600 transition-colors flex items-center justify-center gap-2"
              >
                <TestTube className="w-4 h-4" />
                Testing
              </Tabs.Trigger>
              <Tabs.Trigger
                value="config"
                className="flex-1 px-6 py-3 font-medium text-gray-700 hover:text-gray-900 border-b-2 border-transparent data-[state=active]:border-blue-500 data-[state=active]:text-blue-600 transition-colors flex items-center justify-center gap-2"
              >
                <Settings className="w-4 h-4" />
                Config
              </Tabs.Trigger>
          </Tabs.List>

          {/* Tab Content */}
          <div className="flex-1 overflow-y-auto">
            <Tabs.Content value="serial" className="h-full">
              <SerialPortTab />
            </Tabs.Content>
            <Tabs.Content value="axis" className="h-full">
              <AxisControlTab />
            </Tabs.Content>
            <Tabs.Content value="testing" className="h-full">
              <TestingTab />
            </Tabs.Content>
            <Tabs.Content value="config" className="h-full">
              <ConfigTab />
            </Tabs.Content>
          </div>
        </Tabs.Root>
      </div>

      {/* Resizer */}
      <div 
        className="w-1 bg-gray-300 hover:bg-gray-400 cursor-col-resize transition-colors relative group"
        onMouseDown={handleMouseDown}
      >
        <div className="absolute inset-y-0 -inset-x-1 group-hover:bg-blue-500 group-hover:bg-opacity-20 transition-colors" />
      </div>

      {/* Right Panel - 3D Visualization */}
      <div 
        className="border-l border-gray-300 relative"
        style={{ width: `${100 - leftWidth}%` }}
      >
        <RobotArm3D 
          width={Math.max(400, containerSize.width * (100 - leftWidth) / 100)} 
          height={containerSize.height}
          className="absolute inset-0 w-full h-full"
        />
      </div>
    </div>
  );
};