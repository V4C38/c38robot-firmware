import React from 'react';
import * as Slider from '@radix-ui/react-slider';
import type { RobotJointConfig } from '@/types/robot-config.types';

interface AxisSliderProps {
  value: number;
  currentValue: number;
  robotJoint?: RobotJointConfig;
  fallbackMin: number;
  fallbackMax: number;
  onChange: (value: number[]) => void;
  onCommit: (value: number[]) => void;
  collapsed?: boolean;
  isAtLimit?: boolean;
  limitIndex?: number | null;
}

export const AxisSlider: React.FC<AxisSliderProps> = ({
  value,
  currentValue,
  robotJoint,
  fallbackMin,
  fallbackMax,
  onChange,
  onCommit,
  collapsed = false,
  isAtLimit,
  limitIndex
}) => {
  // Use robot config limits if available, otherwise fall back to joint limits
  const actualMin = robotJoint?.minAngle ?? fallbackMin;
  const actualMax = robotJoint?.maxAngle ?? fallbackMax;
  const fullRangeMin = -180;
  const fullRangeMax = 180;
  
  // Calculate slider width as percentage of full range
  const sliderWidthPercent = ((actualMax - actualMin) / (fullRangeMax - fullRangeMin)) * 100;
  
  // Calculate offset from center (0 degrees)
  const centerOffset = ((actualMin + actualMax) / 2) / (fullRangeMax - fullRangeMin) * 100;
  
  // Calculate current value position relative to center
  const currentValueFromCenter = currentValue / (fullRangeMax - fullRangeMin) * 100;
  const targetValueFromCenter = value / (fullRangeMax - fullRangeMin) * 100;

  if (collapsed) {
    // Collapsed view: only show current value progress bar
    return (
      <div className="flex items-center gap-3">
        <span className="text-xs w-12 text-gray-600">
          {actualMin}°
        </span>
        
        <div className="relative flex items-center justify-center w-full h-4">
          <div 
            className={`relative h-1.5 rounded-full ${isAtLimit ? 'bg-red-200' : 'bg-gray-200'}`}
            style={{ width: `${sliderWidthPercent}%` }}
          >
            {/* Center line at 0° */}
            <div 
              className="absolute w-0.5 h-full bg-gray-400"
              style={{ 
                left: `${Math.abs(actualMin) / (actualMax - actualMin) * 100}%`,
                transform: 'translateX(-50%)'
              }}
            />
            
            {/* Current value fill from center */}
            <div 
              className={`absolute ${isAtLimit ? 'bg-red-500' : 'bg-green-500'} h-1.5 rounded-full transition-all duration-300`}
              style={{
                left: currentValue >= 0 
                  ? `${Math.abs(actualMin) / (actualMax - actualMin) * 100}%`
                  : `${(Math.abs(actualMin) + currentValue) / (actualMax - actualMin) * 100}%`,
                width: `${Math.abs(currentValue) / (actualMax - actualMin) * 100}%`
              }}
            />
            
            {/* Current position indicator */}
            <div 
              className={`absolute w-2.5 h-2.5 ${isAtLimit ? 'bg-red-600' : 'bg-green-600'} border border-white rounded-full shadow-sm transform -translate-y-0.5 transition-all duration-300`}
              style={{
                left: `calc(${(currentValue - actualMin) / (actualMax - actualMin) * 100}% - 5px)`
              }}
            />
            {isAtLimit && (
              <div className="absolute -top-5 text-xs text-red-600">
                {limitIndex === 0 ? 'Left limit' : limitIndex === 1 ? 'Right limit' : 'Limit'}
              </div>
            )}
          </div>
        </div>
        
        <span className="text-xs w-12 text-gray-600 text-right">
          {actualMax}°
        </span>
      </div>
    );
  }

  // Expanded view: show both target and current sliders
  return (
    <div className="space-y-2">
      {/* Target Value Slider with correct range display */}
      <div className="flex items-center gap-3">
        <span className="text-xs w-12 text-gray-600">
          {actualMin}°
        </span>
        
        <div className="relative flex items-center justify-center w-full h-4">
          {/* Slider container with dynamic width */}
          <div 
            className={`relative h-1.5 rounded-full ${isAtLimit ? 'bg-red-200' : 'bg-gray-200'}`}
            style={{ width: `${sliderWidthPercent}%` }}
          >
            {/* Center line at 0° */}
            <div 
              className="absolute w-0.5 h-full bg-gray-400"
              style={{ 
                left: `${Math.abs(actualMin) / (actualMax - actualMin) * 100}%`,
                transform: 'translateX(-50%)'
              }}
            />
            
            {/* Target value fill from center */}
            <div 
              className="absolute bg-blue-500 h-full rounded-full transition-all duration-200"
              style={{
                left: value >= 0 
                  ? `${Math.abs(actualMin) / (actualMax - actualMin) * 100}%`
                  : `${(Math.abs(actualMin) + value) / (actualMax - actualMin) * 100}%`,
                width: `${Math.abs(value) / (actualMax - actualMin) * 100}%`
              }}
            />
            
            {/* Slider component */}
            <Slider.Root
              className="absolute inset-0 flex items-center select-none touch-none"
              value={[value]}
              onValueChange={onChange}
              onValueCommit={onCommit}
              max={actualMax}
              min={actualMin}
              step={0.1}
            >
              <Slider.Track className="relative grow h-1.5 rounded-full bg-transparent">
                <Slider.Range className="absolute bg-transparent h-full rounded-full" />
              </Slider.Track>
              <Slider.Thumb className={`block w-4 h-4 ${isAtLimit ? 'bg-red-500 border-red-600' : 'bg-white border-blue-500'} border-2 rounded-full shadow-md focus:outline-none focus:ring-1 focus:ring-blue-500`} />
            </Slider.Root>
          </div>
        </div>
        
        <span className="text-xs w-12 text-gray-600 text-right">
          {actualMax}°
        </span>
      </div>

      {/* Current Value Progress Bar */}
      <div className="flex items-center gap-3">
        <span className="text-xs w-12 text-gray-600">
          {actualMin}°
        </span>
        
        <div className="relative flex items-center justify-center w-full h-4">
          {/* Progress bar container with dynamic width */}
          <div 
            className={`relative h-1.5 rounded-full ${isAtLimit ? 'bg-red-200' : 'bg-gray-200'}`}
            style={{ width: `${sliderWidthPercent}%` }}
          >
            {/* Center line at 0° */}
            <div 
              className="absolute w-0.5 h-full bg-gray-400"
              style={{ 
                left: `${Math.abs(actualMin) / (actualMax - actualMin) * 100}%`,
                transform: 'translateX(-50%)'
              }}
            />
            
            {/* Current value fill from center */}
            <div 
              className={`absolute ${isAtLimit ? 'bg-red-500' : 'bg-green-500'} h-1.5 rounded-full transition-all duration-300`}
              style={{
                left: currentValue >= 0 
                  ? `${Math.abs(actualMin) / (actualMax - actualMin) * 100}%`
                  : `${(Math.abs(actualMin) + currentValue) / (actualMax - actualMin) * 100}%`,
                width: `${Math.abs(currentValue) / (actualMax - actualMin) * 100}%`
              }}
            />
            
            {/* Current position indicator */}
            <div 
              className={`absolute w-2.5 h-2.5 ${isAtLimit ? 'bg-red-600' : 'bg-green-600'} border border-white rounded-full shadow-sm transform -translate-y-0.5 transition-all duration-300`}
              style={{
                left: `calc(${(currentValue - actualMin) / (actualMax - actualMin) * 100}% - 5px)`
              }}
            />
            {isAtLimit && (
              <div className="absolute -top-5 text-xs text-red-600">
                {limitIndex === 0 ? 'Left limit' : limitIndex === 1 ? 'Right limit' : 'Limit'}
              </div>
            )}
          </div>
        </div>
        
        <span className="text-xs w-12 text-gray-600 text-right">
          {actualMax}°
        </span>
      </div>
    </div>
  );
};
