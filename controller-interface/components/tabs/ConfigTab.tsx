'use client';

import React, { useState, useCallback } from 'react';
import { useRobot } from '@/contexts/RobotContext';
import { Save, RefreshCw, Download, Upload, AlertTriangle, CheckCircle } from 'lucide-react';

interface ConfigFile {
  name: string;
  content: string;
  filename: string;
  isValid: boolean;
  error?: string;
}

export const ConfigTab: React.FC = () => {
  const { robotConfig, commandConfig, reloadConfig } = useRobot();
  const [robotConfigText, setRobotConfigText] = useState(
    JSON.stringify(robotConfig, null, 2) || ''
  );
  const [commandConfigText, setCommandConfigText] = useState(
    JSON.stringify(commandConfig, null, 2) || ''
  );
  const [robotConfigValid, setRobotConfigValid] = useState(true);
  const [commandConfigValid, setCommandConfigValid] = useState(true);
  const [robotConfigError, setRobotConfigError] = useState('');
  const [commandConfigError, setCommandConfigError] = useState('');
  const [saveStatus, setSaveStatus] = useState<'idle' | 'saving' | 'success' | 'error'>('idle');
  const [reloadStatus, setReloadStatus] = useState<'idle' | 'reloading' | 'success' | 'error'>('idle');

  // Validate JSON
  const validateJSON = useCallback((text: string, setValid: (valid: boolean) => void, setError: (error: string) => void) => {
    try {
      JSON.parse(text);
      setValid(true);
      setError('');
      return true;
    } catch (error) {
      setValid(false);
      setError(error instanceof Error ? error.message : 'Invalid JSON');
      return false;
    }
  }, []);

  // Handle robot config changes
  const handleRobotConfigChange = useCallback((value: string) => {
    setRobotConfigText(value);
    validateJSON(value, setRobotConfigValid, setRobotConfigError);
  }, [validateJSON]);

  // Handle command config changes
  const handleCommandConfigChange = useCallback((value: string) => {
    setCommandConfigText(value);
    validateJSON(value, setCommandConfigValid, setCommandConfigError);
  }, [validateJSON]);

  // Save robot config
  const saveRobotConfig = useCallback(async () => {
    if (!robotConfigValid) return;
    
    setSaveStatus('saving');
    try {
      const response = await fetch('/api/config/robot', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          config: JSON.parse(robotConfigText)
        }),
      });

      if (response.ok) {
        setSaveStatus('success');
        setTimeout(() => setSaveStatus('idle'), 2000);
      } else {
        setSaveStatus('error');
        setTimeout(() => setSaveStatus('idle'), 3000);
      }
    } catch (error) {
      console.error('Failed to save robot config:', error);
      setSaveStatus('error');
      setTimeout(() => setSaveStatus('idle'), 3000);
    }
  }, [robotConfigText, robotConfigValid]);

  // Save command config
  const saveCommandConfig = useCallback(async () => {
    if (!commandConfigValid) return;
    
    setSaveStatus('saving');
    try {
      const response = await fetch('/api/config/commands', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          config: JSON.parse(commandConfigText)
        }),
      });

      if (response.ok) {
        setSaveStatus('success');
        setTimeout(() => setSaveStatus('idle'), 2000);
      } else {
        setSaveStatus('error');
        setTimeout(() => setSaveStatus('idle'), 3000);
      }
    } catch (error) {
      console.error('Failed to save command config:', error);
      setSaveStatus('error');
      setTimeout(() => setSaveStatus('idle'), 3000);
    }
  }, [commandConfigText, commandConfigValid]);

  // Reload arm configuration
  const reloadArmConfig = useCallback(async () => {
    setReloadStatus('reloading');
    try {
      await reloadConfig();
      setReloadStatus('success');
      setTimeout(() => setReloadStatus('idle'), 2000);
      
      // Update the text areas with the new config
      setRobotConfigText(JSON.stringify(robotConfig, null, 2) || '');
      setCommandConfigText(JSON.stringify(commandConfig, null, 2) || '');
    } catch (error) {
      console.error('Failed to reload config:', error);
      setReloadStatus('error');
      setTimeout(() => setReloadStatus('idle'), 3000);
    }
  }, [reloadConfig, robotConfig, commandConfig]);

  // Download config file
  const downloadConfig = useCallback((content: string, filename: string) => {
    const blob = new Blob([content], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }, []);

  // Load config file
  const loadConfigFile = useCallback((file: File, setConfig: (content: string) => void) => {
    const reader = new FileReader();
    reader.onload = (e) => {
      if (e.target?.result) {
        setConfig(e.target.result as string);
      }
    };
    reader.readAsText(file);
  }, []);

  return (
    <div className="p-6 space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-2xl font-bold text-black">Configuration</h2>
          <p className="text-gray-600 mt-1">
            Edit robot and command configurations
          </p>
        </div>
        
        <div className="flex items-center gap-3">
          <button
            onClick={reloadArmConfig}
            disabled={reloadStatus === 'reloading'}
            className={`px-4 py-2 font-medium rounded-md flex items-center gap-2 transition-colors text-sm ${
              reloadStatus === 'reloading'
                ? 'bg-gray-300 text-black cursor-not-allowed'
                : reloadStatus === 'success'
                ? 'bg-green-600 text-white'
                : reloadStatus === 'error'
                ? 'bg-red-600 text-white'
                : 'bg-blue-600 hover:bg-blue-700 text-white'
            }`}
          >
            <RefreshCw className={`w-4 h-4 ${reloadStatus === 'reloading' ? 'animate-spin' : ''}`} />
            {reloadStatus === 'reloading' ? 'Reloading...' : 
             reloadStatus === 'success' ? 'Reloaded!' :
             reloadStatus === 'error' ? 'Failed' : 'Reload Arm Config'}
          </button>
        </div>
      </div>

      {/* Robot Configuration */}
      <div className="bg-gray-50 rounded-lg p-4">
        <div className="flex items-center justify-between mb-3">
          <div className="flex items-center gap-2">
            <h3 className="text-lg font-medium text-black">Robot Arm Configuration</h3>
            {robotConfigValid ? (
              <CheckCircle className="w-5 h-5 text-green-600" />
            ) : (
              <AlertTriangle className="w-5 h-5 text-red-600" />
            )}
          </div>
          <div className="flex items-center gap-2">
            <button
              onClick={() => downloadConfig(robotConfigText, 'robot_arm_config.json')}
              className="px-3 py-1 text-sm bg-gray-200 hover:bg-gray-300 text-black rounded-md flex items-center gap-1 transition-colors"
            >
              <Download className="w-3 h-3" />
              Download
            </button>
            <label className="px-3 py-1 text-sm bg-gray-200 hover:bg-gray-300 text-black rounded-md flex items-center gap-1 transition-colors cursor-pointer">
              <Upload className="w-3 h-3" />
              Upload
              <input
                type="file"
                accept=".json"
                className="hidden"
                onChange={(e) => {
                  const file = e.target.files?.[0];
                  if (file) {
                    loadConfigFile(file, handleRobotConfigChange);
                  }
                }}
              />
            </label>
            <button
              onClick={saveRobotConfig}
              disabled={!robotConfigValid || saveStatus === 'saving'}
              className={`px-3 py-1 text-sm font-medium rounded-md flex items-center gap-1 transition-colors ${
                !robotConfigValid || saveStatus === 'saving'
                  ? 'bg-gray-300 text-black cursor-not-allowed'
                  : saveStatus === 'success'
                  ? 'bg-green-600 text-white'
                  : saveStatus === 'error'
                  ? 'bg-red-600 text-white'
                  : 'bg-blue-600 hover:bg-blue-700 text-white'
              }`}
            >
              <Save className="w-3 h-3" />
              {saveStatus === 'saving' ? 'Saving...' : 
               saveStatus === 'success' ? 'Saved!' :
               saveStatus === 'error' ? 'Failed' : 'Save'}
            </button>
          </div>
        </div>

        {!robotConfigValid && (
          <div className="mb-3 p-2 bg-red-50 border border-red-200 rounded text-red-700 text-sm">
            {robotConfigError}
          </div>
        )}

        <textarea
          value={robotConfigText}
          onChange={(e) => handleRobotConfigChange(e.target.value)}
          className="w-full h-80 p-3 font-mono text-sm border border-gray-300 rounded-md bg-white text-black resize-none"
          placeholder="Robot configuration JSON..."
        />
      </div>

      {/* Command Configuration */}
      <div className="bg-gray-50 rounded-lg p-4">
        <div className="flex items-center justify-between mb-3">
          <div className="flex items-center gap-2">
            <h3 className="text-lg font-medium text-black">Command Configuration</h3>
            {commandConfigValid ? (
              <CheckCircle className="w-5 h-5 text-green-600" />
            ) : (
              <AlertTriangle className="w-5 h-5 text-red-600" />
            )}
          </div>
          <div className="flex items-center gap-2">
            <button
              onClick={() => downloadConfig(commandConfigText, 'CommandConfig.json')}
              className="px-3 py-1 text-sm bg-gray-200 hover:bg-gray-300 text-black rounded-md flex items-center gap-1 transition-colors"
            >
              <Download className="w-3 h-3" />
              Download
            </button>
            <label className="px-3 py-1 text-sm bg-gray-200 hover:bg-gray-300 text-black rounded-md flex items-center gap-1 transition-colors cursor-pointer">
              <Upload className="w-3 h-3" />
              Upload
              <input
                type="file"
                accept=".json"
                className="hidden"
                onChange={(e) => {
                  const file = e.target.files?.[0];
                  if (file) {
                    loadConfigFile(file, handleCommandConfigChange);
                  }
                }}
              />
            </label>
            <button
              onClick={saveCommandConfig}
              disabled={!commandConfigValid || saveStatus === 'saving'}
              className={`px-3 py-1 text-sm font-medium rounded-md flex items-center gap-1 transition-colors ${
                !commandConfigValid || saveStatus === 'saving'
                  ? 'bg-gray-300 text-black cursor-not-allowed'
                  : saveStatus === 'success'
                  ? 'bg-green-600 text-white'
                  : saveStatus === 'error'
                  ? 'bg-red-600 text-white'
                  : 'bg-blue-600 hover:bg-blue-700 text-white'
              }`}
            >
              <Save className="w-3 h-3" />
              {saveStatus === 'saving' ? 'Saving...' : 
               saveStatus === 'success' ? 'Saved!' :
               saveStatus === 'error' ? 'Failed' : 'Save'}
            </button>
          </div>
        </div>

        {!commandConfigValid && (
          <div className="mb-3 p-2 bg-red-50 border border-red-200 rounded text-red-700 text-sm">
            {commandConfigError}
          </div>
        )}

        <textarea
          value={commandConfigText}
          onChange={(e) => handleCommandConfigChange(e.target.value)}
          className="w-full h-80 p-3 font-mono text-sm border border-gray-300 rounded-md bg-white text-black resize-none"
          placeholder="Command configuration JSON..."
        />
      </div>
    </div>
  );
};
