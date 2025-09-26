import { NextResponse } from 'next/server';
import { promises as fs } from 'fs';
import path from 'path';

export async function POST() {
  try {
    // Read both configuration files to trigger reload
    const robotConfigPath = path.join(process.cwd(), 'config', 'robot_arm_config.json');
    const commandConfigPath = path.join(process.cwd(), 'config', 'CommandConfig.json');
    
    // Verify files exist and are valid JSON
    const robotConfigData = await fs.readFile(robotConfigPath, 'utf8');
    const commandConfigData = await fs.readFile(commandConfigPath, 'utf8');
    
    // Parse to validate JSON
    const robotConfig = JSON.parse(robotConfigData);
    const commandConfig = JSON.parse(commandConfigData);
    
    return NextResponse.json({ 
      success: true, 
      message: 'Configuration reloaded successfully',
      robotConfig,
      commandConfig
    });
  } catch (error) {
    console.error('Error reloading config:', error);
    return NextResponse.json(
      { error: 'Failed to reload configuration' },
      { status: 500 }
    );
  }
}
