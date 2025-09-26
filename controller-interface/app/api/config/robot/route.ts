import { NextRequest, NextResponse } from 'next/server';
import { promises as fs } from 'fs';
import path from 'path';
import SerialManager from '@/lib/server/SerialManager';

export async function POST(request: NextRequest) {
  try {
    const { config } = await request.json();
    
    if (!config) {
      return NextResponse.json(
        { error: 'Configuration is required' },
        { status: 400 }
      );
    }

    const configPath = path.join(process.cwd(), 'config', 'robot_arm_config.json');
    
    // Write the configuration to file
    await fs.writeFile(configPath, JSON.stringify(config, null, 2), 'utf8');

    // Update in-memory SerialManager config for immediate effect
    try {
      const serialManager = SerialManager.getInstance();
      serialManager.setRobotConfig(config);
    } catch (e) {
      console.warn('Could not update SerialManager robot config:', e);
    }
    
    return NextResponse.json({ 
      success: true, 
      message: 'Robot configuration saved successfully' 
    });
  } catch (error) {
    console.error('Error saving robot config:', error);
    return NextResponse.json(
      { error: 'Failed to save robot configuration' },
      { status: 500 }
    );
  }
}

export async function GET() {
  try {
    const configPath = path.join(process.cwd(), 'config', 'robot_arm_config.json');
    const configData = await fs.readFile(configPath, 'utf8');
    const config = JSON.parse(configData);
    
    return NextResponse.json({ config });
  } catch (error) {
    console.error('Error loading robot config:', error);
    return NextResponse.json(
      { error: 'Failed to load robot configuration' },
      { status: 500 }
    );
  }
}
