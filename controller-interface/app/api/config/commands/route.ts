import { NextRequest, NextResponse } from 'next/server';
import { promises as fs } from 'fs';
import path from 'path';

export async function POST(request: NextRequest) {
  try {
    const { config } = await request.json();
    
    if (!config) {
      return NextResponse.json(
        { error: 'Configuration is required' },
        { status: 400 }
      );
    }

    const configPath = path.join(process.cwd(), 'config', 'CommandConfig.json');
    
    // Write the configuration to file
    await fs.writeFile(configPath, JSON.stringify(config, null, 2), 'utf8');
    
    return NextResponse.json({ 
      success: true, 
      message: 'Command configuration saved successfully' 
    });
  } catch (error) {
    console.error('Error saving command config:', error);
    return NextResponse.json(
      { error: 'Failed to save command configuration' },
      { status: 500 }
    );
  }
}

export async function GET() {
  try {
    const configPath = path.join(process.cwd(), 'config', 'CommandConfig.json');
    const configData = await fs.readFile(configPath, 'utf8');
    const config = JSON.parse(configData);
    
    return NextResponse.json({ config });
  } catch (error) {
    console.error('Error loading command config:', error);
    return NextResponse.json(
      { error: 'Failed to load command configuration' },
      { status: 500 }
    );
  }
}
