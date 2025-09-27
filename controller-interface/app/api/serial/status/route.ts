import { NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';

// GET /api/serial/status - Get connection status
export async function GET() {
  try {
    const serialManager = SerialManager.getInstance();
    
    // Wait for the robot config to be loaded (with a timeout)
    try {
      await serialManager.waitForInitialization(2000);
    } catch (initError) {
      console.warn('Robot config not loaded within timeout:', initError);
    }
    
    const connected = serialManager.getConnectionStatus();
    const config = serialManager.getCommandConfig();
    const robotConfig = serialManager.getRobotConfig();
    const latestState = serialManager.getLatestState?.() ?? null;
    
    return NextResponse.json({ connected, config, robotConfig, latestState });
  } catch (error) {
    console.error('Failed to get status:', error);
    return NextResponse.json({ 
      error: 'Failed to get status',
      message: error instanceof Error ? error.message : 'Unknown error'
    }, { status: 500 });
  }
}
