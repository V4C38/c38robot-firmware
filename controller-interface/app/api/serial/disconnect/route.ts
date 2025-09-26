import { NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';

// POST /api/serial/disconnect - Disconnect from serial port
export async function POST() {
  try {
    const serialManager = SerialManager.getInstance();
    // Prefer graceful close, then hard-release as fallback
    await serialManager.closePort();
    // Brief pause to allow OS to release handle
    await new Promise(resolve => setTimeout(resolve, 500));
    
    return NextResponse.json({ success: true, connected: false });
  } catch (error) {
    console.error('Failed to disconnect from serial port:', error);
    return NextResponse.json({ 
      error: 'Failed to disconnect from serial port',
      message: error instanceof Error ? error.message : 'Unknown error'
    }, { status: 500 });
  }
}
