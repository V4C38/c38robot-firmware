import { NextRequest, NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';

// POST /api/serial/connect - Connect to serial port
export async function POST(request: NextRequest) {
  try {
    const { port, baudRate = 115200 } = await request.json();
    
    if (!port) {
      return NextResponse.json({ error: 'Port is required' }, { status: 400 });
    }

    const serialManager = SerialManager.getInstance();
    const success = await serialManager.openPort(port, baudRate);
    
    return NextResponse.json({ success, connected: success });
  } catch (error) {
    console.error('Failed to connect to serial port:', error);
    return NextResponse.json({ 
      error: 'Failed to connect to serial port',
      message: error instanceof Error ? error.message : 'Unknown error'
    }, { status: 500 });
  }
}
