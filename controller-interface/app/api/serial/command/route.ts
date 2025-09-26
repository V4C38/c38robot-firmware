import { NextRequest, NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';

// POST /api/serial/command - Send command to serial port
export async function POST(request: NextRequest) {
  try {
    const command = await request.json();
    
    const serialManager = SerialManager.getInstance();
    const sentCommand = await serialManager.sendCommand(command);
    
    return NextResponse.json({ success: true, command: sentCommand });
  } catch (error) {
    console.error('Failed to send command:', error);
    return NextResponse.json({ 
      error: 'Failed to send command',
      message: error instanceof Error ? error.message : 'Unknown error'
    }, { status: 500 });
  }
}
