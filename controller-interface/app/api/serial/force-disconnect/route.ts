import { NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';

// POST /api/serial/force-disconnect - Force disconnect and cleanup
export async function POST() {
  try {
    const serialManager = SerialManager.getInstance();
    // Force close with cleanup to guarantee release
    await serialManager.forceDisconnect();
    
    return NextResponse.json({ 
      success: true, 
      connected: false,
      message: 'Port forcefully disconnected and cleaned up' 
    });
  } catch (error) {
    console.error('Failed to force disconnect:', error);
    return NextResponse.json({ 
      error: 'Failed to force disconnect',
      message: error instanceof Error ? error.message : 'Unknown error'
    }, { status: 500 });
  }
}
