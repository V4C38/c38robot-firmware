import { NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';

// GET /api/serial/ports - Get available serial ports
export async function GET() {
  try {
    const ports = await SerialManager.getInstance().getAvailablePorts();
    return NextResponse.json({ ports });
  } catch (error) {
    console.error('Failed to list serial ports:', error);
    return NextResponse.json({ error: 'Failed to list serial ports' }, { status: 500 });
  }
}
