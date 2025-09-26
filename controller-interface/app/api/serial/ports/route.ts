import { NextResponse } from 'next/server';

// GET /api/serial/ports - Get available serial ports
export async function GET() {
  try {
    const { SerialPort } = await import('serialport');
    const ports = await SerialPort.list();
    return NextResponse.json({ ports });
  } catch (error) {
    console.error('Failed to list serial ports:', error);
    return NextResponse.json({ error: 'Failed to list serial ports' }, { status: 500 });
  }
}
