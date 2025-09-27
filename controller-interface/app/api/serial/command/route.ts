import { NextRequest, NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';
import type { Command } from '@/types/command.types';

// POST /api/serial/command - Send command to serial port
export async function POST(request: NextRequest) {
  try {
    const command = await request.json() as Omit<Command, 'uuid' | 'type'>;

    const serialManager = SerialManager.getInstance();
    // For getState, coalesce concurrent requests to avoid flooding the MCU
    const commandName = (command as unknown as { command: string }).command;
    if (commandName === 'getState') {
      const { command: sentCommand, response } = await serialManager.sendGetStateCoalesced(command);
      return NextResponse.json({ success: true, command: sentCommand, response });
    }

    // For other commands, wait for their specific response
    const { command: sentCommand, response } = await serialManager.sendCommandAndWait(command);

    return NextResponse.json({ success: true, command: sentCommand, response });
  } catch (error) {
    console.error('Failed to send command:', error);
    try {
      const serialManager = SerialManager.getInstance();
      await serialManager.appendLog(`API error sending command: ${error instanceof Error ? error.message : 'Unknown error'}`, 'ERROR');
    } catch {}
    return NextResponse.json({ 
      error: 'Failed to send command',
      message: error instanceof Error ? error.message : 'Unknown error'
    }, { status: 500 });
  }
}
