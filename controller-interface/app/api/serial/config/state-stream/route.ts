import { NextRequest, NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';

export async function POST(request: NextRequest) {
  try {
    const { enabled } = await request.json();
    if (typeof enabled !== 'boolean') {
      return NextResponse.json({ error: 'Invalid enabled' }, { status: 400 });
    }
    const serialManager = SerialManager.getInstance();
    const { response } = await serialManager.sendCommandAndWait({ command: 'enableStateStream', enabled });
    return NextResponse.json({ success: true, response });
  } catch (error) {
    console.error('Failed to set state stream:', error);
    return NextResponse.json({ error: 'Failed to set state stream' }, { status: 500 });
  }
}

