import { NextRequest, NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';

export async function POST(request: NextRequest) {
  try {
    const { enabled } = await request.json();
    const serialManager = SerialManager.getInstance();
    const { response } = await serialManager.sendCommandAndWait({ command: 'enableStateStream', enabled });
    return NextResponse.json({ success: true, response });
  } catch (error) {
    console.error('Failed to set state stream:', error);
    return NextResponse.json({ error: 'Failed to set state stream' }, { status: 500 });
  }
}
import { NextRequest, NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';

export async function POST(request: NextRequest) {
  try {
    const { ms } = await request.json();
    const serialManager = SerialManager.getInstance();
    const { response } = await serialManager.sendCommandAndWait({ command: 'setStateInterval', ms });
    return NextResponse.json({ success: true, response });
  } catch (error) {
    console.error('Failed to set state interval:', error);
    return NextResponse.json({ error: 'Failed to set state interval' }, { status: 500 });
  }
}

