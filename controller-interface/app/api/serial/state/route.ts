import { NextResponse } from 'next/server';
import SerialManager from '@/lib/server/SerialManager';

export async function GET() {
  try {
    const serialManager = SerialManager.getInstance();
    const latestState = serialManager.getLatestState?.() ?? null;
    return NextResponse.json({ latestState });
  } catch (error) {
    console.error('Failed to get latest state:', error);
    return NextResponse.json({ error: 'Failed to get latest state' }, { status: 500 });
  }
}

