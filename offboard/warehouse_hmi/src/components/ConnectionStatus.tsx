import { Wifi, WifiOff, Loader2 } from 'lucide-react';
import { cn } from '@/lib/utils';

interface ConnectionStatusProps {
  connected: boolean;
  connecting: boolean;
  onConnect: () => void;
  onDisconnect: () => void;
}

export const ConnectionStatus = ({
  connected,
  connecting,
  onConnect,
  onDisconnect,
}: ConnectionStatusProps) => {
  return (
    <div className="flex items-center gap-4">
      <div className="flex items-center gap-2">
        <div
          className={cn(
            'w-3 h-3 rounded-full transition-all duration-300',
            connected && 'bg-success animate-pulse-glow',
            connecting && 'bg-warning animate-pulse',
            !connected && !connecting && 'bg-destructive'
          )}
        />
        <span className="text-sm text-muted-foreground uppercase tracking-wider">
          {connected ? 'Online' : connecting ? 'Connecting' : 'Offline'}
        </span>
      </div>
      
      <button
        onClick={connected ? onDisconnect : onConnect}
        disabled={connecting}
        className={cn(
          'flex items-center gap-2 px-4 py-2 rounded-md transition-all duration-300',
          'border border-primary/30 hover:border-primary',
          'bg-card hover:bg-primary/10',
          'text-primary text-sm font-mono uppercase tracking-wider',
          'disabled:opacity-50 disabled:cursor-not-allowed',
          connected && 'glow-primary'
        )}
      >
        {connecting ? (
          <Loader2 className="w-4 h-4 animate-spin" />
        ) : connected ? (
          <Wifi className="w-4 h-4" />
        ) : (
          <WifiOff className="w-4 h-4" />
        )}
        {connected ? 'Disconnect' : 'Connect'}
      </button>
    </div>
  );
};
