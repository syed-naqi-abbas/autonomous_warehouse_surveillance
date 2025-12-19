import { Bot, Activity } from 'lucide-react';
import { ConnectionStatus } from './ConnectionStatus';

interface HeaderProps {
  connected: boolean;
  connecting: boolean;
  onConnect: () => void;
  onDisconnect: () => void;
}

export const Header = ({ connected, connecting, onConnect, onDisconnect }: HeaderProps) => {
  return (
    <header className="w-full border-b border-border/30 bg-card/50 backdrop-blur-sm">
      <div className="container mx-auto px-4 py-4">
        <div className="flex items-center justify-between">
          {/* Logo and Title */}
          <div className="flex items-center gap-4">
            <div className="relative">
              <div className="w-12 h-12 rounded-lg bg-primary/20 border border-primary/50 flex items-center justify-center glow-primary">
                <Bot className="w-7 h-7 text-primary" />
              </div>
              <div className="absolute -top-1 -right-1 w-3 h-3 rounded-full bg-success animate-pulse-glow" />
            </div>
            <div>
              <h1 className="font-display text-xl tracking-wider text-foreground">
                AUTONOMOUS <span className="text-primary text-glow">ROBOT</span>
              </h1>
              <p className="text-xs text-muted-foreground uppercase tracking-widest">
                Real-time Monitoring System
              </p>
            </div>
          </div>

          {/* Status and Connection */}
          <div className="flex items-center gap-6">
            <div className="hidden md:flex items-center gap-2 text-sm text-muted-foreground">
              <Activity className="w-4 h-4 text-primary animate-pulse" />
              <span className="font-mono">ROS2 Bridge</span>
            </div>
            <ConnectionStatus
              connected={connected}
              connecting={connecting}
              onConnect={onConnect}
              onDisconnect={onDisconnect}
            />
          </div>
        </div>
      </div>
    </header>
  );
};
