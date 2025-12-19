import { useState } from 'react';
import { Settings, Server } from 'lucide-react';
import { cn } from '@/lib/utils';

interface ConnectionConfigProps {
  url: string;
  onUrlChange: (url: string) => void;
  connected: boolean;
}

export const ConnectionConfig = ({ url, onUrlChange, connected }: ConnectionConfigProps) => {
  const [isEditing, setIsEditing] = useState(false);
  const [tempUrl, setTempUrl] = useState(url);

  const handleSave = () => {
    onUrlChange(tempUrl);
    setIsEditing(false);
  };

  return (
    <div className="flex items-center gap-3 px-4 py-3 bg-card/50 border border-border/30 rounded-lg">
      <Server className="w-4 h-4 text-primary" />
      
      {isEditing ? (
        <div className="flex-1 flex items-center gap-2">
          <input
            type="text"
            value={tempUrl}
            onChange={(e) => setTempUrl(e.target.value)}
            className="flex-1 bg-muted/50 border border-border/50 rounded px-3 py-1.5 text-sm font-mono text-foreground focus:outline-none focus:border-primary"
            placeholder="ws://localhost:9090"
          />
          <button
            onClick={handleSave}
            className="px-3 py-1.5 bg-primary text-primary-foreground rounded text-sm font-mono hover:bg-primary/90 transition-colors"
          >
            Save
          </button>
          <button
            onClick={() => {
              setTempUrl(url);
              setIsEditing(false);
            }}
            className="px-3 py-1.5 bg-muted text-foreground rounded text-sm font-mono hover:bg-muted/80 transition-colors"
          >
            Cancel
          </button>
        </div>
      ) : (
        <>
          <div className="flex-1">
            <span className="text-xs text-muted-foreground uppercase tracking-wider">WebSocket URL</span>
            <p className="text-sm font-mono text-foreground truncate">{url}</p>
          </div>
          <button
            onClick={() => setIsEditing(true)}
            disabled={connected}
            className={cn(
              'p-2 rounded hover:bg-muted transition-colors',
              connected && 'opacity-50 cursor-not-allowed'
            )}
          >
            <Settings className="w-4 h-4 text-muted-foreground" />
          </button>
        </>
      )}
    </div>
  );
};
