import { ReactNode } from 'react';
import { cn } from '@/lib/utils';

interface DataPanelProps {
  title: string;
  icon?: ReactNode;
  children: ReactNode;
  className?: string;
  status?: 'active' | 'warning' | 'error' | 'inactive';
}

export const DataPanel = ({
  title,
  icon,
  children,
  className,
  status = 'inactive',
}: DataPanelProps) => {
  return (
    <div
      className={cn(
        'glass rounded-lg overflow-hidden transition-all duration-300',
        status === 'active' && 'border-primary/50 glow-primary',
        status === 'warning' && 'border-warning/50 glow-warning',
        status === 'error' && 'border-destructive/50',
        status === 'inactive' && 'border-border/30',
        className
      )}
    >
      {/* Header */}
      <div className="flex items-center gap-2 px-4 py-3 border-b border-border/30 bg-muted/30">
        <div
          className={cn(
            'w-2 h-2 rounded-full',
            status === 'active' && 'bg-success animate-pulse',
            status === 'warning' && 'bg-warning animate-pulse',
            status === 'error' && 'bg-destructive',
            status === 'inactive' && 'bg-muted-foreground'
          )}
        />
        {icon && <span className="text-primary">{icon}</span>}
        <h3 className="font-display text-sm uppercase tracking-wider text-foreground">
          {title}
        </h3>
      </div>

      {/* Content */}
      <div className="p-4">{children}</div>
    </div>
  );
};
