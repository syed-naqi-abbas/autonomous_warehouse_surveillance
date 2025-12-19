import { cn } from '@/lib/utils';

interface DataValueProps {
  label: string;
  value: string | number;
  unit?: string;
  size?: 'sm' | 'md' | 'lg';
  highlight?: boolean;
}

export const DataValue = ({
  label,
  value,
  unit,
  size = 'md',
  highlight = false,
}: DataValueProps) => {
  return (
    <div className="flex flex-col">
      <span className="text-xs uppercase tracking-wider text-muted-foreground mb-1">
        {label}
      </span>
      <div className="flex items-baseline gap-1">
        <span
          className={cn(
            'font-mono font-semibold transition-all duration-300',
            size === 'sm' && 'text-lg',
            size === 'md' && 'text-2xl',
            size === 'lg' && 'text-4xl',
            highlight ? 'text-primary text-glow' : 'text-foreground'
          )}
        >
          {value}
        </span>
        {unit && (
          <span className="text-sm text-muted-foreground">{unit}</span>
        )}
      </div>
    </div>
  );
};
