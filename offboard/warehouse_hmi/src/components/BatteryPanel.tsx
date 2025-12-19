import { Battery, BatteryCharging, BatteryLow, BatteryWarning } from 'lucide-react';
import { DataPanel } from './DataPanel';
import { DataValue } from './DataValue';
import { Progress } from '@/components/ui/progress';
import { cn } from '@/lib/utils';

interface BatteryData {
  percentage?: number;
  voltage?: number;
  current?: number;
  charging?: boolean;
  temperature?: number;
  state?: string;
}

interface BatteryPanelProps {
  data?: BatteryData;
  connected: boolean;
}

export const BatteryPanel = ({ data, connected }: BatteryPanelProps) => {
  const percentage = data?.percentage ?? 85;
  const voltage = data?.voltage ?? 24.5;
  const current = data?.current ?? 2.3;
  const charging = data?.charging ?? false;
  const temperature = data?.temperature ?? 25;
  const state = data?.state ?? 'normal';

  const getBatteryIcon = () => {
    if (charging) return BatteryCharging;
    if (percentage < 20) return BatteryLow;
    if (percentage < 40) return BatteryWarning;
    return Battery;
  };

  const BatteryIcon = getBatteryIcon();

  const getStatus = () => {
    if (!connected) return 'inactive';
    if (percentage < 20) return 'error';
    if (percentage < 40) return 'warning';
    return 'active';
  };

  return (
    <DataPanel
      title="Battery Status"
      icon={<BatteryIcon className="w-4 h-4" />}
      status={connected ? 'active' : 'inactive'}
    >
      <div className="space-y-2">
        {/* Battery Percentage */}
        <div>
          <div className="flex items-center justify-between mb-1">
            <span className="text-xs font-semibold text-muted-foreground">
              Charge
            </span>
            <span className="text-sm font-bold text-foreground">
              {percentage}%
            </span>
          </div>
          <Progress value={percentage} className="h-2" />
        </div>

        {/* Battery Stats */}
        <div className="grid grid-cols-2 gap-2 pt-1">
          <DataValue label="Voltage" value={voltage} unit="V" size="sm" />
          <DataValue label="Current" value={current} unit="A" size="sm" />
          <DataValue label="Temp" value={temperature} unit="°C" size="sm" />
          <DataValue label="State" value={state} size="sm" />
        </div>
      </div>
    </DataPanel>
  );
};
