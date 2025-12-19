import { Activity } from 'lucide-react';
import { DataPanel } from './DataPanel';
import { DataValue } from './DataValue';

interface VelocityPanelProps {
  data?: any;
  connected: boolean;
}

export const VelocityPanel = ({ data, connected }: VelocityPanelProps) => {
  // Extract velocity data
  const linearX = data?.linear?.x?.toFixed(3) || '0.000';
  const linearY = data?.linear?.y?.toFixed(3) || '0.000';
  const linearZ = data?.linear?.z?.toFixed(3) || '0.000';
  const angularZ = data?.angular?.z?.toFixed(3) || '0.000';

  return (
    <DataPanel
      title="Velocity Command"
      icon={<Activity className="w-5 h-5" />}
      status={connected ? 'active' : 'inactive'}
    >
      <div className="grid grid-cols-2 gap-4">
        <DataValue
          label="Linear X"
          value={linearX}
          unit="m/s"
          size="sm"
        />
        <DataValue
          label="Linear Y"
          value={linearY}
          unit="m/s"
          size="sm"
        />
        <DataValue
          label="Linear Z"
          value={linearZ}
          unit="m/s"
          size="sm"
        />
        <DataValue
          label="Angular Z"
          value={angularZ}
          unit="rad/s"
          size="sm"
        />
      </div>
    </DataPanel>
  );
};
