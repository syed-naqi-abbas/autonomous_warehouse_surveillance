import { Radar } from 'lucide-react';
import { DataPanel } from './DataPanel';
import { DataValue } from './DataValue';

interface SensorPanelProps {
  data: Record<string, any>;
  connected: boolean;
}

export const SensorPanel = ({ data, connected }: SensorPanelProps) => {
  // Extract odometry data
  const odom = data['/odom'];
  const posX = odom?.pose?.pose?.position?.x?.toFixed(2) || '0.00';
  const posY = odom?.pose?.pose?.position?.y?.toFixed(2) || '0.00';
  const posZ = odom?.pose?.pose?.position?.z?.toFixed(2) || '0.00';
  
  // Calculate orientation (yaw from quaternion)
  const qz = odom?.pose?.pose?.orientation?.z || 0;
  const qw = odom?.pose?.pose?.orientation?.w || 1;
  const yaw = (Math.atan2(2 * qw * qz, 1 - 2 * qz * qz) * 180 / Math.PI).toFixed(1);

  // Extract laser scan data
  const scan = data['/scan'];
  const rangeMin = scan?.range_min?.toFixed(2) || 'N/A';
  const rangeMax = scan?.range_max?.toFixed(2) || 'N/A';
  const numRanges = scan?.ranges?.length || 0;

  return (
    <DataPanel
      title="Sensor Data"
      icon={<Radar className="w-5 h-5" />}
      status={connected ? 'active' : 'inactive'}
    >
      <div className="space-y-4">
        {/* Odometry */}
        <div>
          <h4 className="text-xs font-semibold text-muted-foreground uppercase tracking-wider mb-2">
            Odometry
          </h4>
          <div className="grid grid-cols-2 gap-3">
            <DataValue label="Pos X" value={posX} unit="m" size="sm" />
            <DataValue label="Pos Y" value={posY} unit="m" size="sm" />
            <DataValue label="Pos Z" value={posZ} unit="m" size="sm" />
            <DataValue label="Yaw" value={yaw} unit="°" size="sm" />
          </div>
        </div>

        {/* Laser Scan */}
        <div>
          <h4 className="text-xs font-semibold text-muted-foreground uppercase tracking-wider mb-2">
            Laser Scan
          </h4>
          <div className="grid grid-cols-2 gap-3">
            <DataValue label="Range Min" value={rangeMin} unit="m" size="sm" />
            <DataValue label="Range Max" value={rangeMax} unit="m" size="sm" />
            <DataValue label="Points" value={numRanges.toString()} unit="" size="sm" />
          </div>
        </div>
      </div>
    </DataPanel>
  );
};
