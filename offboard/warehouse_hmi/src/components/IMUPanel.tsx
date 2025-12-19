import { Activity } from 'lucide-react';
import { DataPanel } from './DataPanel';
import { DataValue } from './DataValue';

interface IMUPanelProps {
  data?: any;
  connected: boolean;
}

export const IMUPanel = ({ data, connected }: IMUPanelProps) => {
  const getAcceleration = () => {
    if (!data || !data.linear_acceleration) {
      return { x: 0, y: 0, z: 0 };
    }
    return {
      x: data.linear_acceleration.x.toFixed(2),
      y: data.linear_acceleration.y.toFixed(2),
      z: data.linear_acceleration.z.toFixed(2),
    };
  };

  const getAngularVelocity = () => {
    if (!data || !data.angular_velocity) {
      return { x: 0, y: 0, z: 0 };
    }
    return {
      x: data.angular_velocity.x.toFixed(2),
      y: data.angular_velocity.y.toFixed(2),
      z: data.angular_velocity.z.toFixed(2),
    };
  };

  const accel = getAcceleration();
  const angVel = getAngularVelocity();

  return (
    <DataPanel
      title="IMU Sensor"
      icon={<Activity className="w-4 h-4" />}
      status={connected ? 'active' : 'inactive'}
    >
      <div className="space-y-3">
        {/* Linear Acceleration */}
        <div>
          <p className="text-xs font-semibold text-muted-foreground mb-1.5 uppercase tracking-wider">
            Acceleration
          </p>
          <div className="grid grid-cols-3 gap-2">
            <DataValue label="X" value={accel.x} unit="m/s²" size="sm" />
            <DataValue label="Y" value={accel.y} unit="m/s²" size="sm" />
            <DataValue label="Z" value={accel.z} unit="m/s²" size="sm" />
          </div>
        </div>

        {/* Angular Velocity */}
        <div>
          <p className="text-xs font-semibold text-muted-foreground mb-1.5 uppercase tracking-wider">
            Ang. Velocity
          </p>
          <div className="grid grid-cols-3 gap-2">
            <DataValue label="R" value={angVel.x} unit="rad/s" size="sm" />
            <DataValue label="P" value={angVel.y} unit="rad/s" size="sm" />
            <DataValue label="Y" value={angVel.z} unit="rad/s" size="sm" />
          </div>
        </div>
      </div>
    </DataPanel>
  );
};