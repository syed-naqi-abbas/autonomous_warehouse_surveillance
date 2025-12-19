import { useState, useEffect } from 'react';
import { useROS } from '@/hooks/useROS';
import { Header } from '@/components/Header';
import { MapViewer } from '@/components/MapViewer';
import { VelocityPanel } from '@/components/VelocityPanel';
import { BatteryPanel } from '@/components/BatteryPanel';
import { SensorPanel } from '@/components/SensorPanel';
import { IMUPanel } from '@/components/IMUPanel';
import { TopicMonitor } from '@/components/TopicMonitor';
import { QRImagePanel } from '@/components/QRImagePanel';
import { ConnectionConfig } from '@/components/ConnectionConfig';
import { DataPanel } from '@/components/DataPanel';
import { DataValue } from '@/components/DataValue';
import { Map, Cpu, Clock, Thermometer } from 'lucide-react';

const Index = () => {
  const [wsUrl, setWsUrl] = useState('ws://localhost:9090');
  const {
    connected,
    connecting,
    topicData,
    connect,
    disconnect,
    subscribe,
    ros,
  } = useROS({ url: wsUrl });

  // Subscribe to common ROS2 topics when connected
  useEffect(() => {
    if (connected) {
      subscribe('/cmd_vel', 'geometry_msgs/Twist');
      subscribe('/odom', 'nav_msgs/Odometry');
      subscribe('/battery_state', 'sensor_msgs/BatteryState');
      subscribe('/scan', 'sensor_msgs/LaserScan');
      subscribe('/imu/data', 'sensor_msgs/Imu');
      subscribe('/image_raw', 'sensor_msgs/Image');
    }
  }, [connected, subscribe]);

  // Simulated uptime
  const [uptime, setUptime] = useState(0);
  useEffect(() => {
    const interval = setInterval(() => {
      setUptime((prev) => prev + 1);
    }, 1000);
    return () => clearInterval(interval);
  }, []);

  const formatUptime = (seconds: number) => {
    const hrs = Math.floor(seconds / 3600);
    const mins = Math.floor((seconds % 3600) / 60);
    const secs = seconds % 60;
    return `${hrs.toString().padStart(2, '0')}:${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  return (
    <div className="min-h-screen bg-background flex flex-col">
      <Header
        connected={connected}
        connecting={connecting}
        onConnect={connect}
        onDisconnect={disconnect}
      />

      <main className="flex-1 w-full max-w-[98vw] mx-auto px-8 py-5">
        {/* Connection Config */}
        <div className="mb-5">
          <ConnectionConfig
            url={wsUrl}
            onUrlChange={setWsUrl}
            connected={connected}
          />
        </div>

        {/* Three Column Layout */}
        <div className="grid grid-cols-12 gap-6 h-[calc(100vh-240px)]">
          {/* LEFT COLUMN - Map, Sensor (4 cols) */}
          <div className="col-span-4 space-y-5 overflow-y-auto scrollbar-techy">
            {/* Map - Increased Height */}
            <div className="h-[600px]">
              <DataPanel
                title="RViz2 Map"
                icon={<Map className="w-5 h-5" />}
                status={connected ? 'active' : 'inactive'}
                className="h-full"
              >
                <div className="h-full">
                  <MapViewer
                    ros={ros}
                    connected={connected}
                    mapTopic="/map"
                    poseTopic="/robot_pose"
                  />
                </div>
              </DataPanel>
            </div>

            {/* Sensor Panel */}
            <SensorPanel data={topicData} connected={connected} />
          </div>

          {/* MIDDLE COLUMN - System Status, Battery, IMU, Velocity (4 cols) */}
          <div className="col-span-4 space-y-5 overflow-y-auto scrollbar-techy">
            {/* System Stats */}
            <DataPanel
              title="System Status"
              icon={<Cpu className="w-5 h-5" />}
              status={connected ? 'active' : 'inactive'}
            >
              <div className="grid grid-cols-2 gap-5">
                <DataValue
                  label="Uptime"
                  value={formatUptime(uptime)}
                  size="md"
                />
                <DataValue
                  label="CPU Temp"
                  value="42"
                  unit="°C"
                  size="md"
                />
                <DataValue
                  label="Memory"
                  value="68"
                  unit="%"
                  size="md"
                />
                <DataValue
                  label="Disk"
                  value="45"
                  unit="%"
                  size="md"
                />
              </div>
            </DataPanel>

            {/* Battery Panel */}
            <BatteryPanel data={topicData['/battery_state']} connected={connected} />

            {/* IMU Panel */}
            <IMUPanel data={topicData['/imu/data']} connected={connected} />

            {/* Velocity Panel */}
            <VelocityPanel data={topicData['/cmd_vel']} connected={connected} />
          </div>

          {/* RIGHT COLUMN - QR Image Logging (4 cols) */}
          <div className="col-span-4 space-y-5">
            {/* QR Image Panel - Fixed height with dedicated scrollbar */}
            <div className="h-[600px] overflow-y-auto scrollbar-techy"> 
              <QRImagePanel ros={ros} data={undefined} connected={connected} />
            </div>

            {/* Topic Monitor - Below QR */}
            <TopicMonitor topics={topicData} connected={connected} />
          </div>
        </div>
      </main>

      {/* Footer */}
      <footer className="py-3 px-8 border-t border-border/30 bg-background">
        <div className="flex items-center justify-between text-sm text-muted-foreground">
          <div className="flex items-center gap-6">
            <span className="flex items-center gap-2">
              <Clock className="w-4 h-4" />
              Last update: {new Date().toLocaleTimeString()}
            </span>
            <span className="flex items-center gap-2">
              <Thermometer className="w-4 h-4" />
              Environment: Normal
            </span>
          </div>
          <span className="font-mono">ROS2 Jazzy • rosbridge_suite v1.3</span>
        </div>
      </footer>
    </div>
  );
};

export default Index;
