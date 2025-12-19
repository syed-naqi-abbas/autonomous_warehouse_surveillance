import { useEffect, useRef, useState } from 'react';
import { cn } from '@/lib/utils';
import * as ROSLIB from 'roslib';

interface MapViewerProps {
  ros: ROSLIB.Ros | null;
  connected: boolean;
  mapTopic?: string;
  poseTopic?: string;
}

interface MapData {
  width: number;
  height: number;
  resolution: number;
  data: number[];
  origin: { x: number; y: number };
}

interface PoseData {
  x: number;
  y: number;
  theta: number;
}

export const MapViewer = ({
  ros,
  connected,
  mapTopic = '/map',
  poseTopic = '/robot_pose',
}: MapViewerProps) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [poseData, setPoseData] = useState<PoseData>({ x: 0, y: 0, theta: 0 });
  const [scale, setScale] = useState(1);

  // Subscribe to map and pose topics
  useEffect(() => {
    if (!ros || !connected) return;

    const mapListener = new ROSLIB.Topic({
      ros: ros,
      name: mapTopic,
      messageType: 'nav_msgs/OccupancyGrid',
    });

    const poseListener = new ROSLIB.Topic({
      ros: ros,
      name: poseTopic,
      messageType: 'geometry_msgs/PoseStamped',
    });

    mapListener.subscribe((message: any) => {
      setMapData({
        width: message.info.width,
        height: message.info.height,
        resolution: message.info.resolution,
        data: message.data,
        origin: {
          x: message.info.origin.position.x,
          y: message.info.origin.position.y,
        },
      });
    });

    poseListener.subscribe((message: any) => {
      const q = message.pose.orientation;
      const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      setPoseData({
        x: message.pose.position.x,
        y: message.pose.position.y,
        theta: theta,
      });
    });

    return () => {
      mapListener.unsubscribe();
      poseListener.unsubscribe();
    };
  }, [ros, connected, mapTopic, poseTopic]);

  // Render map on canvas
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const width = canvas.width;
    const height = canvas.height;

    // Clear canvas with dark background
    ctx.fillStyle = '#0a0e14';
    ctx.fillRect(0, 0, width, height);

    // Draw grid
    ctx.strokeStyle = 'rgba(0, 255, 255, 0.1)';
    ctx.lineWidth = 0.5;
    const gridSize = 20;
    
    for (let x = 0; x < width; x += gridSize) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, height);
      ctx.stroke();
    }
    
    for (let y = 0; y < height; y += gridSize) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }

    if (mapData) {
      // Draw occupancy grid with high contrast
      const cellWidth = (width / mapData.width) * scale;
      const cellHeight = (height / mapData.height) * scale;

      for (let y = 0; y < mapData.height; y++) {
        for (let x = 0; x < mapData.width; x++) {
          const value = mapData.data[y * mapData.width + x];
          
          if (value === -1) {
            // Unknown/Unmapped - Light gray with subtle glow
            ctx.fillStyle = 'rgba(100, 110, 130, 0.6)';
            ctx.shadowColor = 'rgba(100, 110, 130, 0.3)';
            ctx.shadowBlur = 2;
          } else if (value === 0) {
            // Free space - Bright cyan/white
            ctx.fillStyle = 'rgba(200, 255, 255, 0.15)';
            ctx.shadowColor = 'rgba(0, 255, 255, 0.1)';
            ctx.shadowBlur = 1;
          } else {
            // Occupied - Strong red/orange gradient
            const intensity = value / 100;
            const hue = 20 * (1 - intensity); // Red to orange
            ctx.fillStyle = `rgba(${255 - intensity * 100}, ${100 - intensity * 50}, ${50}, ${0.7 + intensity * 0.3})`;
            ctx.shadowColor = `rgba(255, 100, 100, ${intensity * 0.4})`;
            ctx.shadowBlur = 2;
          }
          
          ctx.fillRect(x * cellWidth, y * cellHeight, cellWidth, cellHeight);
        }
      }
      
      ctx.shadowColor = 'transparent';

      // Draw robot position
      const robotX = ((poseData.x - mapData.origin.x) / mapData.resolution) * cellWidth;
      const robotY = height - ((poseData.y - mapData.origin.y) / mapData.resolution) * cellHeight;

      // Robot glow effect
      const gradient = ctx.createRadialGradient(robotX, robotY, 0, robotX, robotY, 25);
      gradient.addColorStop(0, 'rgba(0, 255, 255, 1)');
      gradient.addColorStop(0.5, 'rgba(0, 255, 255, 0.4)');
      gradient.addColorStop(1, 'rgba(0, 255, 255, 0)');
      ctx.fillStyle = gradient;
      ctx.beginPath();
      ctx.arc(robotX, robotY, 25, 0, Math.PI * 2);
      ctx.fill();

      // Robot body
      ctx.fillStyle = '#00ffff';
      ctx.beginPath();
      ctx.arc(robotX, robotY, 8, 0, Math.PI * 2);
      ctx.fill();

      // Robot direction arrow with glow
      ctx.save();
      ctx.translate(robotX, robotY);
      ctx.rotate(-poseData.theta);
      
      ctx.shadowColor = 'rgba(0, 255, 255, 0.6)';
      ctx.shadowBlur = 4;
      
      ctx.fillStyle = '#00ffff';
      ctx.beginPath();
      ctx.moveTo(15, 0);
      ctx.lineTo(-8, -10);
      ctx.lineTo(-8, 10);
      ctx.closePath();
      ctx.fill();
      
      ctx.restore();
    } else {
      // Demo animation when no map data
      const time = Date.now() / 1000;
      const centerX = width / 2;
      const centerY = height / 2;

      // Scanning effect
      ctx.strokeStyle = 'rgba(0, 255, 255, 0.3)';
      ctx.lineWidth = 2;
      for (let i = 0; i < 5; i++) {
        const radius = 30 + i * 40 + ((time * 50) % 40);
        ctx.beginPath();
        ctx.arc(centerX, centerY, radius, 0, Math.PI * 2);
        ctx.stroke();
      }

      // Center point with glow
      const centerGradient = ctx.createRadialGradient(centerX, centerY, 0, centerX, centerY, 15);
      centerGradient.addColorStop(0, 'rgba(0, 255, 255, 1)');
      centerGradient.addColorStop(1, 'rgba(0, 255, 255, 0)');
      ctx.fillStyle = centerGradient;
      ctx.beginPath();
      ctx.arc(centerX, centerY, 15, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = '#00ffff';
      ctx.beginPath();
      ctx.arc(centerX, centerY, 8, 0, Math.PI * 2);
      ctx.fill();

      // Rotating line
      ctx.save();
      ctx.translate(centerX, centerY);
      ctx.rotate(time);
      ctx.strokeStyle = 'rgba(0, 255, 255, 0.6)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(150, 0);
      ctx.stroke();
      ctx.restore();

      // "Waiting for map" text
      ctx.fillStyle = 'rgba(0, 255, 255, 0.7)';
      ctx.font = 'bold 14px "JetBrains Mono"';
      ctx.textAlign = 'center';
      ctx.fillText('AWAITING MAP DATA', centerX, height - 30);
    }
  }, [mapData, poseData, scale]);

  // Animation loop for demo
  useEffect(() => {
    if (mapData) return;

    let animationId: number;
    const animate = () => {
      const canvas = canvasRef.current;
      if (canvas) {
        const ctx = canvas.getContext('2d');
        if (ctx) {
          setScale((s) => s);
        }
      }
      animationId = requestAnimationFrame(animate);
    };
    animate();

    return () => cancelAnimationFrame(animationId);
  }, [mapData]);

  return (
    <div className="relative w-full h-full min-h-[400px] rounded-lg overflow-hidden">
      {/* Scan line overlay */}
      <div className="absolute inset-0 pointer-events-none overflow-hidden">
        <div className="w-full h-1 bg-gradient-to-b from-transparent via-primary/30 to-transparent animate-scan" />
      </div>
      
      {/* Corner decorations */}
      <div className="absolute top-0 left-0 w-8 h-8 border-l-2 border-t-2 border-primary/50" />
      <div className="absolute top-0 right-0 w-8 h-8 border-r-2 border-t-2 border-primary/50" />
      <div className="absolute bottom-0 left-0 w-8 h-8 border-l-2 border-b-2 border-primary/50" />
      <div className="absolute bottom-0 right-0 w-8 h-8 border-r-2 border-b-2 border-primary/50" />

      <canvas
        ref={canvasRef}
        width={600}
        height={400}
        className={cn(
          'w-full h-full object-contain',
          connected && 'glow-primary'
        )}
        style={{ filter: 'contrast(1.3) brightness(1.05)' }}
      />

      {/* Status badge */}
      <div className="absolute top-4 left-4 flex items-center gap-2 px-3 py-1.5 rounded-full bg-card/80 border border-border/50">
        <div className={cn(
          'w-2 h-2 rounded-full',
          connected ? 'bg-success animate-pulse' : 'bg-muted-foreground'
        )} />
        <span className="text-xs font-mono uppercase tracking-wider text-muted-foreground">
          {connected ? 'Live' : 'Demo'}
        </span>
      </div>

    </div>
  );
};
