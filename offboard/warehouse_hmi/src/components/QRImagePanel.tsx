import { QrCode, Clock, Trash2 } from 'lucide-react';
import { DataPanel } from './DataPanel';
import { cn } from '@/lib/utils';
import { useEffect, useState, useRef } from 'react';
import * as ROSLIB from 'roslib';

interface QRImageData {
  id: string;
  timestamp: string;
  qrString: string;
  rackId: string;
  shelfId: string;
  itemCode: string;
}

interface QRImagePanelProps {
  ros: ROSLIB.Ros | null;
  data?: QRImageData[];
  connected: boolean;
}

export const QRImagePanel = ({ ros, data, connected }: QRImagePanelProps) => {
  const [qrImages, setQrImages] = useState<QRImageData[]>([]);
  const qrStringsRef = useRef<Set<string>>(new Set());

  useEffect(() => {
    if (!ros || !connected) return;

    const qrTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/qr_data',
      messageType: 'std_msgs/String',
    });

    qrTopic.subscribe((message: any) => {
      const qrString = message.data;

      // Check if QR string already exists
      if (qrStringsRef.current.has(qrString)) {
        return;
      }

      qrStringsRef.current.add(qrString);

      const parts = qrString.split('_');

      let rackId = 'UNKNOWN';
      let shelfId = 'UNKNOWN';
      let itemCode = 'UNKNOWN';

      if (parts.length >= 3) {
        rackId = `RACK-${parts[0].replace('R', '')}`;
        shelfId = `SHELF-${parts[1].replace('S', '')}`;
        itemCode = `ITEM-${parts[2].replace('ITM', '')}`;
      }

      const newQR: QRImageData = {
        id: `${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        timestamp: new Date().toLocaleString(),
        qrString: qrString,
        rackId: rackId,
        shelfId: shelfId,
        itemCode: itemCode,
      };

      setQrImages((prev) => [newQR, ...prev]);
    });

    return () => {
      qrTopic.unsubscribe();
      qrStringsRef.current.clear();
    };
  }, [ros, connected]);

  const handleDelete = (id: string) => {
    setQrImages((prev) => {
      const deletedQr = prev.find((qr) => qr.id === id);
      if (deletedQr) {
        qrStringsRef.current.delete(deletedQr.qrString);
      }
      return prev.filter((qr) => qr.id !== id);
    });
  };

  return (
    <DataPanel
      title="QR Image Captures"
      icon={<QrCode className="w-4 h-4" />}
      status={connected ? 'active' : 'inactive'}
    >
      <div className="space-y-3 overflow-y-auto scrollbar-techy h-[500px]">
        {qrImages.length === 0 ? (
          <div className="text-center py-8 text-muted-foreground text-sm">
            <QrCode className="w-8 h-8 mx-auto mb-2 opacity-50" />
            <p>No QR captures yet</p>
            <p className="text-xs mt-1">QR codes will appear here when captured</p>
          </div>
        ) : (
          qrImages.map((qr) => (
            <div
              key={qr.id}
              className="border border-border/30 rounded bg-muted/30 p-3 hover:bg-muted/50 transition-colors relative group"
            >
              {/* Header with timestamp */}
              <div className="flex items-center justify-between gap-2 mb-3 pb-2 border-b border-border/30">
                <div className="flex items-center gap-2">
                  <Clock className="w-3 h-3 text-muted-foreground" />
                  <span className="text-xs font-mono text-muted-foreground">
                    {qr.timestamp}
                  </span>
                </div>
                <button
                  onClick={() => handleDelete(qr.id)}
                  className="opacity-0 group-hover:opacity-100 transition-opacity p-1 hover:bg-destructive/20 rounded"
                  title="Delete"
                >
                  <Trash2 className="w-3 h-3 text-destructive" />
                </button>
              </div>

              {/* QR Data Grid */}
              <div className="grid grid-cols-2 gap-3">
                <div className="space-y-1">
                  <span className="text-xs uppercase tracking-wider text-muted-foreground">
                    QR String
                  </span>
                  <div className="text-sm font-mono text-foreground break-all">
                    {qr.qrString}
                  </div>
                </div>
                <div className="space-y-1">
                  <span className="text-xs uppercase tracking-wider text-muted-foreground">
                    Rack ID
                  </span>
                  <div className="text-sm font-mono font-semibold text-primary">
                    {qr.rackId}
                  </div>
                </div>
                <div className="space-y-1">
                  <span className="text-xs uppercase tracking-wider text-muted-foreground">
                    Shelf ID
                  </span>
                  <div className="text-sm font-mono font-semibold text-foreground">
                    {qr.shelfId}
                  </div>
                </div>
                <div className="space-y-1">
                  <span className="text-xs uppercase tracking-wider text-muted-foreground">
                    Item Code
                  </span>
                  <div className="text-sm font-mono font-semibold text-foreground">
                    {qr.itemCode}
                  </div>
                </div>
              </div>
            </div>
          ))
        )}
      </div>
    </DataPanel>
  );
};