import { useState } from 'react';
import { Terminal, ChevronDown, ChevronRight, Radio } from 'lucide-react';
import { DataPanel } from './DataPanel';
import { cn } from '@/lib/utils';
import { Badge } from './ui/badge';

interface TopicData {
  [key: string]: any;
}

interface TopicMonitorProps {
  topics: TopicData;
  connected: boolean;
}

export const TopicMonitor = ({ topics, connected }: TopicMonitorProps) => {
  const [expanded, setExpanded] = useState<string[]>([]);

  const toggleExpand = (topic: string) => {
    setExpanded((prev) =>
      prev.includes(topic)
        ? prev.filter((t) => t !== topic)
        : [...prev, topic]
    );
  };

  const topicList = Object.keys(topics);

  return (
    <DataPanel
      title="Topic Monitor"
      icon={<Terminal className="w-4 h-4" />}
      status={connected && topicList.length > 0 ? 'active' : 'inactive'}
    >
      <div className="max-h-[300px] overflow-y-auto space-y-2 scrollbar-techy">
        {topicList.length === 0 ? (
          <div className="text-center py-8 text-muted-foreground text-sm">
            <Terminal className="w-8 h-8 mx-auto mb-2 opacity-50" />
            <p>No topics subscribed</p>
            <p className="text-xs mt-1">Connect and subscribe to ROS2 topics</p>
          </div>
        ) : (
          topicList.map((topic) => (
            <div
              key={topic}
              className="border border-border/30 rounded bg-muted/30 overflow-hidden"
            >
              <button
                onClick={() => toggleExpand(topic)}
                className="w-full flex items-center gap-2 px-3 py-2 text-left hover:bg-muted/50 transition-colors"
              >
                {expanded.includes(topic) ? (
                  <ChevronDown className="w-4 h-4 text-primary" />
                ) : (
                  <ChevronRight className="w-4 h-4 text-muted-foreground" />
                )}
                <span className="text-sm font-mono text-primary truncate flex-1">
                  {topic}
                </span>
                <div className="w-2 h-2 rounded-full bg-success animate-pulse" />
              </button>
              
              {expanded.includes(topic) && (
                <div className="px-3 py-2 border-t border-border/30 bg-background/50">
                  <pre className="text-xs font-mono text-muted-foreground overflow-x-auto whitespace-pre-wrap">
                    {JSON.stringify(topics[topic], null, 2)}
                  </pre>
                </div>
              )}
            </div>
          ))
        )}
      </div>
    </DataPanel>
  );
};
