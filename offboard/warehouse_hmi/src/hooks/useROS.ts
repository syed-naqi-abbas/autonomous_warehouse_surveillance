import { useState, useEffect, useCallback, useRef } from 'react';
import * as ROSLIB from 'roslib';

interface ROSConfig {
  url: string;
}

interface ROSState {
  connected: boolean;
  connecting: boolean;
  error: string | null;
}

interface TopicData {
  [key: string]: any;
}

export const useROS = (config: ROSConfig) => {
  const [state, setState] = useState<ROSState>({
    connected: false,
    connecting: false,
    error: null,
  });
  const [topicData, setTopicData] = useState<TopicData>({});
  const rosRef = useRef<ROSLIB.Ros | null>(null);
  const subscribersRef = useRef<Map<string, ROSLIB.Topic<any>>>(new Map());

  const connect = useCallback(() => {
    if (rosRef.current) {
      rosRef.current.close();
    }

    setState({ connected: false, connecting: true, error: null });

    const ros = new ROSLIB.Ros({ url: config.url });

    ros.on('connection', () => {
      console.log('Connected to ROS2 bridge');
      setState({ connected: true, connecting: false, error: null });
    });

    ros.on('error', (error) => {
      console.error('ROS connection error:', error);
      setState({ connected: false, connecting: false, error: 'Connection error' });
    });

    ros.on('close', () => {
      console.log('ROS connection closed');
      setState({ connected: false, connecting: false, error: null });
    });

    rosRef.current = ros;
  }, [config.url]);

  const disconnect = useCallback(() => {
    subscribersRef.current.forEach((topic) => topic.unsubscribe());
    subscribersRef.current.clear();
    if (rosRef.current) {
      rosRef.current.close();
      rosRef.current = null;
    }
    setState({ connected: false, connecting: false, error: null });
  }, []);

  const subscribe = useCallback((topicName: string, messageType: string) => {
    if (!rosRef.current || !state.connected) return;

    if (subscribersRef.current.has(topicName)) return;

    const topic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType: messageType,
    });

    topic.subscribe((message) => {
      setTopicData((prev) => ({
        ...prev,
        [topicName]: message,
      }));
    });

    subscribersRef.current.set(topicName, topic);
  }, [state.connected]);

  const unsubscribe = useCallback((topicName: string) => {
    const topic = subscribersRef.current.get(topicName);
    if (topic) {
      topic.unsubscribe();
      subscribersRef.current.delete(topicName);
    }
  }, []);

  useEffect(() => {
    return () => {
      disconnect();
    };
  }, [disconnect]);

  return {
    ...state,
    topicData,
    connect,
    disconnect,
    subscribe,
    unsubscribe,
    ros: rosRef.current,
  };
};
