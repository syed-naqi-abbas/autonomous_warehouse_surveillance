# Warehouse HMI — Offboard Interface

Overview
- This web application is the offboard Human‑Machine Interface (HMI) for the autonomous warehouse robot.
- It displays live ROS2 topics (map, robot pose, sensor feeds) and an RViz-like map view delivered via rosbridge.
- QR codes scanned through the website are saved by the HMI with details (decoded value, timestamp, and optional metadata). By default scans are persisted in the browser; you can configure a backend endpoint to persist them server‑side.

Features
- Live map and robot pose visualization (map published by SLAM /slam_toolbox).
- Topic viewer for common robot topics (/map, /tf, /odom, /scan, /detected_shelf_info, /navigate_to_pose, /cmd_vel).
- QR code scanner with scan history and detail view (saved locally or to configured backend).
- Simple teleoperation and goal sending via Nav2-compatible action client from the UI.


Quick start (development)
1. Install dependencies:
   npm install
2. Start the dev server:
   npm run dev
3. Open the URL shown by the dev server (usually http://localhost:5173).

Build for production
- Build the static assets:
  npm run build
- Serve the contents of the `dist` folder using your preferred static file server.

Configuring ROS connection
- The HMI connects to ROS through a rosbridge websocket. Configure the websocket URL in:
  offboard/warehouse_hmi/src/hooks/useROS.ts
- You can also use an environment variable REACT_APP_ROSBRIDGE_WS_URL to set the websocket URL at runtime (if the project uses env support). Default is:
  ws://localhost:9090

Where QR scans are stored
- Default: scans are saved in the browser (localStorage) and are visible in the UI scan history.
- Optional backend: set a backend endpoint (e.g. via REACT_APP_QR_SAVE_ENDPOINT) to persist scans to a server or database. The saved record includes decoded data, timestamp, and any attached metadata.

ROS topics and actions commonly used by this HMI
- Topics: /map, /tf, /odom, /scan, /detected_shelf_info, /frontier_goals, /cmd_vel
- Actions: /navigate_to_pose (Nav2 NavigateToPose action)
- The HMI subscribes/publishes through rosbridge; make sure rosbridge is running on the robot or accessible from this host.

Troubleshooting
- HMI cannot connect: confirm rosbridge websocket is reachable from your browser and CORS/network rules allow the connection.
- No map shown: verify SLAM is running and publishing /map and TF.
- QR scans not persisting: check browser localStorage or backend endpoint configuration and network logs in browser devtools.

Editing & contribution
- Edit code locally with your preferred IDE, commit and push. The project is built with Vite + React + TypeScript + Tailwind.
- Dev commands: npm install, npm run dev, npm run build.

Notes
- This HMI is intended to run on the offboard laptop and connect to the onboard rosbridge/websocket or local ROS instance.
- For exact integration points, see:
  - offboard/warehouse_hmi/src/hooks/useROS.ts (ROS connection)
  - offboard/warehouse_hmi/src/components/MapViewer.tsx (map rendering)
  - offboard/warehouse_hmi/src/pages/Index.tsx (main dashboard and topic list)
