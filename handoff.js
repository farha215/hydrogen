const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell,
  HeadingLevel, AlignmentType, BorderStyle, WidthType, ShadingType,
  LevelFormat, PageNumber, Footer, TabStopType, TabStopPosition
} = require('docx');
const fs = require('fs');

const BLUE = "1a3a5c";
const LIGHT_BLUE = "dce8f5";
const GRAY = "f4f4f4";
const border = { style: BorderStyle.SINGLE, size: 1, color: "cccccc" };
const borders = { top: border, bottom: border, left: border, right: border };

function h1(text) {
  return new Paragraph({
    heading: HeadingLevel.HEADING_1,
    spacing: { before: 320, after: 120 },
    children: [new TextRun({ text, bold: true, size: 32, color: BLUE, font: "Arial" })]
  });
}

function h2(text) {
  return new Paragraph({
    heading: HeadingLevel.HEADING_2,
    spacing: { before: 240, after: 80 },
    children: [new TextRun({ text, bold: true, size: 26, color: BLUE, font: "Arial" })]
  });
}

function h3(text) {
  return new Paragraph({
    spacing: { before: 160, after: 60 },
    children: [new TextRun({ text, bold: true, size: 24, color: "333333", font: "Arial" })]
  });
}

function p(text, opts = {}) {
  return new Paragraph({
    spacing: { before: 60, after: 60 },
    children: [new TextRun({ text, size: 22, font: "Arial", ...opts })]
  });
}

function code(text) {
  return new Paragraph({
    spacing: { before: 40, after: 40 },
    indent: { left: 720 },
    shading: { fill: "f0f0f0", type: ShadingType.CLEAR },
    children: [new TextRun({ text, font: "Courier New", size: 18, color: "cc0000" })]
  });
}

function bullet(text, level = 0) {
  return new Paragraph({
    numbering: { reference: "bullets", level },
    spacing: { before: 40, after: 40 },
    children: [new TextRun({ text, size: 22, font: "Arial" })]
  });
}

function divider() {
  return new Paragraph({
    spacing: { before: 120, after: 120 },
    border: { bottom: { style: BorderStyle.SINGLE, size: 4, color: "cccccc", space: 1 } },
    children: []
  });
}

function twoColTable(rows, headerRow) {
  const makeRow = (col1, col2, isHeader = false) => new TableRow({
    children: [
      new TableCell({
        borders, width: { size: 4000, type: WidthType.DXA },
        shading: { fill: isHeader ? LIGHT_BLUE : "ffffff", type: ShadingType.CLEAR },
        margins: { top: 80, bottom: 80, left: 120, right: 120 },
        children: [new Paragraph({ children: [new TextRun({ text: col1, bold: isHeader, size: 20, font: "Arial" })] })]
      }),
      new TableCell({
        borders, width: { size: 5360, type: WidthType.DXA },
        shading: { fill: isHeader ? LIGHT_BLUE : "ffffff", type: ShadingType.CLEAR },
        margins: { top: 80, bottom: 80, left: 120, right: 120 },
        children: [new Paragraph({ children: [new TextRun({ text: col2, bold: isHeader, size: 20, font: "Arial" })] })]
      })
    ]
  });

  return new Table({
    width: { size: 9360, type: WidthType.DXA },
    columnWidths: [4000, 5360],
    rows: [
      ...(headerRow ? [makeRow(headerRow[0], headerRow[1], true)] : []),
      ...rows.map(r => makeRow(r[0], r[1]))
    ]
  });
}

const doc = new Document({
  numbering: {
    config: [
      { reference: "bullets", levels: [
        { level: 0, format: LevelFormat.BULLET, text: "\u2022", alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 720, hanging: 360 } } } },
        { level: 1, format: LevelFormat.BULLET, text: "\u25E6", alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 1080, hanging: 360 } } } }
      ]}
    ]
  },
  styles: {
    default: { document: { run: { font: "Arial", size: 22 } } },
    paragraphStyles: [
      { id: "Heading1", name: "Heading 1", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { size: 32, bold: true, font: "Arial", color: BLUE },
        paragraph: { spacing: { before: 320, after: 120 }, outlineLevel: 0 } },
      { id: "Heading2", name: "Heading 2", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { size: 26, bold: true, font: "Arial", color: BLUE },
        paragraph: { spacing: { before: 240, after: 80 }, outlineLevel: 1 } }
    ]
  },
  sections: [{
    properties: {
      page: { size: { width: 12240, height: 15840 }, margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 } }
    },
    footers: {
      default: new Footer({
        children: [new Paragraph({
          children: [
            new TextRun({ text: "Hydrogen AUV — Session Handoff    ", size: 18, font: "Arial", color: "888888" }),
            new TextRun({ children: [PageNumber.CURRENT], size: 18, font: "Arial", color: "888888" })
          ]
        })]
      })
    },
    children: [

      // ── Title ──
      new Paragraph({
        spacing: { before: 0, after: 80 },
        children: [new TextRun({ text: "Hydrogen AUV — Session Handoff Document", bold: true, size: 48, font: "Arial", color: BLUE })]
      }),
      new Paragraph({
        spacing: { before: 0, after: 320 },
        children: [new TextRun({ text: "Architecture changes, fixes applied, and next steps for the behaviour tree rewrite", size: 22, font: "Arial", color: "555555", italics: true })]
      }),
      divider(),

      // ── 1. WORKSPACE ──
      h1("1. Workspace setup"),
      p("The codebase has moved from ~/Desktop/hydrogen to a fresh workspace at ~/robosub. Packages are the same."),
      h2("Repo clone sequence"),
      code("cd ~/robosub/src"),
      code("git clone -b gazebo https://github.com/MITB-AUVTeam/hydrogen.git"),
      p("If custom_interfaces, control_system, and prequalification_bt are nested inside the cloned hydrogen folder, move them up:"),
      code("mv ~/robosub/src/hydrogen/custom_interfaces ~/robosub/src/"),
      code("mv ~/robosub/src/hydrogen/control_system ~/robosub/src/"),
      code("mv ~/robosub/src/hydrogen/prequalification_bt ~/robosub/src/"),
      h2("Build sequence"),
      code("cd ~/robosub"),
      code("source /opt/ros/humble/setup.bash"),
      code("colcon build"),
      code("source install/setup.bash"),
      p("Add the source line to .bashrc so it persists across terminals:"),
      code('echo "source ~/robosub/install/setup.bash" >> ~/.bashrc'),

      h2("Build warnings — fixed"),
      p("Stale paths to ~/auvws and ~/mobility workspaces were causing ~40 WARNING lines at build time. Fixed by commenting out three lines in ~/.bashrc:"),
      code("# Line 128: export GZ_SIM_RESOURCE_PATH=...auvws..."),
      code("# Line 129: source ~/auvws/install/setup.bash"),
      code("# Line 136: source ~/mobility/install/setup.bash"),
      p("Applied with: sed -i '128s/^/#/' ~/.bashrc  (repeated for lines 129 and 136)"),
      divider(),

      // ── 2. PLUGIN ──
      h1("2. libauv_absolute_depth.so plugin"),
      h2("What it does"),
      p("A Gazebo Ignition system plugin written by a team member. Class name: AUVAbsoluteDepth. Implements ISystemConfigure and ISystemPostUpdate. Every simulation step it reads the robot's world-frame Z position and publishes it as std_msgs/Float64 on /altimeter. This replaces any need for RTAB-Map or odometry for depth measurement."),

      h2("SDF parameters"),
      twoColTable([
        ["<topic>", "ROS topic to publish on. Default: /altimeter"]
      ], ["Parameter", "Description"]),
      new Paragraph({ children: [], spacing: { before: 120 } }),

      h2("Plugin tag location"),
      p("The plugin was already correctly defined in model/robot.gazebo. A duplicate was accidentally added to robot.xacro during debugging — that duplicate must be removed. Only robot.gazebo should have the plugin tag:"),
      code('<gazebo>'),
      code('  <plugin filename="libauv_absolute_depth.so" name="AUVAbsoluteDepth">'),
      code('    <topic>/altimeter</topic>'),
      code('  </plugin>'),
      code('</gazebo>'),

      h2("Why it was failing"),
      p("Two root causes:"),
      bullet("The .so was installed to share/hydrogen/plugins/ by setup.py. Gazebo system plugins need to be on IGN_GAZEBO_SYSTEM_PLUGIN_PATH, which was not being set anywhere in model_launch.py."),
      bullet("IGN_GAZEBO_RESOURCE_PATH (which was set) only covers models and worlds — NOT system plugins. Different env var entirely."),

      h2("Fix applied — model_launch.py"),
      p("Added AppendEnvironmentVariable for IGN_GAZEBO_SYSTEM_PLUGIN_PATH pointing to the install lib/ directory:"),
      code("install_dir = os.path.dirname(pkg_share)"),
      code("plugin_path = os.path.join(install_dir, 'lib')"),
      code("set_plugin_path = AppendEnvironmentVariable("),
      code("    name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',"),
      code("    value=plugin_path"),
      code(")"),
      code("ld.add_action(set_plugin_path)"),

      h2("Fix applied — setup.py"),
      p("Changed plugin install destination from share/ to lib/ so it lands where IGN_GAZEBO_SYSTEM_PLUGIN_PATH points:"),
      code("# Before (wrong):"),
      code("('share/' + package_name + '/plugins', glob('plugins/*')),"),
      code("# After (correct):"),
      code("('lib/' + package_name, glob('plugins/*.so')),"),

      h2("Temporary workaround (already applied)"),
      p("While the build system fix is being propagated, the .so was copied directly to Gazebo's system plugin directory:"),
      code("sudo cp /home/farha/Desktop/hydrogen/install/hydrogen/share/hydrogen/plugins/libauv_absolute_depth.so \\"),
      code("    /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/"),
      p("This works immediately without a rebuild. The setup.py fix makes it permanent."),
      divider(),

      // ── 3. ARCHITECTURE CHANGE ──
      h1("3. Architecture change — dropping RTAB-Map"),
      h2("Old architecture"),
      p("ZED camera -> RTAB-Map (rgbd_odometry) -> /odom -> behaviour tree. The BT used /odom for x/y/z position, IMU for yaw."),

      h2("New architecture"),
      p("No SLAM, no odometry, no XY position tracking. State estimation is purely:"),
      bullet("/imu — yaw/orientation only"),
      bullet("/altimeter — depth (z) only, from the libauv_absolute_depth plugin"),
      p("All navigation is visual servoing. The robot reacts to what it sees in /detections_3d, not where it thinks it is."),

      h2("Files that are unaffected"),
      twoColTable([
        ["yolo_node.py", "Only subscribes to /camera/RGB_image_raw/front. No odom dependency."],
        ["data_distance_node.py", "Depth from camera pixels. No odom dependency."],
        ["allocation_matrix.py", "Only sees /cmd_vel in, thrusters out. Fully isolated."],
        ["model_launch.py", "Updated for plugin path. RTAB_map_launch.py is now obsolete."]
      ], ["File", "Status"]),
      new Paragraph({ children: [], spacing: { before: 120 } }),

      h2("RTAB_map_launch.py — obsolete"),
      p("This launch file is no longer needed and should not be run. It launched: rgbd_sync_front, rgbd_sync_down, rgbd_odometry, rtabmap SLAM, rtabmap_viz, and global_planner. All of these are replaced by the depth plugin + IMU."),
      divider(),

      // ── 4. BT CHANGES ──
      h1("4. Behaviour tree — what needs changing"),
      p("This is the main outstanding work. The BT rewrite has been planned but not yet implemented. Pick up from here in the next session."),

      h2("Nodes that are BROKEN without odom"),
      twoColTable([
        ["AllSystemsOK", "Checks odom_received — must remove odom check, keep IMU check only"],
        ["WaitForStableOdom", "Existed only for RTAB-Map convergence delay — delete entirely"],
        ["SaveToBlackboard", "Saved odom x/y/z as T0 home — repurpose to save starting IMU yaw only"],
        ["DiveToDepth", "Sources z from odom.z — change to latest_altimeter"],
        ["DriveThruGate", "Tracks travel distance via odom x/y delta — replace with time-based drive"],
        ["NavigateTo", "atan2(target.y - cur.y, ...) — meaningless without XY, rewrite as blind timed return"],
        ["NavigateAround", "Computes XY waypoints around pole — rewrite as visual orbit"]
      ], ["Node", "Problem and fix needed"]),
      new Paragraph({ children: [], spacing: { before: 120 } }),

      h2("Nodes that survive unchanged"),
      twoColTable([
        ["IsObjectSeen", "Purely visual, reads /detections_3d. No odom."],
        ["Do360Turn", "Uses IMU yaw only. Fine."],
        ["StayStill", "Timer only. Fine."]
      ], ["Node", "Status"]),
      new Paragraph({ children: [], spacing: { before: 120 } }),

      h2("Rewrite plan — ordered by dependency"),
      p("Do these in order. Each step is a prerequisite for the next."),
      bullet("Step 1 — main.cpp + RobotContext: remove /odom subscription, remove odom_received/odom_sample_count, remove latest_odom. Wire latest_altimeter into getCurrentPose().z. Remove dead path_pub publisher. getCurrentPose() now returns (yaw from IMU, z from altimeter) only — no x/y."),
      bullet("Step 2 — AllSystemsOK: remove odom_received check. Keep imu_received only."),
      bullet("Step 3 — WaitForStableOdom: delete the node class entirely from bt_nodes.h and bt_nodes.cpp. Remove from prequalification.xml."),
      bullet("Step 4 — SaveToBlackboard: change from saving full pose to saving starting yaw (float) only. Used by NavigateTo for blind return direction."),
      bullet("Step 5 — DiveToDepth: change z source from odom.z to ctx->latest_altimeter.data."),
      bullet("Step 6 — DriveThruGate ALIGN phase: unchanged (visual servoing, works fine). DRIVE phase: replace odom distance tracking with time-based. Capture oz at alignment time as gate_drive_distance_. Drive for (gate_drive_distance_ / SURGE_SPEED) seconds."),
      bullet("Step 7 — NavigateAround: replace XY waypoint math with visual orbit. Keep pole at fixed lateral offset (ox ~ constant), sweep yaw in increments until 360 degrees completed."),
      bullet("Step 8 — NavigateTo: replace XY math with blind timed return. Turn 180 degrees from saved start yaw, surge forward for fixed duration."),
      bullet("Step 9 — prequalification.xml: remove WaitForStableOdom node, update SaveToBlackboard ports, verify all other node params still valid."),

      h2("Key coordinate frame reminder"),
      p("Camera optical frame — Z forward (depth), X right (horizontal offset), Y down. This is what /detections_3d publishes and what bt_nodes.h reads as ox, oy, oz. This frame is unchanged by the architecture switch."),

      h2("Gain constants — known redundancy"),
      p("K_YAW=4.5 appears independently in DriveThruGate, NavigateTo, and NavigateAround as separate constexpr values per class. Changing one does not affect the others. This is a known maintenance issue — flagged for future refactor as ROS 2 parameters but not blocking the current rewrite."),
      divider(),

      // ── 5. TOPIC MAP ──
      h1("5. Current topic map"),
      twoColTable([
        ["/camera/RGB_image_raw/front", "Gazebo -> yolo_node.py"],
        ["/camera/depth_image_raw/front", "Gazebo -> data_distance_node.py"],
        ["/camera_info_front", "Gazebo -> data_distance_node.py"],
        ["/detections_2d", "yolo_node.py -> data_distance_node.py"],
        ["/detections_3d", "data_distance_node.py -> BT executor"],
        ["/imu", "Gazebo -> BT executor (yaw only)"],
        ["/altimeter", "libauv_absolute_depth plugin -> BT executor (depth)"],
        ["/cmd_vel", "BT executor -> allocation_matrix.py"],
        ["/cmd_thrust", "allocation_matrix.py -> Gazebo thrusters"],
        ["/goal_pose", "BT executor -> (visualization only, no planner)"]
      ], ["Topic", "Flow"]),
      new Paragraph({ children: [], spacing: { before: 120 } }),
      p("/odom is no longer subscribed to by anything. /planned_path publisher in main.cpp is dead code — can be removed."),
      divider(),

      // ── 6. FILE CHECKLIST ──
      h1("6. Files changed this session"),
      twoColTable([
        ["model_launch.py", "Added IGN_GAZEBO_SYSTEM_PLUGIN_PATH env var"],
        ["setup.py", "Changed plugin install path from share/ to lib/"],
        ["robot.xacro", "Added then removed duplicate plugin tag (robot.gazebo already had it)"],
        ["~/.bashrc", "Commented out lines 128, 129, 136 (stale auvws/mobility paths)"]
      ], ["File", "Change"]),
      new Paragraph({ children: [], spacing: { before: 200 } }),

      p("Files NOT yet changed (pending BT rewrite):"),
      bullet("main.cpp"),
      bullet("bt_nodes.h"),
      bullet("bt_nodes.cpp"),
      bullet("prequalification.xml"),
    ]
  }]
});

Packer.toBuffer(doc).then(buf => {
  fs.writeFileSync('/mnt/user-data/outputs/hydrogen_handoff.docx', buf);
  console.log('done');
});
