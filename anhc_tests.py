#!/usr/bin/env python3
"""
ANHC Complete Test Suite — Python (zsh/bash compatible)
Usage: python3 ~/robot_ws/anhc_tests.py
Run while: ros2 launch anhc_robot_bringup anhc_full_system.launch.py auto_goal:=false
"""
import subprocess, time, math, sys, json


def run(cmd, timeout=8):
    try:
        r = subprocess.run(
            cmd, shell=True, capture_output=True,
            text=True, timeout=timeout
        )
        return r.stdout.strip()
    except subprocess.TimeoutExpired:
        return ""
    except Exception as e:
        return f"ERROR: {e}"


def section(title):
    print(f"\n{'═'*50}")
    print(f"  {title}")
    print('═'*50)


def check(label, condition, detail=""):
    icon = "✅" if condition else "❌"
    print(f"  {icon}  {label}")
    if detail:
        print(f"       {detail}")
    return condition


# ══════════════════════════════════════════════
# TEST 5 — Nodes and Topics
# ══════════════════════════════════════════════
def test5_nodes_topics():
    section("TEST 5 — Nodes & Topics")
    nodes  = run("ros2 node list 2>/dev/null")
    topics = run("ros2 topic list 2>/dev/null")
    ctrl   = run("ros2 control list_controllers 2>/dev/null")

    results = []
    required_kw  = ['map_handler','planner','path_follower',
                    'robot_state_pub','controller_manager']
    required_top = ['/anhc/scan','/anhc/odom','/anhc/map',
                    '/anhc/planned_path','/anhc/cmd_vel','/tf']

    print("\n  Nodes:")
    for kw in required_kw:
        match = [n for n in nodes.splitlines() if kw in n]
        ok = len(match) > 0
        results.append(ok)
        check(match[0] if match else f'<missing: {kw}>', ok)

    print("\n  Topics:")
    for t in required_top:
        ok = t in topics
        results.append(ok)
        check(t, ok)

    print("\n  Controllers:")
    jsb_ok = 'joint_state_broadcaster' in ctrl and 'active' in ctrl
    ddc_ok = 'diff_drive' in ctrl and 'active' in ctrl
    check("joint_state_broadcaster active", jsb_ok)
    check("diff_drive_controller active",   ddc_ok)
    results += [jsb_ok, ddc_ok]

    return all(results)


# ══════════════════════════════════════════════
# TEST 6 — Sensor Rates
# ══════════════════════════════════════════════
def test6_sensor_rates():
    section("TEST 6 — Sensor Data Rates")

    sensors = [
        ('/anhc/odom',           10, 35, 'Odometry'),
        ('/anhc/scan',            4, 20, '2D LiDAR'),
        ('/tf',                  20, 90, 'TF'),
    ]

    results = []
    for topic, lo, hi, name in sensors:
        cmd = f"timeout 6 ros2 topic hz {topic} 2>/dev/null"
        out = run(cmd, timeout=8)
        rates = []
        for line in out.splitlines():
            if 'average rate:' in line:
                try:
                    rates.append(float(line.split(':')[1].strip()))
                except:
                    pass
        if rates:
            avg = sum(rates) / len(rates)
            ok  = lo <= avg <= hi
            results.append(ok)
            check(f"{name}: {avg:.1f} Hz (expect {lo}–{hi})", ok)
        else:
            results.append(False)
            check(f"{name}: NO DATA", False,
                  f"Topic {topic} not publishing")

    return all(results)


# ══════════════════════════════════════════════
# TEST 9 — Map Building
# ══════════════════════════════════════════════
def test9_map():
    section("TEST 9 — Map Building")

    out = run("ros2 topic echo --once /anhc/map 2>/dev/null", timeout=10)

    res_ok = 'resolution' in out
    w_ok   = 'width: 200' in out
    h_ok   = 'height: 200' in out

    check("Map resolution present", res_ok)
    check("Map width = 200",        w_ok)
    check("Map height = 200",       h_ok)

    # Count values in data array
    free = occ = unk = 0
    in_data = False
    for line in out.splitlines():
        if 'data:' in line:
            in_data = True
        if in_data:
            for val in line.replace(',', ' ').replace('[', ' ').replace(']', ' ').split():
                try:
                    v = int(val)
                    if v == 0:    free += 1
                    elif v == 100: occ += 1
                    elif v == -1:  unk += 1
                except:
                    pass

    print(f"\n  Map cell counts:")
    print(f"    Free     (0)  : {free}")
    print(f"    Occupied (100): {occ}")
    print(f"    Unknown  (-1) : {unk}")

    data_ok = (free + occ + unk) > 100
    occ_ok  = occ > 0

    check("Map has data (>100 cells parsed)", data_ok)
    check("Obstacles detected (occ > 0)",     occ_ok,
          "Drive robot near walls if 0" if not occ_ok else "")

    return res_ok and w_ok and h_ok and data_ok


# ══════════════════════════════════════════════
# TEST 10 — A* Planning
# ══════════════════════════════════════════════
def test10_planning():
    section("TEST 10 — A* Path Planning")

    # Send goal
    print("  Sending goal (7.0, 0.0)...")
    run("ros2 topic pub --once /anhc/goal geometry_msgs/msg/PoseStamped "
        '"{header: {frame_id: odom}, '
        'pose: {position: {x: 7.0, y: 0.0, z: 0.0}, '
        'orientation: {w: 1.0}}}" 2>/dev/null',
        timeout=5)

    print("  Waiting 5s for A* to compute...")
    time.sleep(5)

    # Read path
    out = run("ros2 topic echo --once /anhc/planned_path 2>/dev/null",
              timeout=8)

    waypoints = out.count('position:')
    found     = waypoints > 10

    check(f"Path published ({waypoints} waypoints)", found,
          "Check planner logs if 0" if not found else
          f"{waypoints * 0.1:.1f}m path length")

    if found:
        # Verify first and last points roughly match start and goal
        lines = out.splitlines()
        xs = []
        for line in lines:
            if 'x:' in line and 'stamp' not in line:
                try:
                    xs.append(float(line.split(':')[1].strip()))
                except:
                    pass
        if xs:
            print(f"    Path X range: {min(xs):.2f} → {max(xs):.2f}")
            range_ok = (max(xs) - min(xs)) > 5.0
            check("Path spans >5m in X", range_ok)
            return found and range_ok

    return found


# ══════════════════════════════════════════════
# TEST 11 — Full Navigation
# ══════════════════════════════════════════════
def test11_navigation():
    section("TEST 11 — Full Autonomous Navigation")

    GOAL_X, GOAL_Y = 7.0, 0.0
    TOLERANCE       = 0.5
    MAX_WAIT        = 90

    print(f"  Goal: ({GOAL_X}, {GOAL_Y}) | Timeout: {MAX_WAIT}s")
    print(f"\n  {'Time':>6}  {'X':>8}  {'Y':>8}  {'Dist':>8}")
    print(f"  {'─'*36}")

    run(f"ros2 topic pub --once /anhc/goal geometry_msgs/msg/PoseStamped "
        f'"{{\\"header\\": {{\\"frame_id\\": \\"odom\\"}}, '
        f'\\"pose\\": {{\\"position\\": {{\\"x\\": {GOAL_X}, '
        f'\\"y\\": {GOAL_Y}, \\"z\\": 0.0}}, '
        f'\\"orientation\\": {{\\"w\\": 1.0}}}}}}" 2>/dev/null',
        timeout=5)

    start    = time.time()
    reached  = False
    last_pos = (-7.0, 0.0)
    positions = []

    while time.time() - start < MAX_WAIT:
        out = run("ros2 topic echo --once --no-arr /anhc/odom 2>/dev/null",
                  timeout=3)
        x = y = None
        in_pos = False
        for line in out.splitlines():
            if 'position:' in line:
                in_pos = True
            if in_pos:
                if 'x:' in line and x is None:
                    try: x = float(line.split(':')[1])
                    except: pass
                if 'y:' in line and y is None:
                    try: y = float(line.split(':')[1])
                    except: pass
            if x is not None and y is not None:
                break

        if x is not None and y is not None:
            dist = math.hypot(GOAL_X - x, GOAL_Y - y)
            t    = time.time() - start
            positions.append((t, x, y, dist))
            last_pos = (x, y)
            print(f"  {t:>6.1f}s  {x:>8.3f}  {y:>8.3f}  {dist:>7.3f}m")
            if dist < TOLERANCE:
                reached = True
                break
        time.sleep(2)

    print()
    if reached:
        elapsed = positions[-1][0]
        max_x   = max(p[1] for p in positions)
        check(f"Goal reached in {elapsed:.1f}s", True)
        check(f"Robot crossed arena (max_x={max_x:.2f})", max_x > 0)
        return True
    else:
        moved = abs(last_pos[0] - (-7.0)) > 0.5
        check("Goal reached", False,
              f"Last pos: ({last_pos[0]:.2f},{last_pos[1]:.2f})")
        check("Robot moved at all", moved)
        return False


# ══════════════════════════════════════════════
# TEST 12 — Multi-Goal
# ══════════════════════════════════════════════
def test12_multigoal():
    section("TEST 12 — Multi-Goal Replanning")

    goals = [(3.0, 5.0, "upper-right"),
             (-2.0, -4.0, "lower-left"),
             (7.0, 0.0,  "far-right")]

    results = []
    for gx, gy, name in goals:
        print(f"\n  Goal: {name} ({gx}, {gy})")
        run(f"ros2 topic pub --once /anhc/goal "
            f"geometry_msgs/msg/PoseStamped "
            f'"{{\\"header\\": {{\\"frame_id\\": \\"odom\\"}}, '
            f'\\"pose\\": {{\\"position\\": {{\\"x\\": {gx}, '
            f'\\"y\\": {gy}, \\"z\\": 0.0}}, '
            f'\\"orientation\\": {{\\"w\\": 1.0}}}}}}" 2>/dev/null',
            timeout=5)

        time.sleep(4)

        out = run("ros2 topic echo --once /anhc/planned_path 2>/dev/null",
                  timeout=6)
        wps = out.count('position:')
        ok  = wps > 5
        results.append(ok)
        check(f"Path found ({wps} waypoints)", ok)

        if ok:
            print(f"    Waiting 20s to travel...")
            time.sleep(20)

    return all(results)


# ══════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════
def main():
    print("\n" + "█"*50)
    print("  ANHC ROBOT — COMPLETE TEST SUITE")
    print("  Make sure simulation is running!")
    print("█"*50)

    all_results = {}

    all_results['T5  Nodes & Topics']    = test5_nodes_topics()
    all_results['T6  Sensor Rates']      = test6_sensor_rates()
    all_results['T9  Map Building']      = test9_map()
    all_results['T10 A* Planning']       = test10_planning()

    run_nav = input("\n  Run navigation tests T11+T12? (y/n): ").strip().lower()
    if run_nav == 'y':
        all_results['T11 Full Navigation'] = test11_navigation()
        all_results['T12 Multi-Goal']      = test12_multigoal()

    # Final report
    section("FINAL REPORT")
    passed = sum(all_results.values())
    total  = len(all_results)
    for name, ok in all_results.items():
        check(name, ok)

    print(f"\n  Score: {passed}/{total}")
    if passed == total:
        print("\n  🎉 ALL TESTS PASSED — Project complete!")
    else:
        failed = [n for n, ok in all_results.items() if not ok]
        print(f"\n  ⚠️  Fix these: {', '.join(failed)}")
    print()


if __name__ == '__main__':
    main()


def reset_robot():
    """Teleport robot back to start position (-7, 0)."""
    print("\n  [Reset] Returning robot to start (-7, 0)...")
    run("ros2 topic pub --once /anhc/cmd_vel geometry_msgs/msg/Twist "
        '"{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null')
    # Use Gazebo set_pose service
    run('gz service -s /world/anhc_simple_world/set_pose '
        '--reqtype gz.msgs.Pose '
        '--reptype gz.msgs.Boolean '
        '--timeout 2000 '
        '--req "name: \\"anhc_robot\\" position {x: -7 y: 0 z: 0.15}" '
        '2>/dev/null')
    import time; time.sleep(2)
    pos = run("ros2 topic echo --once --no-arr /anhc/odom 2>/dev/null")
    print(f"  [Reset] Done")
