#!/usr/bin/env python3
"""Exercise the actual trajectory server on an isolated ROS master."""
import os
import signal
import subprocess
import tempfile
import time


def main():
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:11385"
    import rosgraph
    import rospy
    from general_planner.msg import ExplorationTaskRequest, ExplorationTrajectory, PlannerPositionCommand
    from std_msgs.msg import Bool, Empty
    from traj_utils.msg import PolyTraj
    try:
        rosgraph.Master("/probe").getPid()
    except Exception:
        pass
    else:
        raise RuntimeError("isolated test port already in use")
    processes, logs = [], []

    def spawn(command):
        log = tempfile.TemporaryFile(mode="w+")
        logs.append(log)
        p = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p

    try:
        spawn(["roscore", "-p", "11385"])
        for _ in range(100):
            try:
                rosgraph.Master("/probe").getPid()
                break
            except Exception:
                time.sleep(.05)
        rospy.init_node("exploration_command_binding_test", disable_signals=True)
        messages = []
        rospy.Subscriber("/planning/exploration/command_bound", PlannerPositionCommand,
                         messages.append, queue_size=100)
        task = rospy.Publisher("/planning/exploration/task_request", ExplorationTaskRequest, queue_size=10)
        trajectory = rospy.Publisher("/planning/exploration/trajectory", ExplorationTrajectory, queue_size=10)
        enabled = rospy.Publisher("/planning/exploration/command_enabled", Bool, queue_size=10, latch=True)
        raw = rospy.Publisher("/planning/trajectory", PolyTraj, queue_size=10)
        heartbeat = rospy.Publisher("/planning/heartbeat", Empty, queue_size=10)
        server = spawn(["rosrun", "general_planner", "highspeed_traj_server",
                        "__name:=binding_test_server", "_require_task_binding:=true"])

        def wait(seconds, predicate=lambda: False):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if server.poll() is not None:
                    raise RuntimeError("trajectory server exited")
                heartbeat.publish(Empty())
                if predicate():
                    return True
                time.sleep(.01)
            return False

        assert wait(5, lambda: task.get_num_connections() and trajectory.get_num_connections()
                    and enabled.get_num_connections()), "subscribers did not start"

        def start(epoch):
            m = ExplorationTaskRequest()
            m.task_epoch = epoch; m.task_id = "binding-test:" + str(epoch); m.start = True
            task.publish(m)
            wait(.05)

        def command(epoch, trajectory_id):
            m = ExplorationTrajectory()
            m.task_epoch = epoch; m.task_id = "binding-test:" + str(epoch)
            p = m.position
            p.order = 7; p.traj_id = trajectory_id; p.start_time = rospy.Time.now(); p.duration = [1.]
            p.coef_x = [0.]*7 + [float(epoch)]
            p.coef_y = [0.]*8; p.coef_z = [0.]*7 + [1.5]
            return m

        enabled.publish(Bool(data=False)); wait(.1)
        start(1); trajectory.publish(command(1, 10)); wait(.1)
        assert not messages, "disabled task emitted a command"
        enabled.publish(Bool(data=True))
        assert wait(1, lambda: messages and messages[-1].task_epoch == 1), "enable lost queued current-task trajectory"
        enabled.publish(Bool(data=False)); wait(.1); start(2)
        trajectory.publish(command(2, 1)); wait(.1)  # simulate a relaunched planner's local counter
        enabled.publish(Bool(data=True))
        assert wait(1, lambda: messages and messages[-1].task_epoch == 2), "previous task counter rejected new task id 1"
        start(1); trajectory.publish(command(1, 99)); raw.publish(command(1, 99).position)
        invalid = command(2, 2); invalid.position.coef_x[-1] = float("nan")
        trajectory.publish(invalid); wait(.2)
        assert messages[-1].task_epoch == 2 and messages[-1].command.position.x == 2., "stale/raw/invalid trajectory changed current command"
        trajectory.publish(command(3, 1)); wait(.1)
        assert messages[-1].task_epoch == 2, "future trajectory executed before START"
        start(3)
        assert wait(1, lambda: messages[-1].task_epoch == 3), "START lost trajectory delivered by the other connection first"
        print("trajectory task binding, pause/enable ordering, restarted trajectory counter, stale/raw/invalid rejection: PASS", flush=True)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait()
        for log in logs:
            log.close()


if __name__ == "__main__":
    main()
