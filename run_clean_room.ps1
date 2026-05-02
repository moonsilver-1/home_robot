$ErrorActionPreference = 'Stop'

$repo = Split-Path -Parent $MyInvocation.MyCommand.Path
$linuxPath = "/mnt/" + $repo.Substring(0, 1).ToLower() + $repo.Substring(2).Replace('\', '/')

if ($args.Count -eq 0) {
    $room = 'living_room'
} else {
    $room = $args[0]
}

$script = @"
cd '$linuxPath' &&
source /opt/ros/noetic/setup.bash &&
source devel/setup.bash &&
if ! rostopic list >/tmp/cleanbot_rostopic_check.log 2>&1; then
  echo 'ROS master is not running.';
  echo 'Start the full simulator first:';
  echo '  .\run_cleanbot_full_demo.ps1';
  exit 1;
fi &&
if ! rostopic list | grep -q '^/move_base/goal$'; then
  echo '/move_base is not running, so the robot cannot move to cleaning goals yet.';
  echo 'Start the full SLAM/navigation stack first:';
  echo '  .\run_cleanbot_full_demo.ps1';
  exit 1;
fi &&
if ! rostopic list | grep -q '^/room_coverage_cleaner/status$'; then
  echo 'room_coverage_cleaner is not running, so /cleanbot/clean_room has no active cleaner to handle it.';
  echo 'Start the full demo first:';
  echo '  .\run_cleanbot_full_demo.ps1';
  exit 1;
fi &&
echo 'Requesting room cleaning: $room' &&
rostopic pub /cleanbot/clean_room std_msgs/String "data: $room" -1
"@

wsl -d Ubuntu-20.04-Mirror -- bash -lc $script
