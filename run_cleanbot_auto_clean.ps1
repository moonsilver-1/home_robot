$ErrorActionPreference = 'Stop'

$repo = Split-Path -Parent $MyInvocation.MyCommand.Path
$linuxPath = "/mnt/" + $repo.Substring(0, 1).ToLower() + $repo.Substring(2).Replace('\', '/')

if ($args.Count -eq 0) {
    $room = 'living_room'
    $extraArgs = ''
} else {
    $room = $args[0]
    if ($args.Count -gt 1) {
        $extraArgs = ($args[1..($args.Count - 1)] -join ' ')
    } else {
        $extraArgs = ''
    }
}

$script = "cd '$linuxPath' && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch cleanbot_course_project static_room_cleaning_demo.launch gui:=true autostart_room:=$room autostart_delay:=15.0 $extraArgs"

wsl -d Ubuntu-20.04-Mirror -- bash -lc $script
