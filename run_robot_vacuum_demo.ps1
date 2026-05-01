$ErrorActionPreference = 'Stop'

$repo = Split-Path -Parent $MyInvocation.MyCommand.Path
$linuxPath = "/mnt/" + $repo.Substring(0,1).ToLower() + $repo.Substring(2).Replace('\','/')

$argsString = $args -join ' '
$script = "cd '$linuxPath' && source /opt/ros/noetic/setup.bash && source devel/setup.bash && roslaunch cleanbot_course_project robot_vacuum_house_demo.launch $argsString"

wsl -d Ubuntu-20.04-Mirror -- bash -lc $script
